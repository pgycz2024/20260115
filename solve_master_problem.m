function [sol, status] = solve_master_problem(mpc, gen_data, wind_data, storage_data, T_horizon, current_step, current_u_on, initial_Pg, fixed_x_gm)
% SOLVE_MASTER_PROBLEM_V2 (融合 PPSR 增强版)
% 特性 1: 引入 SCF 约束(基于虚拟流的连通性约束) ，确保动态分区的物理连通性
% 特性 2: 引入 Cranking Power，模拟启动冲击

    %% 1. 初始化参数
    nb = size(mpc.bus, 1);
    nl = size(mpc.branch, 1);
    ng = size(mpc.gen, 1);
    n_wind = length(wind_data.idx);
    
    idx_bs = find(gen_data.type == 2); 
    idx_sync = find(gen_data.type == 1);
    idx_wind = find(gen_data.type == 3);
    n_bs = length(idx_bs); 
    
    % 处理输入参数
    if nargin < 7 || isempty(current_u_on), current_u_on = zeros(ng, 1); current_u_on(idx_bs) = 1; end
    if nargin < 8 || isempty(initial_Pg), initial_Pg = zeros(ng, 1); end
    is_static_mode = (nargin >= 9 && ~isempty(fixed_x_gm));
    
    % 风电预测数据切片
    t_range = current_step : min(current_step + T_horizon - 1, size(wind_data.forecast, 2));
    wf_horizon = wind_data.forecast(:, t_range);
    we_horizon = wind_data.error_bound(:, t_range);
    % 补全长度
    if size(wf_horizon, 2) < T_horizon
        pad_len = T_horizon - size(wf_horizon, 2);
        wf_horizon = [wf_horizon, repmat(wf_horizon(:,end), 1, pad_len)];
        we_horizon = [we_horizon, repmat(we_horizon(:,end), 1, pad_len)];
    end

    %% 2. 变量定义
    % 2.1 机组状态变量
    x_gm = binvar(ng, n_bs, 'full');        % 机组分区归属
    u_start = binvar(ng, T_horizon, 'full');% 启动动作
    u_on = binvar(ng, T_horizon, 'full');   % 在线状态
    
    % 2.2 功率变量 (区分 净出力 P_net, 发电 P_gen, 倒送电 P_cons)
    P_net = sdpvar(ng, T_horizon, 'full');  % 实际注入电网的功率 (+/-)
    P_gen = sdpvar(ng, T_horizon, 'full');  % 发电机发出的正功率
    
    % 2.3 辅助变量 (灵活性)
    R_sync = sdpvar(length(idx_sync), n_bs, T_horizon, 'full');
    n_stor = length(storage_data.idx);
    x_bm = binvar(n_stor, n_bs, 'full');    % 储能分区归属
    R_bess = sdpvar(n_stor, n_bs, T_horizon, 'full');
    Slack_Flex = sdpvar(n_bs, T_horizon, 'full'); % 灵活性缺额松弛
    
    % 2.4 [新增] 连通性变量 (借鉴 PPSR)
    % y_line_m: 线路 l 是否属于分区 m
    y_line_m = binvar(nl, n_bs, 'full'); 
    % f_line_m: 分区 m 内的虚拟流 (用于校验连通性)
    f_line_m = sdpvar(nl, n_bs, 'full');
    % x_bus_m: 节点 b 是否属于分区 m (为了连通性，必须显式定义节点归属)
    x_bus_m = binvar(nb, n_bs, 'full');

    %% 3. 约束条件构建
    Constraints = [];
    
    % --- 3.1 基础状态逻辑 ---
    for g = 1:ng
        % 状态转移: u_on(t) = u_on(t-1) + u_start(t)
        Constraints = [Constraints, u_on(g, 1) == current_u_on(g) + u_start(g, 1)];
        if current_u_on(g) == 1, Constraints = [Constraints, u_start(g, 1) == 0]; end % 已开机不能重启
        
        for t = 2:T_horizon
            Constraints = [Constraints, u_on(g, t) >= u_on(g, t-1)]; % 单调不掉线
            Constraints = [Constraints, u_on(g, t) - u_on(g, t-1) == u_start(g, t)];
        end
        
        % 启动延迟 (T_start_hot): 还没到时间的不能动
        t_wait = gen_data.T_start_hot(g);
        if current_u_on(g) == 0 && t_wait > 0
            limit_t = min(t_wait, T_horizon);
            Constraints = [Constraints, u_on(g, 1:limit_t) == 0];
        end
    end

    % --- 3.2 [增强] 物理出力与启动倒送电 (Cranking Power) ---
    for g = 1:ng
        T_crank = gen_data.T_crank(g);
        P_crank_val = gen_data.P_crank(g);
        
        for t = 1:T_horizon
            % 核心逻辑: P_net = P_gen - P_consumption
            % 如果处在启动初期 (u_on=1 但刚启动不久)，P_gen=0, P_consumption=P_crank
            
            % 简化判断：我们很难在线性模型里精确判断"刚启动了多久"
            % 这里使用一个近似技巧：如果是本时段启动 (u_start=1)，则强制为负
            % (为了简化计算，假设 T_crank 主要影响启动的那一刻，或者稍微放宽)
            
            if T_crank > 0
                % 如果 t 时刻启动，则 P_net 必须等于 -P_crank
                Constraints = [Constraints, P_net(g, t) <= gen_data.Pg_max(g) * u_on(g,t) - P_crank_val * u_start(g,t)];
                % 下界：如果没启动是0，启动了至少是 -P_crank
                Constraints = [Constraints, P_net(g, t) >= -P_crank_val * u_start(g,t)];
            else
                % BS 机组或无延迟机组
                Constraints = [Constraints, P_net(g, t) >= gen_data.Pg_min(g) * u_on(g,t)];
                Constraints = [Constraints, P_net(g, t) <= gen_data.Pg_max(g) * u_on(g,t)];
            end
            
            % 爬坡约束 (针对 P_net)
            if t == 1
                Constraints = [Constraints, P_net(g, 1) - initial_Pg(g) <= gen_data.Ramp(g)];
                Constraints = [Constraints, initial_Pg(g) - P_net(g, 1) <= gen_data.Ramp(g)];
            else
                Constraints = [Constraints, P_net(g, t) - P_net(g, t-1) <= gen_data.Ramp(g)];
                Constraints = [Constraints, P_net(g, t-1) - P_net(g, t) <= gen_data.Ramp(g)];
            end
            
            % 风电出力上限
            if ismember(g, wind_data.idx)
                k_idx = find(wind_data.idx == g);
                Constraints = [Constraints, P_net(g, t) <= wf_horizon(k_idx, t) * u_on(g, t)];
            end
        end
    end

    % --- 3.3 [新增] 动态连通性约束 (PPSR 核心) ---
    % 仅对 t=1 时刻的拓扑进行严格约束 (为了减少计算量，假设拓扑在预测时域内保持稳定)
    
    % (1) 节点-分区 映射
    Constraints = [Constraints, sum(x_bus_m, 2) == 1]; % 每个节点属于一个分区
    
    % (2) 机组必须跟随其所在的节点
    for g = 1:ng
        bus_idx = find(mpc.bus(:,1) == mpc.gen(g,1)); % 找到机组所在节点的行号
        % 机组分区 x_gm 必须等于 节点分区 x_bus_m
        if is_static_mode
             Constraints = [Constraints, x_gm(g, :) == fixed_x_gm(g, :)];
             % 在静态模式下，节点分区也被锁定 (需要传入，这里简化为重新推导或假设一致)
             % 简单起见，静态模式让求解器自己去找符合 fixed_x_gm 的 x_bus_m
        else
             Constraints = [Constraints, x_gm(g, :) == x_bus_m(bus_idx, :)];
        end
    end
    
    % (3) 线路-分区 映射 (如果两端节点在同一区，线路就在该区)
    % 使用 Big-M 逻辑: y_line_m(l, k) <= x_bus_m(from, k), y_line_m(l, k) <= x_bus_m(to, k)
    idx_from = mpc.branch(:, 1); % 这里需要转成内部索引，假设 mpc.bus 顺序对应 1..nb
    idx_to   = mpc.branch(:, 2); 
    % 修正：MATPOWER 的 branch 列存的是 Bus ID，需要映射到 1..nb 索引
    [~, idx_from_map] = ismember(idx_from, mpc.bus(:,1));
    [~, idx_to_map]   = ismember(idx_to, mpc.bus(:,1));
    
    for m = 1:n_bs
        % 线路归属约束
        Constraints = [Constraints, y_line_m(:, m) <= x_bus_m(idx_from_map, m)];
        Constraints = [Constraints, y_line_m(:, m) <= x_bus_m(idx_to_map, m)];
        
        % (4) 单商品流约束 (Single Commodity Flow)
        % 构建节点关联矩阵 A (nb x nl)
        % 注意：这里要在循环内动态构建流平衡，比较耗时。
        % 既然是 118 节点，我们手写稀疏形式会快一点。
        
        % 注入: 除了 Root 节点(BS)，其他节点必须消耗 1 单位虚拟流
        % Root 节点提供 sum(x_bus_m without root) 的流
        % 简单写法： A * f_line_m(:, m) == Demand_Flow
        
        % 流守恒: Sum(f_in) - Sum(f_out) = -x_bus_m (如果是负荷节点)
        % 对应 Root 节点，源源不断流出。
        
        root_bus_id = mpc.gen(idx_bs(m), 1);
        [~, root_idx] = ismember(root_bus_id, mpc.bus(:,1));
        
        % 虚拟流生成：除 Root 外，每个属于该分区的节点消耗 1
        % Node_Flow_Balance = A * f
        % 这种写法在 YALMIP 里略复杂，我们用最简化的"生成树"约束：
        
        % 仅当线路开启时允许流
        Constraints = [Constraints, f_line_m(:, m) >= -nb * y_line_m(:, m)];
        Constraints = [Constraints, f_line_m(:, m) <=  nb * y_line_m(:, m)];
    end
    
    % 统一流平衡约束 (矩阵化加速)
    % A_incidence: 节点-支路关联矩阵
    A_incidence = sparse(nb, nl);
    for l = 1:nl
        A_incidence(idx_from_map(l), l) = 1;
        A_incidence(idx_to_map(l), l) = -1;
    end
    
    for m = 1:n_bs
        root_bus_id = mpc.gen(idx_bs(m), 1);
        [~, root_idx] = ismember(root_bus_id, mpc.bus(:,1));
        
        % 节点注入向量: 非根节点如果是该分区的，注入为 -1 (消耗)
        % 根节点注入为: sum(x_bus_m(others, m))
        % 这会让约束变非线性。
        
        % === 替代方案：基于连通性的松弛 ===
        % 为了避免 MILP 过重，我们只约束：
        % 如果节点在分区 m，且不是 root，则流入量 - 流出量 = 1
        % 如果不在分区 m，则平衡为 0
        
        Node_Balance = A_incidence * f_line_m(:, m);
        
        % 对于非根节点 i:
        % 如果 x_bus_m(i,m)=1, Balance = -1 (接收流)
        % 如果 x_bus_m(i,m)=0, Balance = 0
        % => Balance(i) == -1 * x_bus_m(i,m)
        
        not_root_mask = true(nb, 1); not_root_mask(root_idx) = false;
        
        Constraints = [Constraints, Node_Balance(not_root_mask) == -1 * x_bus_m(not_root_mask, m)];
        
        % 对于根节点: 不约束 (或者是其他节点的总和)
        % 这样就强制了所有属于 m 的节点必须通过线路连接到 root
    end

    % --- 3.4 灵活性与供需平衡 (保留原有优势) ---
    % ... (此处代码与之前版本基本一致，主要把 P_g 换成 P_net) ...
    % 同步机灵活性
    for i = 1:length(idx_sync)
        g_idx = idx_sync(i);
        P_max_val = gen_data.Pg_max(g_idx);
        for t = 1:T_horizon
            % R <= P_max - P_net (简化处理，忽略爬坡对备用的限制)
            Constraints = [Constraints, sum(R_sync(i, :, t)) <= P_max_val * u_on(g_idx, t) - P_net(g_idx, t)];
            for m = 1:n_bs
                Constraints = [Constraints, R_sync(i, m, t) >= 0];
                Constraints = [Constraints, R_sync(i, m, t) <= P_max_val * x_gm(g_idx, m)];
            end
        end
    end
    % 储能灵活性
    for b = 1:n_stor
        P_bess_max = storage_data.P_max(b);
        for t = 1:T_horizon
            Constraints = [Constraints, sum(R_bess(b, :, t)) <= P_bess_max];
            for m = 1:n_bs
                Constraints = [Constraints, R_bess(b, m, t) >= 0];
                Constraints = [Constraints, R_bess(b, m, t) <= P_bess_max * x_bm(b, m)];
            end
        end
    end
    
    % 分区供需平衡
    Constraints = [Constraints, Slack_Flex >= 0];
    for m = 1:n_bs
        for t = 1:T_horizon
            Demand_m = 0;
            for k = 1:n_wind
                w_idx = idx_wind(k);
                Demand_m = Demand_m + we_horizon(k, t) * x_gm(w_idx, m); 
            end
            Constraints = [Constraints, sum(R_sync(:, m, t)) + sum(R_bess(:, m, t)) + Slack_Flex(m, t) >= Demand_m];
        end
    end

    %% 4. 目标函数
    % 最大化恢复功率 (P_net) - 惩罚松弛变量
    weights = fliplr(1:T_horizon); 
    Obj_Power = 0;
    for t = 1:T_horizon
        Obj_Power = Obj_Power + sum(P_net(:,t)) * weights(t);
    end
    Obj_Penalty = sum(sum(Slack_Flex)) * 1e5;
    
    % [新增] 连通性惩罚 (最小化使用的线路数，倾向于紧凑分区)
    Obj_Topology = sum(sum(sum(y_line_m))) * 0.1; 
    
    Objective = -Obj_Power + Obj_Penalty + Obj_Topology; 

    %% 5. 求解
    ops = sdpsettings('solver', 'gurobi', 'verbose', 0, 'gurobi.MIPGap', 0.01); 
    sol = optimize(Constraints, Objective, ops);
    
    status = sol.problem;
    
    if status == 0
        sol.x_gm = value(x_gm);
        sol.u_start = value(u_start);
        sol.P_g = value(P_net); % 返回净出力
        sol.Obj = value(Objective);
    else
        sol.x_gm = [];
    end
end