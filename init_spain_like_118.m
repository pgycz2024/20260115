function [mpc, gen_data, wind_data, storage_data, tie_lines_idx] = init_spain_like_118()

    fprintf('正在构建虚拟西班牙电网 (含脆弱联络线模型)...\n');
    mpc = loadcase('case118');
    
    gen_bus = mpc.gen(:, 1);
    num_gen = size(mpc.gen, 1);
    
    %% 1. 区域定义 (制造物理隔离隐患)
    bus_zone = zeros(size(mpc.bus, 1), 1);
    bus_zone(1:79) = 1;      % Zone A (Spain)
    bus_zone(80:end) = 2;    % Zone B (Portugal)
    
    % 识别联络线 (Tie Lines)
    tie_lines_idx = [];
    for i = 1:size(mpc.branch, 1)
        f_bus = mpc.branch(i, 1);
        t_bus = mpc.branch(i, 2);
        if bus_zone(f_bus) ~= bus_zone(t_bus)
            tie_lines_idx = [tie_lines_idx; i];
        end
    end
    fprintf('  >> 识别到 %d 条跨区联络线，将在仿真中途切断。\n', length(tie_lines_idx));
    
    %% 2. 机组类型识别
    idx_Portugal_Gen = find(gen_bus >= 80);
    idx_Spain_Gen = find(gen_bus < 80);
    
    gen_type = ones(num_gen, 1); % 默认为 1 (火电)
    
    % (1) 设置黑启动 (BS)
    bs_buses = [25, 69, 89]; 
    idx_BS = find(ismember(gen_bus, bs_buses));
    gen_type(idx_BS) = 2; 
    
    % (2) 设置新能源 (Renew)
    rng(428); 
    idx_Wind = idx_Portugal_Gen(randperm(length(idx_Portugal_Gen), floor(0.9 * length(idx_Portugal_Gen))));
    idx_Wind = setdiff(idx_Wind, idx_BS);
    idx_PV = idx_Spain_Gen(randperm(length(idx_Spain_Gen), floor(0.7 * length(idx_Spain_Gen))));
    idx_PV = setdiff(idx_PV, idx_BS);
    idx_Renew = [idx_PV; idx_Wind];
    gen_type(idx_Renew) = 3;
    
    % (3) 定义火电机组索引，供后续参数设置使用
    idx_Thermal = find(gen_type == 1);
    
    %% 3. 机组参数设置 (融入 PPSR 启动特性)
    gen_data = struct();
    gen_data.type = gen_type;
    gen_data.Pg_max = mpc.gen(:, 9);
    gen_data.Pg_min = zeros(num_gen, 1);
    
    % --- 启动特性参数 (源自 PPSR 思想) ---
    % 1. 启动功率 (P_crank)
    gen_data.P_crank = zeros(num_gen, 1);
    
    % 火电: 需要巨大的辅机启动功率 (8%容量)
    gen_data.P_crank(idx_Thermal) = 0.08 * gen_data.Pg_max(idx_Thermal); 
    % 新能源: 需要较小的控制用电 (1%容量)
    gen_data.P_crank(idx_Renew) = 0.01 * gen_data.Pg_max(idx_Renew);
    % BS: 不需要倒送电
    gen_data.P_crank(idx_BS) = 0;
    
    % 2. 启动耗能期 (T_crank)
    gen_data.T_crank = zeros(num_gen, 1);
    gen_data.T_crank(idx_Thermal) = 2; % 火电启动前2个Step纯耗电
    gen_data.T_crank(idx_Renew) = 1;   % 新能源启动前1个Step耗电
    gen_data.T_crank(idx_BS) = 0;      
    
    % --- 极端容量与爬坡参数 ---
    gen_data.Pg_max(idx_Wind) = gen_data.Pg_max(idx_Wind) * 4.0; 
    gen_data.Pg_max(idx_PV) = gen_data.Pg_max(idx_PV) * 2.5;
    gen_data.Pg_max(idx_BS) = 200; 
    
    gen_data.Ramp = 0.05 * gen_data.Pg_max;
    gen_data.Ramp(idx_BS) = 0.1 * gen_data.Pg_max(idx_BS);
    
    % --- 启动延迟 (指令发出到开始动作的时间) ---
    gen_data.T_start_hot = zeros(num_gen, 1);
    gen_data.T_start_hot(idx_Renew) = 1;
    
    % 对火电机组随机分配延迟
    for i = 1:length(idx_Thermal)
        gen_data.T_start_hot(idx_Thermal(i)) = randi([3, 6]); 
    end
    
    %% 4. 风光曲线 (前期平稳，为断线做铺垫)
    T_total = 40;
    n_renew = length(idx_Renew);
    t = linspace(0, pi, T_total)'; 
    forecast_matrix = zeros(n_renew, T_total);
    
    for i = 1:n_renew
        g_idx = idx_Renew(i);
        p_cap = gen_data.Pg_max(g_idx);
        if ismember(g_idx, idx_PV)
            curve = sin(t).^1.2; 
        else
            curve = 0.8 + 0.1*sin(2*t); 
        end
        curve = max(0, min(1, curve));
        forecast_matrix(i, :) = curve * p_cap;
    end
    
    wind_data = struct();
    wind_data.idx = idx_Renew;
    wind_data.forecast = forecast_matrix;
    wind_data.error_bound = 0.05 * wind_data.forecast; 
    
    %% 5. 储能
    storage_data = struct();
    storage_data.idx = idx_Renew; 
    storage_data.P_max = 0.30 * gen_data.Pg_max(idx_Renew); 
    storage_data.E_max = 4 * storage_data.P_max;
    
    fprintf('初始化完成: %d 台机组 (BS: %d, Wind: %d, Thermal: %d)\n', ...
        num_gen, length(idx_BS), length(idx_Renew), length(idx_Thermal));

end
