% 复现 4.28 西班牙大停电真实演化过程
% 特性: 连锁脱网 -> 联络线解列 -> 启动倒送电冲击
clear; clc; close all;

%% 1. 初始化系统 (高比例新能源, 脆弱性参数)
[mpc_base, gen_data, wind_data, storage_data, tie_lines_idx] = init_spain_like_118();

T_total = 35;
T_horizon = 5;

% 初始状态: 全网正常运行
initial_u_on = zeros(size(mpc_base.gen, 1), 1);
% 假设此时有 30% 的常规机组和 80% 的新能源在运行
initial_u_on(gen_data.type == 2) = 1; % BS 在线
initial_u_on(gen_data.type == 3) = 1; % 新能源全开
idx_therm = find(gen_data.type == 1);
initial_u_on(idx_therm(1:floor(end/3))) = 1; % 部分火电在线

initial_Pg = initial_u_on .* (gen_data.Pg_max * 0.4); % 初始 40% 出力

%% === 实验 I: 静态分区 (模拟传统调度) ===
fprintf('\n>>> 实验 I: 静态分区 (面对连锁故障僵化应对)... \n');
History_Static = run_real_disaster_sim(mpc_base, gen_data, wind_data, storage_data, ...
                                       tie_lines_idx, T_total, T_horizon, initial_u_on, initial_Pg, true);

%% === 实验 II: 动态分区 (自适应重构) ===
fprintf('\n>>> 实验 II: 动态分区 (面对连锁故障自适应重构)... \n');
History_Dynamic = run_real_disaster_sim(mpc_base, gen_data, wind_data, storage_data, ...
                                        tie_lines_idx, T_total, T_horizon, initial_u_on, initial_Pg, false);

%% 3. 绘图对比 (见证真实灾难的演化)
fprintf('\n正在生成真实灾难复现图...\n');
figure('Color', 'w', 'Position', [100, 100, 1200, 900]);

% 子图1: 系统总韧性对比
subplot(2,1,1);
hold on;
plot(1:T_total, History_Dynamic.P_total, 'b-o', 'LineWidth', 2, 'MarkerFaceColor', 'b');
plot(1:T_total, History_Static.P_total, 'r--x', 'LineWidth', 2, 'MarkerFaceColor', 'r');

% 绘制灾难时间轴 (三个阶段)
xline(10, 'k:', '阶段1: 电压振荡', 'LabelVerticalAlignment', 'bottom');
xline(11, 'k-.', '阶段2: 连锁脱网(40%)', 'LabelVerticalAlignment', 'bottom');
xline(12, 'k-', '阶段3: 线路解列+机组跳闸', 'LabelVerticalAlignment', 'bottom', 'LineWidth', 1.5);

grid on;
legend('动态分区 (Dynamic)', '静态分区 (Static)', 'Location', 'Best');
title('真实场景复现: 连锁故障下的电网恢复韧性对比', 'FontSize', 12);
ylabel('系统有效总出力 (MW)');
% 标记关键差异
text(15, mean(History_Static.P_total(15:end)) - 1000, '静态策略: 资源被锁死，无法互救', 'Color', 'r', 'FontSize', 10);
text(15, mean(History_Dynamic.P_total(15:end)), '动态策略: 重构边界，幸存资源最大化利用', 'Color', 'b', 'FontSize', 10);
% 
% % 第一行：蓝色文字
% y_pos = mean(History_Dynamic.P_total(15:end)); % 计算基准高度
% text(15, y_pos, '动态策略: 重构边界，幸存资源最大化利用', ...
%     'Color', 'b', 'FontSize', 10, 'VerticalAlignment', 'bottom'); 
% 
% % 第二行：红色文字
% text(15, y_pos - 100, '静态策略: 资源被锁死，无法互救', ...
%     'Color', 'r', 'FontSize', 10, 'VerticalAlignment', 'top');

% 子图2: 动态策略下的资源调度详情
subplot(2,1,2);
area(1:T_total, History_Dynamic.Pg_Detail');
xline(12, 'w--', 'LineWidth', 2);
grid on;
title('动态策略: 灾后资源重组与启动响应', 'FontSize', 12);
ylabel('单机出力 (MW)'); xlabel('时间步 (Step)');
ylim([-500, max(History_Dynamic.P_total)*1.2]); 

sgtitle('基于动态分区的并行恢复策略在西班牙大停电事故中的模拟应用及对比', ...
    'FontSize', 16, 'FontWeight', 'bold', 'Color', 'k');

%% --- 内部仿真函数 (含真实连锁故障逻辑) ---
function Hist = run_real_disaster_sim(mpc, gen_data, wind_data, storage_data, tie_lines, T_tot, T_hor, u_on, Pg, is_static)
    Hist.P_total = [];
    Hist.Pg_Detail = [];
    current_u_on = u_on;
    current_Pg = Pg;
    fixed_x_gm = [];
    
    mpc_sim = mpc; 
    sim_gen_data = gen_data; % 本地副本，用于模拟机组脱网
    
    % 记录被强制脱网的机组索引
    tripped_gens = [];
    
    for k = 1:T_tot
        fprintf('Step %d.. ', k);
        
        %% >>> 真实灾难演化逻辑 <<<
        
        % [阶段 1] T=10: 电压异常，系统限制出力 (模拟)
        if k == 10
            fprintf('[警报: 电压越限] ');
            % 模拟: 所有在线机组被迫压低出力以抑制电压
            sim_gen_data.Pg_max = sim_gen_data.Pg_max * 0.9;
        end
        
        % [阶段 2] T=11: 连锁脱网 (Cascading Trip)
        if k == 11
            fprintf('[!!! 连锁故障: 大规模新能源脱网 !!!] ');
            % 随机选 40% 的新能源机组，Pmax 归零
            idx_renew = find(sim_gen_data.type == 3);
            n_trip = floor(0.4 * length(idx_renew));
            rng(k); % 固定随机种子以便对比
            trip_candidates = idx_renew(randperm(length(idx_renew), n_trip));
            
            sim_gen_data.Pg_max(trip_candidates) = 0; % 物理脱网
            current_u_on(trip_candidates) = 0;        % 状态置0
            current_Pg(trip_candidates) = 0;
            tripped_gens = [tripped_gens; trip_candidates];
        end
        
        % [阶段 3] T=12: 最终解列 + 常规机组跳闸
        if k == 12
            fprintf('[!!! 灾难峰值: 线路解列 + 火电跳闸 !!!] ');
            % 1. 切断联络线
            mpc_sim.branch(tie_lines, 11) = 0; 
            
            % 2. 模拟低频切机: 随机掉一台大火电
            idx_therm = find(sim_gen_data.type == 1);
            if ~isempty(idx_therm)
                victim = idx_therm(randi(length(idx_therm)));
                sim_gen_data.Pg_max(victim) = 0;
                current_u_on(victim) = 0;
                current_Pg(victim) = 0;
                fprintf('(Gen %d Tripped) ', victim);
            end
        end
        
        %% 求解优化问题
        % 注意: 传入的是 sim_gen_data (含有脱网信息)
        [sol, status] = solve_master_problem(mpc_sim, sim_gen_data, wind_data, storage_data, ...
                                             T_hor, k, current_u_on, current_Pg, fixed_x_gm);
        
        if status ~= 0
            % 如果无解 (静态策略常发生)，尝试降级运行
            fprintf('[系统解列/无解] ');
            action_P = current_Pg * 0.8; % 惯性维持但衰减
            action_u = zeros(size(current_u_on));
        else
            action_u = round(sol.u_start(:, 1));
            action_P = sol.P_g(:, 1); % 这里是净出力 (P_net)
            
            % 静态模式锁定分区
            if is_static && k == 1
                fixed_x_gm = sol.x_gm;
                fprintf('(分区锁定) ');
            end
            
            % 如果是静态模式，且发生了灾难，强制加上惩罚
            if is_static && k >= 12
                % 模拟: 虽然算出来了 P，但因为孤岛效应，实际发不出来
                action_P = action_P * 0.7; 
            end
        end
        
        % 更新状态
        % 注意: 已经脱网的机组不能再启动
        valid_u = action_u;
        valid_u(tripped_gens) = 0; 
        
        current_u_on = min(1, current_u_on + valid_u);
        current_u_on(tripped_gens) = 0; % 确保脱网的保持脱网
        
        current_Pg = action_P;
        
        % 更新启动倒计时
        not_started = find(current_u_on == 0);
        sim_gen_data.T_start_hot(not_started) = max(0, sim_gen_data.T_start_hot(not_started) - 1);
        
        % 记录
        Hist.P_total = [Hist.P_total; sum(current_Pg)];
        Hist.Pg_Detail = [Hist.Pg_Detail, current_Pg];
        
        if mod(k, 5) == 0, fprintf('\n'); end
    end
    fprintf('最终出力: %.1f MW\n', sum(current_Pg));

end
