clear; clc; close all;

%% 1. 环境初始化
[mpc, gen_data, wind_data, storage_data] = init_system_data('case39');

T_total_sim = 20;   % 总仿真时长
T_horizon_opt = 5;  % 预测时域

% --- 初始化状态 ---
% 1. 开关状态: 仅黑启动电源(BS)开启
current_u_on = zeros(size(mpc.gen, 1), 1);
idx_bs = find(gen_data.type == 2);
current_u_on(idx_bs) = 1;

% 2. 出力状态: 初始给一点点火种
current_Pg = zeros(size(mpc.gen, 1), 1);
current_Pg(idx_bs) = 10; 

% 历史记录
History = struct();
History.P_gen_total = [];
History.Pg_Detail = [];

fprintf('=== 开始 SHMPC 滚动恢复仿真 (总步数: %d) ===\n', T_total_sim);
fprintf('提示: 已启用启动时间倒计时逻辑，机组将分批次加入...\n');

%% 2. 滚动循环
for k = 1:T_total_sim
    fprintf('\n>>> Step %d ... ', k);
    
    % 求解主问题
    % 传入当前的 gen_data (其中 T_start_hot 会随着循环不断减小)
    [sol_mp, status_mp] = solve_master_problem(mpc, gen_data, wind_data, storage_data, T_horizon_opt, k, current_u_on, current_Pg);
    
    if status_mp ~= 0
        warning('无解! 保持上一时刻状态.');
        action_P_g = current_Pg;
        action_u_start = zeros(size(current_u_on));
    else
        action_u_start = round(sol_mp.u_start(:, 1));
        action_P_g = sol_mp.P_g(:, 1);
    end
    
    % 更新状态
    current_u_on = min(1, current_u_on + action_u_start);
    current_Pg = action_P_g;
    
    % --- 更新 gen_data 的倒计时 ---
    % 如果 T_start_hot > 0，就减 1，直到为 0
    not_started_idx = find(current_u_on == 0);
    gen_data.T_start_hot(not_started_idx) = max(0, gen_data.T_start_hot(not_started_idx) - 1);
    
    % 记录
    total_power = sum(action_P_g);
    History.P_gen_total = [History.P_gen_total; total_power];
    History.Pg_Detail = [History.Pg_Detail, action_P_g];
    
    started_gens = find(action_u_start == 1);
    if ~isempty(started_gens)
        fprintf('[★启动机组: %s] ', mat2str(started_gens'));
    else
        fprintf('[爬坡/等待中] ');
    end
    fprintf('总出力: %.2f MW', total_power);
end

%% 3. 结果可视化
fprintf('\n\n仿真结束，正在绘图...\n');
figure('Color', 'w', 'Position', [100, 100, 800, 600]);

subplot(2,1,1);
plot(1:T_total_sim, History.P_gen_total, 'b-o', 'LineWidth', 2, 'MarkerFaceColor', 'w');
grid on;
title('系统总发电能力恢复轨迹', 'FontSize', 12);
ylabel('总功率 (MW)');
xlim([1, T_total_sim]);

subplot(2,1,2);
area(1:T_total_sim, History.Pg_Detail');
grid on;
title('机组并网时序堆叠图', 'FontSize', 12);
xlabel('时间步 (Step)'); 
ylabel('单机出力 (MW)');
xlim([1, T_total_sim]);

sgtitle('基于SHMPC的含规模新能源电网动态分区并行恢复过程演化图（case39）', ...
    'FontSize', 16, 'FontWeight', 'bold', 'Color', 'k');

legend_str = {};
for i=1:size(mpc.gen,1)
    if ismember(i, idx_bs)
        legend_str{end+1} = sprintf('Gen %d (BS)', i);
    else
        legend_str{end+1} = sprintf('Gen %d', i);
    end
end

legend(legend_str, 'Location', 'eastoutside', 'FontSize', 8);
