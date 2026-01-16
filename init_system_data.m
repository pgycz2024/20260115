function [mpc, gen_data, wind_data, storage_data] = init_system_data(case_name)
% INIT_SYSTEM_DATA 初始化系统数据 (适配 PPSR 求解器)
% 自动修改拓扑和参数，并补充 T_crank/P_crank 字段

    fprintf('正在初始化 %s 系统数据...\n', case_name);
    
    %% 1. 加载并预处理标准案例
    mpc = loadcase(case_name);
    
    % IEEE 118 特殊处理：点 10 和 110 是黑启动风电场
    if strcmp(case_name, 'case118')
        if ~ismember(10, mpc.gen(:,1))
            new_row = mpc.gen(1, :); new_row(1) = 10; new_row(9) = 100;
            mpc.gen = [mpc.gen; new_row];
        end
        if ~ismember(110, mpc.gen(:,1))
            new_row = mpc.gen(1, :); new_row(1) = 110; new_row(9) = 100;
            mpc.gen = [mpc.gen; new_row];
        end
        [~, sort_idx] = sort(mpc.gen(:, 1));
        mpc.gen = mpc.gen(sort_idx, :);
        
        bs_bus_ids = [61, 10, 110];
        
        all_gen_buses = mpc.gen(:, 1);
        candidate_wind = setdiff(all_gen_buses, bs_bus_ids);
        rng(118); 
        wind_bus_ids = candidate_wind(randperm(length(candidate_wind), 10)); 
        
    elseif strcmp(case_name, 'case39')
        bs_bus_ids = [30, 36];
        wind_bus_ids = [31, 38];
    end
    
    %% 2. 构建类型索引
    num_gen = size(mpc.gen, 1);
    gen_type = ones(num_gen, 1); % 默认为 1 (火电/Thermal)
    
    idx_BS = find(ismember(mpc.gen(:, 1), bs_bus_ids));
    idx_Wind = find(ismember(mpc.gen(:, 1), wind_bus_ids));
    
    gen_type(idx_BS) = 2;   % 2 = BS
    gen_type(idx_Wind) = 3; % 3 = Wind
    
    %% 3. 设置物理参数 (关键：参数差异化)
    gen_data = struct();
    gen_data.type = gen_type;
    gen_data.Pg_max = mpc.gen(:, 9);
    gen_data.Pg_min = zeros(num_gen, 1);
    
    % --- 3.1 削弱 BS 上限 ---
    if strcmp(case_name, 'case118')
        gen_data.Pg_max(idx_BS) = 150; 
    else
        gen_data.Pg_max(idx_BS) = 200;
    end
    
    % --- 3.2 爬坡率设置 ---
    gen_data.Ramp = 0.04 * gen_data.Pg_max; 
    gen_data.Ramp(idx_BS) = 0.08 * gen_data.Pg_max(idx_BS); 
    
    % --- 3.3 启动时间设置 ---
    gen_data.T_start_hot = zeros(num_gen, 1);
    gen_data.T_start_hot(idx_BS) = 0;
    gen_data.T_start_hot(idx_Wind) = 1;
    
    idx_Thermal = find(gen_type == 1);
    for i = 1:length(idx_Thermal)
        gen_data.T_start_hot(idx_Thermal(i)) = mod(i, 5) + 2; 
    end
    
    gen_data.P_crank = zeros(num_gen, 1);
    gen_data.T_crank = zeros(num_gen, 1);

    % 火电: 设置少量启动功率和耗能时间
    gen_data.P_crank(idx_Thermal) = 0.05 * gen_data.Pg_max(idx_Thermal);
    gen_data.T_crank(idx_Thermal) = 1; % 假设火电启动需消耗1个时段电能

    % BS: 不需要倒送电
    gen_data.P_crank(idx_BS) = 0;
    gen_data.T_crank(idx_BS) = 0;

    % 风电: 极少量自用电
    gen_data.P_crank(idx_Wind) = 0.01 * gen_data.Pg_max(idx_Wind);
    gen_data.T_crank(idx_Wind) = 0; % 风电启动较快，忽略耗能时长
    
    %% 4. 风电与储能数据
    T_total = 100; 
    n_wind = length(idx_Wind);
    
    t_vec = linspace(0, 4*pi, T_total)';
    base_curve = 0.4 + 0.2 * sin(t_vec); 
    
    wind_data = struct();
    wind_data.idx = idx_Wind;
    wind_data.forecast = (base_curve * ones(1, n_wind))' .* mpc.gen(idx_Wind, 9);
    wind_data.forecast = min(wind_data.forecast, 200); 
    wind_data.error_bound = 0.1 * wind_data.forecast;
    
    storage_data = struct();
    storage_data.idx = idx_Wind; 
    storage_data.P_max = 30 * ones(n_wind, 1);
    storage_data.E_max = 100 * ones(n_wind, 1);
    
    fprintf('初始化完成: %d 台机组 (BS: %d, Wind: %d)\n', ...
        num_gen, length(idx_BS), length(idx_Wind));

end