%% 底盘单轮电机系统辨识工具 v2.0
% - 支持架空调试模式
% - 读取 Ozone CSV 数据，完成电流 -> 轮速 的一阶模型辨识
% - 基于辨识结果给出 LQR 力控增益建议

clear; close all; clc;

fprintf('\n========================================\n');
fprintf('  底盘单轮电机系统辨识工具 v2.0\n');
fprintf('  (支持架空调试模式)\n');
fprintf('========================================\n\n');

%% 一、基础参数设置（按需修改）
Kt            = 0.30;      % 输出轴转矩常数 [N·m/A]，M3508P19 数据表
GearRatio     = 19;        % 减速比（仅用于提示）
r_wheel       = 0.077;     % 轮半径 [m]
M_chassis     = 25;        % 底盘总质量 [kg]
CAN_MAX       = 16384;     % CAN 电流控制量程（RoboMaster 默认）
MaxCurrent_A  = 30;        % 单轮允许最大电流 [A]

% 底盘几何尺寸（标准英雄底盘，若有改动请同步修改）
WHEEL_BASE = 0.56;         % 前后轮距 [m]
TRACK_WIDTH = 0.33;        % 左右轮距 [m]

half_L = WHEEL_BASE / 2;
half_W = TRACK_WIDTH / 2;
r_rotation = hypot(half_L, half_W);
J_chassis = M_chassis * (WHEEL_BASE^2 + TRACK_WIDTH^2) / 12;

fprintf('电机参数：\n');
fprintf('  Kt        = %.2f N·m/A (输出轴)\n', Kt);
fprintf('  减速比    = %.0f:1\n', GearRatio);
fprintf('  轮半径    = %.3f m\n', r_wheel);
fprintf('  底盘质量  = %.1f kg\n\n', M_chassis);

%% 二、读取 CSV 数据
fprintf('========== 步骤1：读取 CSV 数据 ==========\n');
[filename, pathname] = uigetfile('*.csv', '选择 Ozone 导出的 CSV 文件');
if isequal(filename, 0)
    error('取消文件选择');
end

data_table = readtable(fullfile(pathname, filename));
fprintf('读取文件: %s\n', filename);

try
    var_names = data_table.Properties.VariableNames;

    if ismember('sysid_data_time_elapsed', var_names)
        time = data_table.sysid_data_time_elapsed;
        input_CAN = data_table.sysid_data_step_input;
        output_omega = data_table.sysid_data_motor_output;
    else
        time_idx = find(contains(var_names, 'time', 'IgnoreCase', true), 1);
        input_idx = find(contains(var_names, 'step_input', 'IgnoreCase', true) | ...
                          contains(var_names, 'current', 'IgnoreCase', true), 1);
        output_idx = find(contains(var_names, 'motor_output', 'IgnoreCase', true) | ...
                           contains(var_names, 'omega', 'IgnoreCase', true) | ...
                           contains(var_names, 'speed', 'IgnoreCase', true), 1);

        if isempty(time_idx) || isempty(input_idx) || isempty(output_idx)
            error('无法自动识别必要列，请检查 CSV 文件列名。');
        end

        time = data_table{:, time_idx};
        input_CAN = data_table{:, input_idx};
        output_omega = data_table{:, output_idx};
    end
catch ME
    error('数据提取失败：%s', ME.message);
end

fprintf('✓ 数据点数: %d\n', numel(time));
fprintf('  时间范围: %.2f ~ %.2f s\n\n', min(time), max(time));

%% 三、单位转换
fprintf('========== 步骤2：单位转换 ==========\n');
input_current_A = input_CAN / CAN_MAX * MaxCurrent_A;

fprintf('  电流范围: %.2f ~ %.2f A\n', min(input_current_A), max(input_current_A));
fprintf('  轮速范围: %.2f ~ %.2f rad/s\n\n', min(output_omega), max(output_omega));

%% 四、电机方向设置
fprintf('========== 电机安装方向设置 ==========\n');
fprintf('  输入  1 : 电机正向安装（正电流得到正轮速）\n');
fprintf('  输入 -1 : 电机反向安装（正电流得到负轮速）\n');
motor_dir = input('输入方向 (默认 1): ');
if isempty(motor_dir)
    motor_dir = 1;
end
if ~(motor_dir == 1 || motor_dir == -1)
    error('电机方向输入无效，请输入 1 或 -1。');
end
if motor_dir == -1
    fprintf('已应用反向补偿，使正电流对应正轮速。\n\n');
else
    fprintf('保持正向安装设置。\n\n');
end
output_omega = output_omega * motor_dir;

%% 五、选择有效数据区间
fprintf('========== 步骤3：选择有效数据区间 ==========\n');

figure('Position', [100, 100, 1400, 600], 'Name', '数据预览');
subplot(1, 2, 1);
plot(time, input_current_A, 'b', 'LineWidth', 1.2);
title('输入：电机电流 [A]'); xlabel('时间 (s)'); ylabel('电流 (A)'); grid on;

subplot(1, 2, 2);
plot(time, output_omega, 'r', 'LineWidth', 1.2);
title('输出：轮速 [rad/s]'); xlabel('时间 (s)'); ylabel('角速度 (rad/s)'); grid on;

fprintf('请在图中使用鼠标左键点击两次，选取有效数据区间。\n');
fprintf('建议跳过起始与结束的过渡段，只保留稳定阶跃。\n');
[x_pick, ~] = ginput(2);
if numel(x_pick) ~= 2
    error('未正确选取数据区间');
end
x_pick = sort(x_pick);

start_idx = find(time >= x_pick(1), 1);
end_idx = find(time <= x_pick(2), 1, 'last');
if isempty(start_idx) || isempty(end_idx)
    error('选取的区间越界，请重新选择。');
end

valid_time = time(start_idx:end_idx) - time(start_idx);
valid_current = input_current_A(start_idx:end_idx);
valid_omega = output_omega(start_idx:end_idx);

if numel(valid_time) < 10
    error('有效数据点过少，请重新选择区间。');
end

% 处理非等距采样
if numel(valid_time) > 2
    dt_array = diff(valid_time);
    dt_mean = mean(dt_array);
    if dt_mean == 0
        error('采样时间间隔为零，请检查数据。');
    end
    if std(dt_array) / dt_mean > 0.01
        fprintf('检测到非等距采样，正在重采样为均匀时间序列...\n');
        t_uniform = (0:dt_mean:valid_time(end))';
        valid_current = interp1(valid_time, valid_current, t_uniform, 'linear', 'extrap');
        valid_omega = interp1(valid_time, valid_omega, t_uniform, 'linear', 'extrap');
        valid_time = t_uniform;
    end
end

fprintf('✓ 选择数据: %d 点, 持续 %.2f s\n\n', numel(valid_time), valid_time(end));

%% 六、系统辨识
fprintf('========== 步骤4：系统辨识（一阶模型）==========\n');

Ts = mean(diff(valid_time));
if isnan(Ts) || Ts <= 0
    error('采样周期计算失败，请检查数据区间。');
end
fprintf('采样周期: %.4f s (%.1f Hz)\n', Ts, 1 / Ts);

data_id = iddata(valid_omega, valid_current, Ts);
sys_I_to_omega = tfest(data_id, 1, 0);

fprintf('\n传递函数 ω(s) / I(s):\n');
disp(tf(sys_I_to_omega));

[num_tf, den_tf] = tfdata(sys_I_to_omega, 'v');
K_sys = num_tf(end);
a_sys = den_tf(end);

% ⭐ 惯量计算：Kt 已为输出轴转矩常数 (0.3 N·m/A)
J_eff = Kt / (K_sys * r_wheel);
b_eff = a_sys * J_eff;
tau_sys = 1 / a_sys;

fprintf('\n辨识结果（单轮，架空状态）：\n');
fprintf('  等效转动惯量 J_eff = %.6f kg·m²\n', J_eff);
fprintf('  等效阻尼系数 b_eff = %.6f N·m·s/rad（仅电机内部）\n', b_eff);
fprintf('  稳态增益 K = %.4f (rad/s)/A\n', K_sys);
fprintf('  时间常数 τ = %.3f s\n', tau_sys);

% 整车阻尼估算（架空）
b_linear_suspended = 4.0 * b_eff / (r_wheel^2);
b_angular_suspended = 4.0 * b_eff * (r_rotation^2) / r_wheel;

fprintf('\n整车等效参数（架空状态，不含地面摩擦）：\n');
fprintf('  平移阻尼 b_linear ≈ %.2f N·s/m\n', b_linear_suspended);
fprintf('  旋转阻尼 b_angular ≈ %.2f N·m·s/rad\n', b_angular_suspended);

y_predict = lsim(sys_I_to_omega, valid_current, valid_time);
residual = valid_omega - y_predict;
fit_percent = 100 * (1 - norm(residual) / norm(valid_omega - mean(valid_omega)));

fprintf('\n模型拟合度: %.1f%%%%\n', fit_percent);
if fit_percent < 70
    warning('拟合度低于 70%%，建议重新挑选数据区间。');
elseif fit_percent < 85
    fprintf('拟合度中等，可继续使用但建议二次确认。\n');
else
    fprintf('拟合度良好！\n');
end

figure('Position', [100, 100, 1200, 500], 'Name', '系统辨识拟合结果');
subplot(1, 2, 1);
plot(valid_time, valid_omega, 'b', 'LineWidth', 1.2, 'DisplayName', '实际'); hold on;
plot(valid_time, y_predict, 'r--', 'LineWidth', 1.2, 'DisplayName', '模型');
legend('Location', 'best'); xlabel('时间 (s)'); ylabel('角速度 (rad/s)');
title(sprintf('系统拟合 (%.1f%%%%)', fit_percent)); grid on;

subplot(1, 2, 2);
plot(valid_time, residual, 'k', 'LineWidth', 1.2);
ylabel('误差 (rad/s)'); xlabel('时间 (s)'); title('残差'); grid on;

%% 七、架空模式阻尼修正
fprintf('\n========== 步骤4.5：阻尼系数修正（架空调试）==========\n');
fprintf('⚠️ 注意：架空状态辨识的阻尼只包含电机内部摩擦！\n');
fprintf('        落地后地面摩擦会增加 5~10 倍的阻尼。\n\n');

is_suspended = input('车轮当前是否架空？(1=是，0=否，默认1): ');
if isempty(is_suspended)
    is_suspended = 1;
end

if is_suspended == 1
    fprintf('\n--- 架空模式：请输入落地后的预期阻尼 ---\n');
    fprintf('参考值（麦轮 + 标准瓷砖地板）：\n');
    fprintf('  平移阻尼 b_linear = 20 ~ 35 N·s/m\n');
    fprintf('  旋转阻尼 b_angular = 10 ~ 18 N·m·s/rad\n');
    fprintf('推荐起始值（保守）：\n');
    fprintf('  平移阻尼 = 25 N·s/m\n');
    fprintf('  旋转阻尼 = 12 N·m·s/rad\n\n');

    b_linear_input = input('输入平移阻尼 b_linear [N·s/m] (默认25): ');
    if isempty(b_linear_input)
        b_linear_input = 25;
    end

    b_angular_input = input('输入旋转阻尼 b_angular [N·m·s/rad] (默认12): ');
    if isempty(b_angular_input)
        b_angular_input = 12;
    end

    b_linear_est = b_linear_input;
    b_angular_est = b_angular_input;

    fprintf('\n✓ 使用手动输入的阻尼系数进行 LQR 设计。\n');
    fprintf('  架空辨识值: b_linear=%.2f, b_angular=%.2f N·m·s/rad（仅供参考）\n', ...
            b_linear_suspended, b_angular_suspended);
    fprintf('  实际设计值: b_linear=%.2f, b_angular=%.2f N·m·s/rad（LQR 使用）\n\n', ...
            b_linear_est, b_angular_est);
else
    fprintf('\n--- 落地模式：直接使用辨识得到的阻尼系数 ---\n');
    b_linear_est = b_linear_suspended;
    b_angular_est = b_angular_suspended;
    fprintf('✓ 使用辨识阻尼: b_linear=%.2f, b_angular=%.2f\n\n', ...
            b_linear_est, b_angular_est);
end

%% 八、LQR 设计
fprintf('========== 步骤5：LQR 设计 ==========\n');

A_linear = -b_linear_est / M_chassis;
B_linear =  1 / M_chassis;
A_angular = -b_angular_est / J_chassis;
B_angular =  1 / J_chassis;

fprintf('\n状态方程：\n');
fprintf('  平移: dv/dt = %.4f * v + %.4f * F\n', A_linear, B_linear);
fprintf('  旋转: dω/dt = %.4f * ω + %.4f * τ\n', A_angular, B_angular);

fprintf('\n请选择 LQR 设计模式：\n');
fprintf('  1 - 自动搜索最优参数 (推荐)\n');
fprintf('  2 - 手动输入 Q / R\n');
mode = input('输入选择 (1 或 2): ');

if mode == 2
    fprintf('\n--- 手动参数模式 ---\n');
    Q_linear = input('输入平移方向 Q_linear (默认 5000): ');
    if isempty(Q_linear)
        Q_linear  = 5000;
    end
    R_linear = input('输入平移方向 R_linear (默认 1): ');
    if isempty(R_linear)
        R_linear  = 1;
    end

    Q_angular = input('输入旋转方向 Q_angular (默认 2000): ');
    if isempty(Q_angular)
        Q_angular = 2000;
    end
    R_angular = input('输入旋转方向 R_angular (默认 1): ');
    if isempty(R_angular)
        R_angular = 1;
    end

    K_best_linear   = lqr(A_linear,  B_linear,  Q_linear,  R_linear);
    K_best_angular  = lqr(A_angular, B_angular, Q_angular, R_angular);
    Q_best_linear   = Q_linear;
    R_best_linear   = R_linear;
    Q_best_angular  = Q_angular;
    R_best_angular  = R_angular;
else
    fprintf('\n--- 自动参数搜索 ---\n');
    best_score_linear = inf;
    candidates_Q_linear = [1000, 2000, 5000, 10000, 20000];
    candidates_R_linear = [0.1, 0.5, 1.0, 2.0, 5.0];

    for Q_linear = candidates_Q_linear
        for R_linear = candidates_R_linear
            K_tmp = lqr(A_linear, B_linear, Q_linear, R_linear);
            A_cl_tmp = A_linear - B_linear * K_tmp;
            if A_cl_tmp < 0
                tau_tmp = 1 / (-A_cl_tmp);
                score = tau_tmp * 1000 + 0.1 * K_tmp;
                if score < best_score_linear
                    best_score_linear = score;
                    K_best_linear = K_tmp;
                    Q_best_linear = Q_linear;
                    R_best_linear = R_linear;
                end
            end
        end
    end

    best_score_angular = inf;
    candidates_Q_angular = [500, 1000, 2000, 5000, 10000];
    candidates_R_angular = [0.1, 0.5, 1.0, 2.0, 5.0];

    for Q_angular = candidates_Q_angular
        for R_angular = candidates_R_angular
            K_tmp = lqr(A_angular, B_angular, Q_angular, R_angular);
            A_cl_tmp = A_angular - B_angular * K_tmp;
            if A_cl_tmp < 0
                tau_tmp = 1 / (-A_cl_tmp);
                score = tau_tmp * 1000 + 0.1 * K_tmp;
                if score < best_score_angular
                    best_score_angular = score;
                    K_best_angular = K_tmp;
                    Q_best_angular = Q_angular;
                    R_best_angular = R_angular;
                end
            end
        end
    end
end

fprintf('\nLQR 设计结果：\n');
fprintf('  平移: Q = %.0f, R = %.2f -> K_velocity = %.2f N/(m/s)\n', ...
        Q_best_linear, R_best_linear, K_best_linear);
fprintf('  旋转: Q = %.0f, R = %.2f -> K_velocity = %.2f N·m/(rad/s)\n', ...
        Q_best_angular, R_best_angular, K_best_angular);

%% 九、性能仿真
fprintf('\n========== 步骤6：性能仿真 ==========\n');

figure('Position', [100, 100, 1200, 420], 'Name', 'LQR 闭环性能');

subplot(1, 2, 1);
A_cl_linear = A_linear - B_linear * K_best_linear;
sys_cl_linear = ss(A_cl_linear, B_linear, 1, 0);
[y_step, t_step] = step(sys_cl_linear, 0:0.001:1);
plot(t_step, y_step, 'b', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('速度响应 (归一化)');
title(sprintf('平移闭环阶跃响应\nK = %.2f N/(m/s)', K_best_linear)); grid on;
info_linear = stepinfo(y_step, t_step);
fprintf('  平移性能: 上升时间=%.3fs, 超调=%.1f%%%%\n', info_linear.RiseTime, info_linear.Overshoot);

subplot(1, 2, 2);
A_cl_angular = A_angular - B_angular * K_best_angular;
sys_cl_angular = ss(A_cl_angular, B_angular, 1, 0);
[y_step, t_step] = step(sys_cl_angular, 0:0.001:1);
plot(t_step, y_step, 'r', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('角速度响应 (归一化)');
title(sprintf('旋转闭环阶跃响应\nK = %.2f N·m/(rad/s)', K_best_angular)); grid on;
info_angular = stepinfo(y_step, t_step);
fprintf('  旋转性能: 上升时间=%.3fs, 超调=%.1f%%%%\n\n', info_angular.RiseTime, info_angular.Overshoot);

%% 十、嵌入式代码参数输出
fprintf('========================================\n');
fprintf(' 嵌入式代码参数（复制到 chassis.c）\n');
fprintf('========================================\n\n');

fprintf('/* ========================================\n');
fprintf(' * 底盘 LQR 控制器参数（基于系统辨识）\n');
fprintf(' * ========================================\n');
fprintf(' * 辨识参数（单轮）：\n');
fprintf(' *   J_eff = %.6f kg·m²\n', J_eff);
fprintf(' *   b_eff = %.6f N·m·s/rad (架空)\n', b_eff);
fprintf(' *   拟合度 = %.1f%%%%\n', fit_percent);
fprintf(' * 设计参数（整车）：\n');
fprintf(' *   b_linear = %.2f N·s/m\n', b_linear_est);
fprintf(' *   b_angular = %.2f N·m·s/rad\n', b_angular_est);
if is_suspended
    fprintf(' *   ⚠️ 使用手动输入阻尼（架空模式）\n');
else
    fprintf(' *   使用辨识阻尼（落地模式）\n');
end
fprintf(' * ========================================*/\n\n');

fprintf('LQR_Velocity_Init_Config_s force_x_lqr_config = {\n');
fprintf('    .K_velocity = %.2ff,\n', K_best_linear);
fprintf('    .K_integral = %.2ff,\n', K_best_linear * 0.05);
fprintf('    .max_out = MAX_CONTROL_FORCE,\n');
fprintf('    .enable_integral = 0,\n');
fprintf('    .integral_limit = 50.0f,\n');
fprintf('    .integral_deadband = 0.02f,\n');
fprintf('    .integral_decay_coef = 0.0f,\n');
fprintf('};\n\n');

fprintf('LQR_Velocity_Init_Config_s force_y_lqr_config = {\n');
fprintf('    .K_velocity = %.2ff,\n', K_best_linear);
fprintf('    .K_integral = %.2ff,\n', K_best_linear * 0.05);
fprintf('    .max_out = MAX_CONTROL_FORCE,\n');
fprintf('    .enable_integral = 0,\n');
fprintf('    .integral_limit = 50.0f,\n');
fprintf('    .integral_deadband = 0.02f,\n');
fprintf('    .integral_decay_coef = 0.0f,\n');
fprintf('};\n\n');

fprintf('LQR_Velocity_Init_Config_s torque_lqr_config = {\n');
fprintf('    .K_velocity = %.2ff,\n', K_best_angular);
fprintf('    .K_integral = %.2ff,\n', K_best_angular * 0.05);
fprintf('    .max_out = MAX_CONTROL_TORQUE,\n');
fprintf('    .enable_integral = 0,\n');
fprintf('    .integral_limit = 10.0f,\n');
fprintf('    .integral_deadband = 0.05f,\n');
fprintf('    .integral_decay_coef = 0.0f,\n');
fprintf('};\n\n');

fprintf('========================================\n');
fprintf('💡 使用建议：\n');
if is_suspended
    fprintf('  1. 架空辨识得到了 J_eff（惯量），该值可直接用于仿真。\n');
    fprintf('  2. 阻尼使用了手动输入值 (%.0f, %.0f)，实车需微调。\n', b_linear_est, b_angular_est);
    fprintf('  3. 落地后若响应过快/振荡，增大阻尼值；反之减小。\n');
else
    fprintf('  1. 落地辨识的参数可直接使用。\n');
    fprintf('  2. 如有振荡可适当增大积分死区或减小 K。\n');
end
fprintf('========================================\n\n');

%% 十一、保存结果
save_file = fullfile(pathname, 'Chassis_LQR_Results.mat');
save(save_file, 'K_best_linear', 'K_best_angular', 'J_eff', 'b_eff', ...
    'b_linear_est', 'b_angular_est', 'fit_percent', 'is_suspended');
fprintf('结果已保存到: %s\n\n', save_file);

fprintf('========================================\n');
fprintf('复制以上配置到 chassis.c 的 ChassisInit() 函数\n');
fprintf('========================================\n');

