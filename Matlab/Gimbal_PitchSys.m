% %= == == == == == == == == == == == == == == == == == == == == == == == == ==
     == == == == == == == == == == == ==
     % % 云台 Pitch 轴 LQR 控制器设计工具 - Pitch 轴专用版 %
    %= == == == == == == == == == == == == == == == == == == == == == == == ==
       == == == == == == == == == == == == ==
       % % 功能： % % 1. 从Ozone CSV数据进行系统辨识（Pitch轴专用） % %
           2. 自动或手动设计LQR增益 % % 3. 生成嵌入式代码 % % % %
           Pitch轴特性： % % -电机型号：M3508(Kt = 0.3 N·m / A, 减速比19 : 1) %
           % -受重力矩影响：M_g =
           m * g * L * sin(θ) % % -重力补偿前馈：600 CAN值（约0.73A） % %
               -线性化有效范围：θ ∈ [-15°, +15°] % % % % 使用方法： % %
               1. 在水平位置附近（θ≈0）进行系统辨识 % %
               2. 在Ozone中导出辨识数据（CSV格式） % % 3. 运行本脚本 % %
               4. 选择有效数据区间 % % 5. 选择自动搜索或手动调参模式 % %
               6. 复制生成的参数到gimbal.c % % % % 注意事项： % %
               -辨识时保持Pitch角度在 ±10° 范围内 % %
               -如果角度偏离过大，线性化模型将不准确 % %
               -重力补偿参数需根据实际负载调整 % % % %
               作者：RoboMaster EC Team % % 日期：2025 -
           11 - 15 % % 版本：Pitch轴专用 v1.0 %
    %= == == == == == == == == == == == == == == == == == == == == == == == ==
       == == == == == == == == == == == == ==

       clear;
close all;
clc;

% % == == == == == 电机参数定义（Pitch轴专用） == == == == == Kt = 0.3;
% 力矩常数[N·m / A](M3508) GearRatio = 19;
% 减速比（M3508标准配置，19 : 1） CAN_MAX = 16384;
% CAN控制值最大值 MaxCurrent_A = 20;
% 最大额定电流[A](C620电调)

        % Pitch轴特有参数
    - 重力补偿分析 GRAVITY_COMP_CAN = 600.0;
% 当前代码中的重力补偿值（CAN指令） GRAVITY_COMP_A =
    GRAVITY_COMP_CAN * MaxCurrent_A / CAN_MAX;
% 转为电流[A] GRAVITY_TORQUE_ESTIMATE = GRAVITY_COMP_A * Kt;
% 估算重力矩[N·m]

    fprintf('\n========================================\n');
fprintf('  Pitch轴LQR控制器设计工具 v1.0\n');
fprintf('========================================\n\n');

fprintf('Pitch轴配置参数:\n');
fprintf('  电机型号: M3508\n');
fprintf('  转矩常数: %.3f N·m/A\n', Kt);
fprintf('  减速比: %.0f:1\n', GearRatio);
fprintf('  CAN范围: ±%d\n', CAN_MAX);
fprintf('  电流范围: ±%.0f A\n', MaxCurrent_A);
fprintf('\n');
fprintf('重力补偿分析:\n');
fprintf('  补偿CAN值: %.0f\n', GRAVITY_COMP_CAN);
fprintf('  等效电流: %.2f A\n', GRAVITY_COMP_A);
fprintf('  估算重力矩: %.3f N·m\n', GRAVITY_TORQUE_ESTIMATE);
fprintf('  （假设在水平位置，重力矩≈0时辨识）\n\n');

% % == == == == == 第1步：读取数据 == == == == ==
    fprintf('========== 步骤1：读取CSV数据 ==========\n');

[ filename, pathname ] =
    uigetfile('*.csv', '选择Ozone导出的CSV文件（Pitch轴辨识数据）');
if isequal (filename, 0)
  error('取消文件选择');
end

    data_table = readtable(fullfile(pathname, filename));
fprintf('读取文件: %s\n', filename);

% 智能提取列 try if ismember ('sysid_datastep_input',
                              data_table.Properties.VariableNames) time =
    data_table.Time;
input_CAN = data_table.sysid_datastep_input;
output_omega_raw = data_table.sysid_datamotor_output;

% 尝试提取Pitch角度（用于验证线性化条件） if ismember ('gimba_IMU_dataPitch',
                                                       data_table.Properties
                                                           .VariableNames)
        pitch_angle = data_table.gimba_IMU_dataPitch;
% 弧度制 has_angle_data = true;
else has_angle_data = false;
end elseif ismember('Time',
                    data_table.Properties.VariableNames) time = data_table.Time;
input_CAN = data_table{ :, 3};
output_omega_raw = data_table{ :, 4};
has_angle_data = false;
else error('无法识别列名，请检查CSV格式');
end catch ME error('数据提取失败: %s', ME.message);
end

    fprintf('✓ 数据点数: %d\n', length(time));

% 检查Pitch角度范围（如果有数据） if has_angle_data
        fprintf('✓ 检测到Pitch角度数据\n');
fprintf('  角度范围: %.2f° ~ %.2f°\n', ... rad2deg(min(pitch_angle)),
        rad2deg(max(pitch_angle)));

if max (abs(pitch_angle))
  > deg2rad(15) warning('⚠️ Pitch角度范围较大(>15°)，线性化可能不准确！');
warning('   建议在水平位置±10°内进行辨识');
fprintf('   当前最大角度: %.2f°\n', rad2deg(max(abs(pitch_angle))));
else fprintf('✓ 角度范围合适，线性化模型有效\n');
end else fprintf('⚠️ 未检测到Pitch角度数据\n');
fprintf('   假设辨识在水平位置(θ≈0)进行\n');
end fprintf('\n');

% % == == == == == 第2步：单位转换（Pitch轴专用） == == == == ==
    fprintf('========== 步骤2：单位转换（Pitch轴） ==========\n');

% CAN值 → 实际电流(A) input_current_A = input_CAN * MaxCurrent_A / CAN_MAX;

% 角速度方向修正 %
    注意：Pitch轴电机方向是NORMAL（不是REVERSE），根据实际情况确定是否取反 %
    默认先不取反，如果辨识结果K为负，则需要取反 output_omega = output_omega_raw;
% Pitch轴电机NORMAL方向，先不取反

        fprintf('电流转换: CAN值 → 实际电流(A)\n');
fprintf('  转换系数: %.6f A/CAN\n', MaxCurrent_A / CAN_MAX);
fprintf('  范围: %.2f ~ %.2f A\n', min(input_current_A), max(input_current_A));
fprintf('\n角速度方向: 保持原始（Pitch电机NORMAL配置）\n');
fprintf('  如果辨识结果K为负，需要在代码中取反\n\n');

% % == == == == == 第3步：数据选择 == == == == ==
    fprintf('========== 步骤3：选择有效数据 ==========\n');

figure('Position', [ 100, 100, 1400, 600 ], 'Name', 'Pitch轴数据预览');
subplot(1, 2, 1);
plot(time, input_current_A, 'b', 'LineWidth', 1.5);
title('输入: 电流 [A] (Pitch轴)');
xlabel('时间 (s)');
ylabel('电流 (A)');
grid on;

subplot(1, 2, 2);
plot(time, output_omega, 'r', 'LineWidth', 1.5);
title('输出: 角速度 [rad/s] (Pitch轴)');
xlabel('时间 (s)');
ylabel('角速度 (rad/s)');
grid on;

fprintf('请点击图中两次选择数据区间（左键点击开始，再点击结束）...\n');
[ x, ~] = ginput(2);
x = sort(x);

start_idx = find(time >= x(1), 1);
end_idx = find(time <= x(2), 1, 'last');

valid_time = time(start_idx : end_idx) - time(start_idx);
valid_current = input_current_A(start_idx : end_idx);
valid_omega = output_omega(start_idx : end_idx);

% 重采样处理非等距数据 dt_array = diff(valid_time);
if std (dt_array)
  / mean(dt_array) > 0.01 fprintf('检测到非等距采样，进行重采样...\n');
t_uni = (0 : mean(dt_array) : valid_time(end))'; valid_current = interp1(
    valid_time, valid_current, t_uni);
valid_omega = interp1(valid_time, valid_omega, t_uni);
valid_time = t_uni;
end

    fprintf('✓ 选择: %d 点, %.2f s\n\n', length(valid_time), valid_time(end));

% % == == == == == 第4步：系统辨识（Pitch轴专用） == == == == ==
    fprintf('========== 步骤4：系统辨识（Pitch轴） ==========\n');

Ts = mean(diff(valid_time));
fprintf('采样周期: %.4f s (%.1f Hz)\n', Ts, 1 / Ts);

data_id = iddata(valid_omega, valid_current, Ts);

% 尝试一阶和二阶系统辨识，选择拟合度更好的
        fprintf('\n正在尝试一阶系统辨识...\n');
sys_I_to_omega_1st = tfest(data_id, 1, 0);
y_sim_1st = lsim(sys_I_to_omega_1st, valid_current, valid_time);
fit_1st = 100 * (1 - norm(valid_omega - y_sim_1st) /
                         norm(valid_omega - mean(valid_omega)));

fprintf('正在尝试二阶系统辨识...\n');
sys_I_to_omega_2nd = tfest(data_id, 2, 1);
y_sim_2nd = lsim(sys_I_to_omega_2nd, valid_current, valid_time);
fit_2nd = 100 * (1 - norm(valid_omega - y_sim_2nd) /
                         norm(valid_omega - mean(valid_omega)));

fprintf('\n拟合度对比:\n');
fprintf('  一阶系统: %.1f%%\n', fit_1st);
fprintf('  二阶系统: %.1f%%\n', fit_2nd);

% 选择拟合度更好的模型（二阶至少好5 % 才采用，避免过拟合） if fit_2nd >
    fit_1st + 5 sys_I_to_omega = sys_I_to_omega_2nd;
y_sim = y_sim_2nd;
fit = fit_2nd;
fprintf('✓ 选择: 二阶系统模型（拟合度显著更好）\n\n');
use_2nd_order = true;
else sys_I_to_omega = sys_I_to_omega_1st;
y_sim = y_sim_1st;
fit = fit_1st;
fprintf('✓ 选择: 一阶系统模型（足够准确）\n\n');
use_2nd_order = false;
end

    fprintf('传递函数 ω(s)/I(s):\n');
disp(tf(sys_I_to_omega));

% 提取参数[num, den] = tfdata(sys_I_to_omega, 'v');

if
  ~use_2nd_order % 一阶系统参数提取 K_omega = num(end);
a = den(2);

% 物理参数（考虑减速比） % 公式：ω(s) /
    I(s) = (Kt * GearRatio / J) /
           (s + b / J) % 其中 K_omega = Kt *GearRatio / J J = Kt * GearRatio
                                                              / K_omega;
% 折算到输出轴的惯量 b = a * J;

fprintf('辨识物理参数（一阶模型）:\n');
fprintf('  输出轴转动惯量 J = %.6f kg·m²\n', J);
fprintf('  阻尼系数 b = %.6f N·m·s/rad\n', b);
fprintf('  等效增益 K_ω = %.3f (rad/s)/A\n', K_omega);
fprintf('  拟合度 = %.1f%%\n\n', fit);

% 验证减速比计算是否合理 J_motor_estimate = J / (GearRatio ^ 2);
% 电机转子惯量估算 fprintf('参数验证:\n');
fprintf('  电机转子惯量估算: %.8f kg·m²\n', J_motor_estimate);
fprintf('  （M3508转子惯量典型值: 5e-5 ~ 1e-4 kg·m²）\n\n');
else % 二阶系统参数 fprintf('辨识物理参数（二阶模型）:\n');
fprintf('  传递函数分子: num = [%s]\n', num2str(num));
fprintf('  传递函数分母: den = [%s]\n', num2str(den));
fprintf('  拟合度 = %.1f%%\n\n', fit);

% 尝试提取等效J和b（近似） K_omega = num(end) / den(end);
a = den(end - 1) / den(end);
J = Kt * GearRatio / K_omega;
b = a * J;

fprintf('近似等效参数（用于LQR设计）:\n');
fprintf('  J_approx = %.6f kg·m²\n', J);
fprintf('  b_approx = %.6f N·m·s/rad\n\n', b);
end

    % 绘制拟合结果 figure('Position', [ 100, 100, 1200, 400 ], 'Name',
                          'Pitch轴系统辨识结果');
plot(valid_time, valid_omega, 'b', 'LineWidth', 1.5, 'DisplayName', '实际输出');
hold on;
plot(valid_time, y_sim, 'r--', 'LineWidth', 1.5, 'DisplayName', '模型预测');
xlabel('时间 (s)');
ylabel('角速度 (rad/s)');
title(sprintf('Pitch轴系统辨识拟合 (拟合度: %.1f%%, %s)', ... fit,
              iif(use_2nd_order, '二阶模型', '一阶模型')));
legend('Location', 'best');
grid on;
hold off;

% % == == == == == 第5步：LQR设计（Pitch轴） == == == == ==
    fprintf('========== 步骤5：LQR设计（Pitch轴） ==========\n');

% 状态空间模型（线性化后，假设重力已补偿） % x =
    [θ; ω] % dx / dt = A *x + B *u A = [ 0, 1; 0, -b / J ];
B = [0; Kt * GearRatio / J];
% 注意：Pitch轴有减速比！ C = [ 1, 0 ];
D = 0;

fprintf('状态空间模型（Pitch轴）:\n');
fprintf('  A = [0, 1; 0, %.4f]\n', -b / J);
fprintf('  B = [0; %.4f]  (已考虑减速比 %.0f:1)\n', Kt *GearRatio / J,
        GearRatio);
fprintf('\n');

% 检查系统可控性 Ctrb = ctrb(A, B);
if rank (Ctrb)
  < size(A, 1) warning('⚠️ 系统不可控！请检查模型！');
else
  fprintf('✓ 系统可控性检查通过\n\n');
end

    % 选择LQR设计模式 fprintf('请选择LQR设计模式:\n');
fprintf('  1 - 自动搜索最优参数\n');
fprintf('  2 - 手动指定Q、R值\n');
mode = input('输入选择 (1或2): ');

if mode
  == 2 % % 手动模式 fprintf('\n--- 手动参数设置模式 ---\n');
fprintf('Q矩阵 = diag([q1, q2])\n');
fprintf('  q1: 角度误差权重（推荐: 500~5000）\n');
fprintf('  q2: 角速度误差权重（推荐: 1~10）\n');
fprintf('R: 控制输入权重（推荐: 0.5~2）\n\n');

q1 = input('输入 q1 (默认1000): ');
if isempty (q1)
  , q1 = 1000;
end

    q2 = input('输入 q2 (默认5): ');
if isempty (q2)
  , q2 = 5;
end

    R = input('输入 R (默认1): ');
if isempty (R)
  , R = 1;
end

    Q = diag([ q1, q2 ]);
[ K, S, e ] = lqr(A, B, Q, R);

fprintf('\n设计参数:\n');
fprintf('  Q = diag([%.0f, %.0f])\n', q1, q2);
fprintf('  R = %.2f\n', R);
fprintf('  K = [%.6f, %.6f]\n\n', K(1), K(2));

else % % 自动搜索模式 fprintf('\n--- 自动参数搜索模式 ---\n');

% 参数搜索范围（Pitch轴可能需要更激进的响应） R_range = [ 0.5, 1, 2, 5 ];
q1_range = [ 500, 1000, 2000, 5000, 10000 ];
q2_range = [ 1, 2, 5, 10, 20 ];

best_score = inf;
results = [];

fprintf('正在搜索最优参数（Pitch轴）...\n');
total = length(R_range) * length(q1_range) * length(q2_range);
count = 0;

    for R = R_range
        for q1 = q1_range
            for q2 = q2_range
                count = count + 1;
    if mod (count, 50)
      == 0 fprintf('  进度: %d/%d\n', count, total);
    end

        Q = diag([ q1, q2 ]);
    [ K_temp, ~, ~] = lqr(A, B, Q, R);

    % 计算闭环特征 omega_n = sqrt(K_temp(1) * B(2));
    % 自然频率 zeta = K_temp(2) * B(2) / (2 * omega_n);
    % 阻尼比

                % 筛选条件（Pitch轴可能需要更快的响应） if zeta >
            0.3 &&
        zeta < 0.9 &&
        K_temp(1) <
            100 % 性能评分（优先考虑响应速度） score = (1.5 / omega_n / 0.05) ^
                                                       2;

    % 保存结果 results =
        [ results; q1, q2, R, K_temp(1), K_temp(2), omega_n, zeta, score ];

    if score
      < best_score best_score = score;
    K = K_temp;
    Q_best = Q;
    R_best = R;
    end end end end end

        fprintf('\n找到 %d 组可行参数\n', size(results, 1));
    fprintf('最优参数:\n');
    fprintf('  Q = diag([%.0f, %.0f])\n', Q_best(1, 1), Q_best(2, 2));
    fprintf('  R = %.2f\n', R_best);
    fprintf('  K = [%.6f, %.6f]\n\n', K(1), K(2));

    % 显示前5个候选参数 if ~isempty(results)[~, idx] = sort(results( :, 8));
    fprintf('前5个候选参数（Pitch轴）:\n');
    fprintf('   q1      q2      R     K_angle   K_vel    ωn     ζ\n');
    fprintf('------------------------------------------------------------\n');
        for
          i = 1
              : min(5, size(results, 1))
                    fprintf('%6.0f  %6.0f  %5.1f  %8.4f  %7.4f  %6.2f  %5.3f\n',
                            results(idx(i), :));
        end fprintf('\n');
        end end

                % %
            == == == == == 第6步：性能分析（Pitch轴） == == == == ==
            fprintf('========== 步骤6：性能分析（Pitch轴） ==========\n');

        % 闭环系统 A_cl = A - B * K;
        sys_cl = ss(A_cl, B, C, D);

        % 阶跃响应 figure('Position', [ 100, 100, 1200, 800 ], 'Name',
                          'Pitch轴LQR性能分析');
        subplot(2, 2, 1);
        step(sys_cl, 1);
        title('单位阶跃响应（Pitch轴）');
        grid on;

        try info = stepinfo(sys_cl);
        fprintf('预测性能（Pitch轴）:\n');
        fprintf('  上升时间: %.1f ms\n', info.RiseTime * 1000);
        fprintf('  调节时间: %.1f ms\n', info.SettlingTime * 1000);
        fprintf('  超调量: %.1f%%\n', info.Overshoot);
        catch fprintf('无法计算阶跃响应信息\n');
        end

            % 极点分析 poles = eig(A_cl);
        fprintf('  闭环极点: %.2f ± %.2fj\n', real(poles(1)),
                abs(imag(poles(1))));
        fprintf('  自然频率: %.2f rad/s\n', abs(poles(1)));
        fprintf('  阻尼比: %.3f\n\n', -real(poles(1)) / abs(poles(1)));

        % Bode图 subplot(2, 2, 2);
        bode(sys_cl);
        title('频域响应（Pitch轴）');
        grid on;

        % 控制输入分析 subplot(2, 2, 3);
        t_sim = 0 : 0.001 : 1;
        u_sim = lsim(-K * sys_cl.A, B, C, D, ones(size(t_sim)), t_sim);
        plot(t_sim, u_sim *(CAN_MAX / MaxCurrent_A), 'LineWidth', 1.5);
        xlabel('时间 (s)');
        ylabel('控制输入 (CAN值)');
        title('单位阶跃下的控制输入（Pitch轴）');
        grid on;
        ylim([ -CAN_MAX, CAN_MAX ]);

        % 根轨迹 subplot(2, 2, 4);
        sys_open = ss(A, B, C, D);
        rlocus(sys_open);
        title('根轨迹（Pitch轴）');
        grid on;

        % % == == == == == 生成嵌入式代码（Pitch轴专用） == == == == ==
            fprintf('========== 嵌入式代码（Pitch轴） ==========\n\n');

        fprintf('/* ========================================\n');
        fprintf(' * Pitch轴 LQR控制器参数 - 基于系统辨识\n');
        fprintf(' * ========================================\n');
        fprintf(' * 电机配置:\n');
        fprintf(' *   电机型号: M3508\n');
        fprintf(' *   减速比: %.0f:1\n', GearRatio);
        fprintf(' *   转矩常数: %.3f N·m/A\n', Kt);
        fprintf(' *   电流范围: ±%.0f A\n', MaxCurrent_A);
        fprintf(' *\n');
        fprintf(' * 系统参数:\n');
        fprintf(' *   输出轴转动惯量 J = %.6f kg·m²\n', J);
        fprintf(' *   阻尼系数 b = %.6f N·m·s/rad\n', b);
        fprintf(' *   拟合度 = %.1f%% (%s)\n', fit,
                iif(use_2nd_order, '二阶模型', '一阶模型'));
        fprintf(' *\n');
        fprintf(' * 重力补偿:\n');
        fprintf(' *   前馈CAN值: %.0f (约%.2f A)\n', GRAVITY_COMP_CAN,
                GRAVITY_COMP_A);
        fprintf(' *   估算重力矩: %.3f N·m\n', GRAVITY_TORQUE_ESTIMATE);
        fprintf(' *   注意：辨识在θ≈0附近进行，重力矩≈0\n');
        fprintf(' *\n');
        fprintf(' * LQR设计参数:\n');
        if exist ('Q_best', 'var')
          fprintf(' *   Q = diag([%.0f, %.0f])\n', Q_best(1, 1), Q_best(2, 2));
        fprintf(' *   R = %.2f\n', R_best);
        else fprintf(' *   Q = diag([%.0f, %.0f])\n', q1, q2);
        fprintf(' *   R = %.2f\n', R);
        end fprintf(' *\n');
        fprintf(' * 预期性能:\n');
        if exist ('info', 'var')
          fprintf(' *   上升时间 ≈ %.0f ms\n', info.RiseTime * 1000);
        fprintf(' *   超调量 ≈ %.1f%%\n', info.Overshoot);
        end fprintf(' *\n');
        fprintf(' * 注意事项:\n');
        fprintf(' *   1. 适用角度范围: -10.5° ~ 48.5°\n');
        fprintf(' *   2. LQR线性化在θ=0附近最准确\n');
        fprintf(' *   3. 大角度时可能需要增益调度或非线性控制\n');
        fprintf(' *   4. 积分项可用于抗重力扰动（可选）\n');
        fprintf(' * ========================================*/\n\n');

        fprintf('.LQR = {\n');
        fprintf('    .K_angle = %.6ff,      // 角度反馈增益 [A/rad]\n', K(1));
        fprintf('    .K_velocity = %.6ff,   // 角速度反馈增益 [A·s/rad]\n',
                K(2));
        fprintf(
            '    .K_integral = 0.0f,       // 积分增益（可选，用于抗重力扰动）\n');
        fprintf('    .max_out = %.2ff,         // 最大电流限制 [A]\n',
                MaxCurrent_A * 0.9);
        fprintf(
            '    .enable_integral = 0,     // 是否启用积分（建议测试后决定）\n');
        fprintf('    .integral_limit = 0.0f,   // 积分限幅\n');
        fprintf('    .integral_deadband = 0.0f, // 积分死区\n');
        fprintf('    .integral_decay_coef = 0.0f, // 积分衰减系数\n');
        fprintf('},\n\n');

        fprintf('/* 重力补偿前馈（当前使用固定值）*/\n');
        fprintf('static float gimbal_pitch_cur_ff = %.1ff;  // CAN值\n',
                GRAVITY_COMP_CAN);
        fprintf('// 或使用角度相关补偿（高级）:\n');
        fprintf('// gimbal_pitch_cur_ff = %.1ff * sin(pitch_angle);\n\n',
                GRAVITY_COMP_CAN);

        fprintf('========================================\n');
        fprintf('完成！请复制上述参数到 gimbal.c 中\n');
        fprintf('Pitch轴LQR配置完成\n');
        fprintf('========================================\n\n');

        % % 保存结果 save_file =
            fullfile(pathname, 'Pitch_LQR_Design_Results.mat');
        save(save_file, 'K', 'J', 'b', 'fit', 'sys_cl', 'A', 'B', 'GearRatio',
             'Kt');
        fprintf('结果已保存到: %s\n', save_file);

        % 辅助函数：三元运算符 function result =
            iif(condition, true_val, false_val) if condition result = true_val;
        else result = false_val;
        end end
