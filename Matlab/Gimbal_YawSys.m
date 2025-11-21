%%===========================================================================
%% 云台LQR控制器设计工具 - 优化版
%%===========================================================================
%% 功能：
%%   1. 从Ozone CSV数据进行系统辨识
%%   2. 自动或手动设计LQR增益
%%   3. 生成嵌入式代码
%%
%% 使用方法：
%%   1. 在Ozone中导出系统辨识数据（CSV格式）
%%   2. 运行本脚本
%%   3. 选择有效数据区间
%%   4. 选择自动搜索或手动调参模式
%%   5. 复制生成的参数到gimbal.c
%%
%% 作者：RoboMaster EC Team
%% 日期：2025-11-03
%%===========================================================================

clear;
close all;
clc;

%% ========== 电机参数定义 ==========
Kt = 0.741;              % 力矩常数 [N·m/A] (GM6020)
GearRatio = 1;           % 减速比（直驱）
CAN_MAX = 16384;         % CAN控制值最大值
MaxCurrent_A = 20;       % 最大额定电流 [A]

fprintf('\n========================================\n');
fprintf('    云台LQR控制器设计工具 v2.0\n');
fprintf('========================================\n\n');

%% ========== 第1步：读取数据 ==========
fprintf('========== 步骤1：读取CSV数据 ==========\n');

[filename, pathname] = uigetfile('*.csv', '选择Ozone导出的CSV文件');
if isequal(filename, 0)
    error('取消文件选择');
end

data_table = readtable(fullfile(pathname, filename));
fprintf('读取文件: %s\n', filename);

% 智能提取列
try
    if ismember('sysid_datastep_input', data_table.Properties.VariableNames)
        time = data_table.Time;
        input_CAN = data_table.sysid_datastep_input;
        output_omega_raw = data_table.sysid_datamotor_output;
    elseif ismember('Time', data_table.Properties.VariableNames)
        time = data_table.Time;
        input_CAN = data_table{:, 3};
        output_omega_raw = data_table{:, 4};
    else
        error('无法识别列名，请检查CSV格式');
    end
catch ME
    error('数据提取失败: %s', ME.message);
end

fprintf('✓ 数据点数: %d\n\n', length(time));

%% ========== 第2步：单位转换 ==========
fprintf('========== 步骤2：单位转换 ==========\n');

% CAN值 → 实际电流(A)
input_current_A = input_CAN / CAN_MAX * MaxCurrent_A;

% 角速度方向修正（电机反转 = 陀螺仪正转）
output_omega = -output_omega_raw;

fprintf('电流转换: CAN值 → 实际电流(A)\n');
fprintf('  转换系数: %.6f A/CAN\n', MaxCurrent_A / CAN_MAX);
fprintf('  范围: %.2f ~ %.2f A\n', min(input_current_A), max(input_current_A));
fprintf('\n角速度方向修正: 已取反\n\n');

%% ========== 第3步：数据选择 ==========
fprintf('========== 步骤3：选择有效数据 ==========\n');

figure('Position', [100, 100, 1400, 600], 'Name', '数据预览');
subplot(1, 2, 1);
plot(time, input_current_A, 'b', 'LineWidth', 1.5);
title('输入: 电流 [A]');
xlabel('时间 (s)');
ylabel('电流 (A)');
grid on;

subplot(1, 2, 2);
plot(time, output_omega, 'r', 'LineWidth', 1.5);
title('输出: 角速度 [rad/s]');
xlabel('时间 (s)');
ylabel('角速度 (rad/s)');
grid on;

fprintf('请点击图中两次选择数据区间（左键点击开始，再点击结束）...\n');
[x, ~] = ginput(2);
x = sort(x);

start_idx = find(time >= x(1), 1);
end_idx = find(time <= x(2), 1, 'last');

valid_time = time(start_idx:end_idx) - time(start_idx);
valid_current = input_current_A(start_idx:end_idx);
valid_omega = output_omega(start_idx:end_idx);

% 重采样处理非等距数据
dt_array = diff(valid_time);
if std(dt_array) / mean(dt_array) > 0.01
    fprintf('检测到非等距采样，进行重采样...\n');
    t_uni = (0:mean(dt_array):valid_time(end))';
    valid_current = interp1(valid_time, valid_current, t_uni);
    valid_omega = interp1(valid_time, valid_omega, t_uni);
    valid_time = t_uni;
end

fprintf('✓ 选择: %d 点, %.2f s\n\n', length(valid_time), valid_time(end));

%% ========== 第4步：系统辨识 ==========
fprintf('========== 步骤4：系统辨识 ==========\n');

Ts = mean(diff(valid_time));
fprintf('采样周期: %.4f s (%.1f Hz)\n', Ts, 1/Ts);

data_id = iddata(valid_omega, valid_current, Ts);

% 使用一阶系统辨识
sys_I_to_omega = tfest(data_id, 1, 0);

fprintf('\n传递函数 ω(s)/I(s):\n');
disp(tf(sys_I_to_omega));

[num, den] = tfdata(sys_I_to_omega, 'v');
K_omega = num(end);
a = den(2);

% 物理参数
J = Kt / K_omega;
b = a * J;

fprintf('辨识物理参数:\n');
fprintf('  转动惯量 J = %.6f kg·m²\n', J);
fprintf('  阻尼系数 b = %.6f N·m·s/rad\n', b);

% 验证拟合度
y_sim = lsim(sys_I_to_omega, valid_current, valid_time);
fit = 100 * (1 - norm(valid_omega - y_sim) / norm(valid_omega - mean(valid_omega)));
fprintf('  拟合度: %.1f%%\n\n', fit);

% 绘制拟合结果
figure('Position', [100, 100, 1200, 400], 'Name', '系统辨识结果');
plot(valid_time, valid_omega, 'b', 'LineWidth', 1.5, 'DisplayName', '实际输出');
hold on;
plot(valid_time, y_sim, 'r--', 'LineWidth', 1.5, 'DisplayName', '模型预测');
xlabel('时间 (s)');
ylabel('角速度 (rad/s)');
title(sprintf('系统辨识拟合结果 (拟合度: %.1f%%)', fit));
legend('Location', 'best');
grid on;
hold off;

%% ========== 第5步：LQR设计 ==========
fprintf('========== 步骤5：LQR设计 ==========\n');

% 状态空间模型
A = [0, 1; 
     0, -b/J];
B = [0; Kt/J];
C = [1, 0];
D = 0;

fprintf('状态空间模型:\n');
fprintf('  A = [0, 1; 0, %.4f]\n', -b/J);
fprintf('  B = [0; %.4f]\n\n', Kt/J);

% 选择LQR设计模式
fprintf('请选择LQR设计模式:\n');
fprintf('  1 - 自动搜索最优参数\n');
fprintf('  2 - 手动指定Q、R值\n');
mode = input('输入选择 (1或2): ');

if mode == 2
    %% 手动模式
    fprintf('\n--- 手动参数设置模式 ---\n');
    fprintf('Q矩阵 = diag([q1, q2])\n');
    fprintf('  q1: 角度误差权重（推荐: 500~5000）\n');
    fprintf('  q2: 角速度误差权重（推荐: 1~10）\n');
    fprintf('R: 控制输入权重（推荐: 0.5~2）\n\n');
    
    q1 = input('输入 q1 (默认1000): ');
    if isempty(q1), q1 = 1000; end
    
    q2 = input('输入 q2 (默认5): ');
    if isempty(q2), q2 = 5; end
    
    R = input('输入 R (默认1): ');
    if isempty(R), R = 1; end
    
    Q = diag([q1, q2]);
    [K, S, e] = lqr(A, B, Q, R);
    
    fprintf('\n设计参数:\n');
    fprintf('  Q = diag([%.0f, %.0f])\n', q1, q2);
    fprintf('  R = %.2f\n', R);
    fprintf('  K = [%.6f, %.6f]\n\n', K(1), K(2));
    
else
    %% 自动搜索模式
    fprintf('\n--- 自动参数搜索模式 ---\n');
    
    % 参数搜索范围（可以自定义）
    R_range = [0.5, 1, 2, 5];
    q1_range = [500, 1000, 2000, 5000, 10000];
    q2_range = [1, 2, 5, 10, 20];
    
    best_score = inf;
    results = [];
    
    fprintf('正在搜索最优参数...\n');
    total = length(R_range) * length(q1_range) * length(q2_range);
    count = 0;
    
    for R = R_range
        for q1 = q1_range
            for q2 = q2_range
                count = count + 1;
                if mod(count, 50) == 0
                    fprintf('  进度: %d/%d\n', count, total);
                end
                
                Q = diag([q1, q2]);
                [K_temp, ~, ~] = lqr(A, B, Q, R);
                
                % 计算闭环特征
                omega_n = sqrt(K_temp(1) * B(2));  % 自然频率
                zeta = K_temp(2) * B(2) / (2 * omega_n);  % 阻尼比
                
                % 筛选条件
                if zeta > 0.3 && zeta < 0.9 && K_temp(1) < 50
                    % 性能评分（优先考虑响应速度）
                    score = (1.8 / omega_n / 0.05)^2;
                    
                    % 保存结果
                    results = [results; q1, q2, R, K_temp(1), K_temp(2), omega_n, zeta, score];
                    
                    if score < best_score
                        best_score = score;
                        K = K_temp;
                        Q_best = Q;
                        R_best = R;
                    end
                end
            end
        end
    end
    
    fprintf('\n找到 %d 组可行参数\n', size(results, 1));
    fprintf('最优参数:\n');
    fprintf('  Q = diag([%.0f, %.0f])\n', Q_best(1,1), Q_best(2,2));
    fprintf('  R = %.2f\n', R_best);
    fprintf('  K = [%.6f, %.6f]\n\n', K(1), K(2));
    
    % 显示前5个候选参数
    if ~isempty(results)
        [~, idx] = sort(results(:, 8));
        fprintf('前5个候选参数:\n');
        fprintf('   q1      q2      R     K_angle   K_vel    ωn     ζ\n');
        fprintf('------------------------------------------------------------\n');
        for i = 1:min(5, size(results, 1))
            fprintf('%6.0f  %6.0f  %5.1f  %8.4f  %7.4f  %6.2f  %5.3f\n', results(idx(i), :));
        end
        fprintf('\n');
    end
end

%% ========== 第6步：性能分析 ==========
fprintf('========== 步骤6：性能分析 ==========\n');

% 闭环系统
A_cl = A - B * K;
sys_cl = ss(A_cl, B, C, D);

% 阶跃响应
figure('Position', [100, 100, 1200, 800], 'Name', 'LQR性能分析');
subplot(2, 2, 1);
step(sys_cl, 1);
title('单位阶跃响应');
grid on;

try
    info = stepinfo(sys_cl);
    fprintf('预测性能:\n');
    fprintf('  上升时间: %.1f ms\n', info.RiseTime * 1000);
    fprintf('  调节时间: %.1f ms\n', info.SettlingTime * 1000);
    fprintf('  超调量: %.1f%%\n', info.Overshoot);
catch
    fprintf('无法计算阶跃响应信息\n');
end

% 极点分析
poles = eig(A_cl);
fprintf('  闭环极点: %.2f ± %.2fj\n', real(poles(1)), abs(imag(poles(1))));
fprintf('  自然频率: %.2f rad/s\n', abs(poles(1)));
fprintf('  阻尼比: %.3f\n\n', -real(poles(1))/abs(poles(1)));

% Bode图
subplot(2, 2, 2);
bode(sys_cl);
title('频域响应');
grid on;

% 控制输入分析
subplot(2, 2, 3);
t_sim = 0:0.001:1;
u_sim = lsim(-K * sys_cl.A, B, C, D, ones(size(t_sim)), t_sim);
plot(t_sim, u_sim * 819.2, 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('控制输入 (CAN值)');
title('单位阶跃下的控制输入');
grid on;
ylim([-CAN_MAX, CAN_MAX]);

% 根轨迹
subplot(2, 2, 4);
sys_open = ss(A, B, C, D);
rlocus(sys_open);
title('根轨迹');
grid on;

%% ========== 生成嵌入式代码 ==========
fprintf('========== 嵌入式代码 ==========\n\n');

fprintf('/* ========================================\n');
fprintf(' * LQR控制器参数 - 基于系统辨识\n');
fprintf(' * ========================================\n');
fprintf(' * 系统参数:\n');
fprintf(' *   转动惯量 J = %.6f kg·m²\n', J);
fprintf(' *   阻尼系数 b = %.6f N·m·s/rad\n', b);
fprintf(' *   拟合度 = %.1f%%\n', fit);
fprintf(' *\n');
fprintf(' * LQR设计参数:\n');
if exist('Q_best', 'var')
    fprintf(' *   Q = diag([%.0f, %.0f])\n', Q_best(1,1), Q_best(2,2));
    fprintf(' *   R = %.2f\n', R_best);
else
    fprintf(' *   Q = diag([%.0f, %.0f])\n', q1, q2);
    fprintf(' *   R = %.2f\n', R);
end
fprintf(' *\n');
fprintf(' * 预期性能:\n');
if exist('info', 'var')
    fprintf(' *   上升时间 ≈ %.0f ms\n', info.RiseTime * 1000);
    fprintf(' *   超调量 ≈ %.1f%%\n', info.Overshoot);
end
fprintf(' * ========================================*/\n\n');

fprintf('.LQR = {\n');
fprintf('    .K_angle = %.6ff,      // 角度反馈增益 [A/rad]\n', K(1));
fprintf('    .K_velocity = %.6ff,   // 角速度反馈增益 [A·s/rad]\n', K(2));
fprintf('    .K_integral = 0.0f,       // 积分增益（Yaw轴一般不需要）\n');
fprintf('    .max_out = %.2ff,         // 最大电流限制 [A]\n', MaxCurrent_A * 0.9);
fprintf('    .enable_integral = 0,     // 禁用积分\n');
fprintf('    .integral_limit = 0.0f,\n');
fprintf('    .integral_deadband = 0.0f,\n');
fprintf('    .integral_decay_coef = 0.0f,\n');
fprintf('},\n\n');

fprintf('========================================\n');
fprintf('完成！请复制上述参数到 gimbal.c 中\n');
fprintf('========================================\n\n');

%% 保存结果
save_file = fullfile(pathname, 'LQR_Design_Results.mat');
save(save_file, 'K', 'J', 'b', 'fit', 'sys_cl', 'A', 'B');
fprintf('结果已保存到: %s\n', save_file);