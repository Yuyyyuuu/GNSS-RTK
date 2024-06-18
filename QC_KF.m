% 文件路径
filename = 'Result_filetestEKF_dy.txt';

% 初始化变量
time = [];
E = [];
N = [];
U = [];
satellites = [];
sigmaP = [];
fixed = [];

% 打开文件用于读取
fid = fopen(filename, 'r');

% 检查文件是否成功打开
if fid == -1
    error('文件无法打开，请检查路径和文件名是否正确。');
end

% 逐行读取文件
while ~feof(fid)
    line = fgetl(fid); % 读取一行
    parts = strsplit(line, ' '); % 按空格分割
    
    % 转换为数值类型，除了fixed状态
    time = [time; str2double(parts{1})];
    E = [E; str2double(parts{2})];
    N = [N; str2double(parts{3})];
    U = [U; str2double(parts{4})];
    satellites = [satellites; str2double(parts{5})];
    sigmaP = [sigmaP; str2double(parts{6})];
    fixed = [fixed; str2double(parts{7})];
end

% 关闭文件
fclose(fid);

% 1. 在一个画布上绘制E、N、U各自的时序图
figure('Name', 'Position Time Series', 'NumberTitle', 'off');
subplot(3, 1, 1);
plot(time, E);
title('E Time Series');
xlabel('Time (s)');
ylabel('E (m)');
grid on;

subplot(3, 1, 2);
plot(time, N);
title('N Time Series');
xlabel('Time (s)');
ylabel('N (m)');
grid on;

subplot(3, 1, 3);
plot(time, U);
title('U Time Series');
xlabel('Time (s)');
ylabel('U (m)');
grid on;

% 2. 计算dE、dN、dU,并在一个画布上绘制dE、dN、dU各自的时序图
mean_E = mean(E);
mean_N = mean(N);
mean_U = mean(U);
dE = E - mean_E;
dN = N - mean_N;
dU = U - mean_U;

figure('Name', 'Deviation Time Series', 'NumberTitle', 'off');
subplot(3, 1, 1);
plot(time, dE);
title('dE Time Series');
xlabel('Time (s)');
ylabel('dE (m)');
grid on;

subplot(3, 1, 2);
plot(time, dN);
title('dN Time Series');
xlabel('Time (s)');
ylabel('dN (m)');
grid on;

subplot(3, 1, 3);
plot(time, dU);
title('dU Time Series');
xlabel('Time (s)');
ylabel('dU (m)');
grid on;

% 3. 统计E、N、U各自的RMS并打印
RMS_E = sqrt(sum(dE.^2) / numel(dE));
RMS_N = sqrt(sum(dN.^2) / numel(dN));
RMS_U = sqrt(sum(dU.^2) / numel(dU));
fprintf('RMS_E: %f\nRMS_N: %f\nRMS_U: %f\n', RMS_E, RMS_N, RMS_U);

% 4. 在一个画布上单独绘制卫星数时序图
figure('Name', 'Satellites Time Series', 'NumberTitle', 'off');
plot(time, satellites);
title('Satellites Time Series');
xlabel('Time (s)');
ylabel('Number of Satellites');
grid on;

% 5. 在一个画布上单独绘制sigmaP时序图
figure('Name', 'Sigma P Time Series', 'NumberTitle', 'off');
plot(time, sigmaP);
title('Sigma P Time Series');
xlabel('Time (s)');
ylabel('Sigma P');
grid on;

%% 6. 统计固定解的比例，并绘制固定解时序图
% 计算固定解的数量
count_fixed = sum(fixed);  % 直接使用 sum 函数计算固定解的数量（fixed 数组中的 1 的个数）

% 计算固定解的比例
proportion_fixed = count_fixed / numel(fixed);  % 计算固定解占总观测数的比例
fprintf('固定解的比例: %.2f%%\n', proportion_fixed * 100);  % 打印固定解的比例

% 显示图形
drawnow;

% 创建一个新的画布用于绘制 E-N 轨迹图
figure('Name', 'E-N Trajectory', 'NumberTitle', 'off');

% 检查数据是否足够
if ~isempty(E) && ~isempty(N) && ~isempty(time)
    % 绘制 E-N 轨迹图
    plot(E, N, 'o-');  % 使用线和点标记轨迹
    hold on;  % 保持当前图形，以便添加其他元素
    
    % 添加图形属性
    title('E-N Trajectory');
    xlabel('E (Eastings) (m)');
    ylabel('N (Northings) (m)');
    grid on;  % 添加网格线
    
    % 显示图形
    drawnow;
    
    % 设置坐标轴的刻度字体大小
    set(gca, 'FontSize', 10);
    
    % 添加颜色填充，使轨迹更加清晰
    set(gca, 'Color', 'none');
    
    % 优化图形显示效果
    box on;  % 绘制边框
    set(gca, 'LineWidth', 1);  % 设置坐标轴线条宽度
else
    warning('E、N 或时间数据为空，无法绘制 E-N 轨迹图。');
end

% 释放 hold 状态
hold off;