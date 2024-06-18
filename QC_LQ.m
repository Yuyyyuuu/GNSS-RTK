% 文件路径
filename = 'Result_filetestLQ_dy.txt';

% 初始化变量
time = [];
E = [];
N = [];
U = [];
pdop = [];
FixRMS = [];
satellites = [];
ratio = [];
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
    
    % 确保当前行有足够的数据
    if length(parts) >= 9
        % 转换为数值类型，除了fixed状态
        time = [time; str2double(parts{1})];
        E = [E; str2double(parts{2})];
        N = [N; str2double(parts{3})];
        U = [U; str2double(parts{4})];
        FixRMS = [FixRMS; str2double(parts{5})];
        % 第五列不需要，所以从第六列开始读取
        pdop = [pdop; str2double(parts{6})];
        satellites = [satellites; str2double(parts{7})];
        ratio = [ratio; str2double(parts{8})];
        
        % fixed状态转换为逻辑值
        fixed = [fixed; strcmp(parts{9}, '固定解')];
    else
        warning('跳过不完整的数据行：%s', line);
    end
end

% 关闭文件
fclose(fid);

% 创建一个新的画布
figure('Name','LQ RTK pos-t','NumberTitle','off');

% 设置画布的大小
set(gcf, 'Position', [100, 100, 800, 2400]);

% 第一个子图：dE坐标随时间变化
subplot('Position', [0.1, 0.7, 0.8, 0.2]);
plot(time, E);
title('E-t');
xlabel('Time(s)');
ylabel('E (m)');


% 第二个子图：dN坐标随时间变化
subplot('Position', [0.1, 0.4, 0.8, 0.2]);
plot(time, N);
title('N-t');
xlabel('Time(s)');
ylabel('N (m)');


% 第三个子图：dU坐标随时间变化
subplot('Position', [0.1, 0.1, 0.8, 0.2]);
plot(time, U);
title('U-t');
xlabel('Time(s)');
ylabel('U (m)');
box on;

% 显示图形
drawnow;

% 计算均值
mean_E = mean(E);
mean_N = mean(N);
mean_U = mean(U);

% 计算 dE、dN、dU
dE = E - mean_E;
dN = N - mean_N;
dU = U - mean_U;



% 创建一个新的画布
figure('Name','LQ RTK error-t','NumberTitle','off');

% 设置画布的大小
set(gcf, 'Position', [100, 100, 800, 2400]);

% 第一个子图：dE坐标随时间变化
subplot('Position', [0.1, 0.7, 0.8, 0.2]);
plot(time, dE);
title('dE-t');
xlabel('Time(s)');
ylabel('dE (m)');


% 第二个子图：dN坐标随时间变化
subplot('Position', [0.1, 0.4, 0.8, 0.2]);
plot(time, dN);
title('dN-t');
xlabel('Time(s)');
ylabel('dN (m)');


% 第三个子图：dU坐标随时间变化
subplot('Position', [0.1, 0.1, 0.8, 0.2]);
plot(time, dU);
title('dU-t');
xlabel('Time(s)');
ylabel('dU (m)');
box on;

% 显示图形
drawnow;

% 计算RMS_E, RMS_N, RMS_U
RMS_E = sqrt(sum(dE.^2) / (numel(dE) - 1));
RMS_N = sqrt(sum(dN.^2) / (numel(dN) - 1));
RMS_U = sqrt(sum(dU.^2) / (numel(dU) - 1));

% 打印RMS_E, RMS_N, RMS_U
fprintf('RMS_E: %f\n', RMS_E);
fprintf('RMS_N: %f\n', RMS_N);
fprintf('RMS_U: %f\n', RMS_U);


% 创建FixRMS时序图的画布
figure('Name','FixRMS','NumberTitle','off');
plot(time, FixRMS);
title('FixRMS Time Series');
xlabel('Time (s)');
ylabel('FixRMS');
grid on; % 添加网格线


% 创建 PDOP 时序图的画布
figure('Name','PDOP Time Series','NumberTitle','off');
plot(time, pdop);
title('PDOP Time Series');
xlabel('Time (s)');
ylabel('PDOP');
grid on; % 添加网格线

% 创建卫星数时序图的画布
figure('Name','Satellites Time Series','NumberTitle','off');
plot(time, satellites);
title('Satellites Time Series');
xlabel('Time (s)');
ylabel('Number of Satellites');
grid on; % 添加网格线

% 创建 ratio 时序图的画布
figure('Name','Ratio Time Series','NumberTitle','off');

% 首先绘制所有的 ratio 数据点
plot(time, ratio, 'g', 'Marker', 'o', 'MarkerFaceColor', 'g', 'MarkerSize', 2);
hold on; % 保持当前图形，以便在同一图形上绘制其他数据

% 找出 ratio 值小于3 的数据点
idx_low = ratio < 3;

% 绘制 ratio 值小于3 的点，以红色显示，且不连线
plot(time(idx_low), ratio(idx_low), 'y', 'Marker', 'o', 'MarkerFaceColor', 'y', 'MarkerSize', 2, 'LineStyle', 'none');

% 其他图形设置
title('Ratio Time Series');
xlabel('Time (s)');
ylabel('Ratio');
grid on; % 添加网格线

% 注意：由于红色点没有连线，图例中的 'Ratio < 3' 项将代表未连线的点
legend('All Data', 'Ratio < 3');

% 释放 hold 状态
hold off;

% 显示图形
drawnow;

% 找出 ratio 值小于3 的索引
idx_low = ratio < 3;

% 计算小于3的 ratio 数量
count_low = sum(idx_low);

% 计算 ratio 的总数
total_count = numel(ratio);

% 计算小于3的比例
proportion_low = count_low / total_count;

% 打印比例
fprintf('Ratio 值小于 3 的比例: %.2f%%\n', proportion_low * 100);


% 创建一个新的画布用于绘制 E-N 轨迹图
figure('Name', 'E-N Trajectory', 'NumberTitle', 'off');

% 检查数据是否足够
if ~isempty(E) && ~isempty(N) && ~isempty(time)
   
    
    % 绘制 E-N 轨迹图
    plot(E, N, 'o-');  % 使用线和点标记轨迹
    hold on;  % 保持当前图形，以便添加其他元素
    
    % 根据数据范围设置坐标轴比例
    axis equal;
    
    % 根据 ratio 值绘制不同颜色的点
    idx_high = ratio >= 3;
    idx_low = ratio < 3;
    plot(E(idx_high), N(idx_high), 'g.', 'MarkerSize', 8);  % Ratio >= 3, 绿色点
    plot(E(idx_low), N(idx_low), 'y.', 'MarkerSize', 8);   % Ratio < 3, 黄色点
    
    % 添加图形属性
    title('E-N Trajectory');
    xlabel('E (Eastings) (m)');
    ylabel('N (Northings) (m)');
    grid on;  % 添加网格线
    
    % 添加图例
    legend('All Trajectory', 'High Ratio', 'Low Ratio');
    
    % 显示图形
    drawnow;
else
    warning('E、N 或时间数据为空，无法绘制 E-N 轨迹图。');
end

% 释放 hold 状态
hold off;
