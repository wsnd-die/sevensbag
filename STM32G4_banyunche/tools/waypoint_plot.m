%% waypoint_plot.m
%  用法:
%    1) 从串口复制数据粘贴为文本文件, 改下面的 filename
%    2) 直接运行: F5 或 命令行输入 waypoint_plot
%
%  数据格式 (STM32 UART3 输出的 CSV):
%    === WAYPOINT START (N points) ===
%    0,0.0000,0.0000,0.00
%    1,500.0000,0.0000,0.00
%    ...
%    === WAYPOINT END ===

clear; close all;

% ========================= 配置 =========================
filename = 'waypoints.txt';   % 串口导出的文本文件
% ========================================================

%% 1. 读取文件
if ~exist(filename, 'file')
    error('文件不存在: %s\n请先将串口输出保存为 %s', filename, filename);
end

fid = fopen(filename, 'r');
if fid < 0
    error('无法打开: %s', filename);
end

data = [];   % [idx, x, y, yaw]
while ~feof(fid)
    line = strtrim(fgetl(fid));
    if isempty(line) || ~contains(line, ',')
        continue;
    end
    % 跳过 WAYPOINT 标记行
    if contains(line, 'WAYPOINT')
        continue;
    end
    % 解析 CSV
    parts = strsplit(line, ',');
    if length(parts) >= 4
        data(end+1, :) = [str2double(parts{1}), ...
                          str2double(parts{2}), ...
                          str2double(parts{3}), ...
                          str2double(parts{4})];
    end
end
fclose(fid);

if isempty(data)
    error('未解析到有效数据, 请检查文件格式');
end

idx = data(:,1);
x   = data(:,2);
y   = data(:,3);
yaw = data(:,4);
N   = length(x);

fprintf('加载完成: %d 个路径点\n', N);
fprintf('起点: (%.1f, %.1f) mm, yaw=%.1f°\n', x(1), y(1), yaw(1));
fprintf('终点: (%.1f, %.1f) mm, yaw=%.1f°\n', x(N), y(N), yaw(N));

%% 2. 绘图
figure('Name', 'Waypoint 路径回放', ...
       'NumberTitle', 'off', ...
       'Position', [100, 100, 1400, 600]);

% ---------- 左图: 路径轨迹 ----------
subplot(1, 2, 1);
hold on; grid on; axis equal;

% 2.1 绘制路径线
plot(x, y, 'b-', 'LineWidth', 1.5);
plot(x, y, 'b.', 'MarkerSize', 6);

% 2.2 起点 / 终点 标注
h1 = scatter(x(1), y(1), 120, 'g', 'o', 'filled', ...
             'DisplayName', sprintf('Start (%.0f,%.0f)', x(1), y(1)));
h2 = scatter(x(N), y(N), 150, 'r', 'x', 'LineWidth', 2.5, ...
             'DisplayName', sprintf('End (%.0f,%.0f)', x(N), y(N)));

% 2.3 朝向箭头 (每 N/20 个点画一个)
step = max(1, floor(N / 20));
arrow_len = (max(x) - min(x) + max(y) - min(y)) / 40;  % 自适应箭头长度
for i = 1:step:N
    rad = deg2rad(yaw(i));
    quiver(x(i), y(i), cos(rad) * arrow_len, sin(rad) * arrow_len, ...
           'r', 'LineWidth', 1.2, 'MaxHeadSize', 1.2, ...
           'AutoScale', 'off');
end

% 2.4 每 5 个箭头标注一次索引号
for i = 1:step*5:N
    text(x(i) + arrow_len*0.5, y(i) + arrow_len*0.5, ...
         num2str(idx(i)), 'FontSize', 7, 'Color', [0.3 0.3 0.3]);
end

xlabel('X (mm)', 'FontSize', 11);
ylabel('Y (mm)', 'FontSize', 11);
title(sprintf('Waypoint 路径轨迹 (%d 点)', N), 'FontSize', 13);
legend([h1, h2], 'Location', 'best');

% ---------- 右图: 偏航角剖面 ----------
subplot(1, 2, 2);
hold on; grid on;

% yaw 曲线
plot(idx, yaw, 'g-', 'LineWidth', 1.5);
% 用圆点标出角度突变 > 30° 的位置
dyaw = abs(diff(yaw));
jump_idx = find([0; dyaw] > 30);
if ~isempty(jump_idx)
    plot(idx(jump_idx), yaw(jump_idx), 'ro', ...
         'MarkerSize', 8, 'LineWidth', 1.2, ...
         'DisplayName', sprintf('Yaw Jump >30° (%d处)', length(jump_idx)));
end

% 参考线
yline(0,   'k--', 'LineWidth', 0.5);
yline(90,  'k:',  'LineWidth', 0.5);
yline(-90, 'k:',  'LineWidth', 0.5);
yline(180, 'k:',  'LineWidth', 0.5);
yline(-180,'k:',  'LineWidth', 0.5);

xlabel('Waypoint Index', 'FontSize', 11);
ylabel('Yaw (deg)', 'FontSize', 11);
title('偏航角剖面', 'FontSize', 13);
legend('Location', 'best');

sgtitle(sprintf('Waypoint 数据分析 — %s', filename), 'FontSize', 14);

%% 3. 输出统计
fprintf('\n=== 路径统计 ===\n');

% 总路径长度 (逐段累加)
seg_dists = sqrt(diff(x).^2 + diff(y).^2);
total_dist = sum(seg_dists);
fprintf('总路径长度 : %.1f mm (%.2f m)\n', total_dist, total_dist / 1000);

% 直线距离 (起点→终点)
straight_dist = sqrt((x(N)-x(1))^2 + (y(N)-y(1))^2);
fprintf('起点→终点   : %.1f mm (%.2f m)\n', straight_dist, straight_dist / 1000);
fprintf('弯曲度      : %.2f (总长/直线距)\n', total_dist / max(straight_dist, 1));

% yaw 范围
fprintf('Yaw 范围     : %.1f° ~ %.1f°\n', min(yaw), max(yaw));
fprintf('Yaw 总变化   : %.1f°\n', sum(abs(diff(yaw))));
fprintf('平均段长     : %.1f mm/点\n', mean(seg_dists));