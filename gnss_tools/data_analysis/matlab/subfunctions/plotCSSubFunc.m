function [figure1] = plotCSSubFunc(x1, y1, plotOrder, legendName, xLim, yLim, xLabel, yLabel, Title, lineType, showContent, figSize, ticks, cs)
% plotSeries 绘制Y1随X1变化的曲线，可以选择折线图或散点图
% 输入：
%   x1: 包含x轴数据的cell数组。可以是n列（每列一个x数据），或者只有一个列（一个x数据）
%   y1: 包含y轴数据的cell数组，长度为n，表示每条曲线的y值
%   plotOrder: y轴数据绘制顺序，默认为 []
%   legendName: 图例的名称，默认为 []
%   xLim: x轴范围，格式为 [xmin, xmax]，默认为 []
%   yLim: y轴范围，格式为 [ymin, ymax]，默认为 []
%   xLabel: x轴标签，默认为 []
%   yLabel: y轴标签，默认为 []
%   lineType: 绘图类型，1表示散点图，2表示折线图
%   showContent: 在图中显示的文本内容，默认为 [] 
%   figSize: 图形大小选择，1=4.3cm×2，2=6.45cm×2，3=8.4cm×2
% 输出：
%   figure1: 绘制的图形句柄

fontName = "Arial";
fontSize = 16;
gcfPos = [10, 10, 8.4*2, 7.0*2]; % 'Units', 'centimeters'
gcaPos = [0.085, 0.12, 0.87, 0.82]; % 'Units', 'centimeters'
colors = {[20,169,89]/255, [255,204,102]/255, [132,94,194]/255, [255,102,102]/255};

% 创建图形
figure1 = figure;
set(figure1, 'Color', 'w');  % 设置背景色为白色
handles = gobjects(1, numel(y1));  % 初始化句柄
hold on;

% 根据plotOrder重新排序x1和y1
if ~isempty(plotOrder)
    y1 = y1(plotOrder);  % 根据plotOrder重新排序y1
    if numel(x1) == numel(y1)  % n对n的情况
        x1 = x1(plotOrder);  % 根据plotOrder重新排序x1
    end
else
    plotOrder = 1:numel(y1);
end

% 绘制每条曲线
for i = 1:numel(y1)
    % 确定当前数据的X和Y值
    if numel(x1) == 1  % 1对n的情况
        xData = x1{1};  % 使用X1中的第一个元素作为X轴数据
    elseif numel(x1) == numel(y1)  % n对n的情况
        xData = x1{i};  % 选择对应的X数据
    else  % 0对n的情况
        xData = (1:numel(y1{i}))';  % 使用索引作为X轴数据
    end
    
    yData = y1{i};  % 选择对应的Y数据
    
    % 根据绘图类型绘制不同的图形
    if lineType == 1  % 散点图
        handles(plotOrder(i)) = scatter(xData, yData, 15, colors{i}, 'filled');
    else  % 折线图
        handles(plotOrder(i)) = plot(xData, yData, '.-', 'Color', colors{i}, 'LineWidth', 1.0, 'MarkerSize', 16);
    end
    
    % 绘制竖线，代表周跳
    for j=1:size(cs,1)
        plot([cs(j,1), cs(j,1)], [cs(j,2)-0.3, cs(j,2)+0.3], 'Color', colors{cs(j,3)+1}, 'LineWidth', 2.0);  % 调整竖线的高度
    end
end

hold off;

% 设置坐标轴标签
if ~isempty(xLabel)
    xlabel(xLabel);
end
if ~isempty(yLabel)
    ylabel(yLabel);
end

% 设置标题
title(Title);

% 设置网格线样式
set(gca, 'XGrid', 'off', 'GridLineStyle', '--', 'GridAlpha', 0.2);  % 关闭 x 轴网格，设置网格线样式
set(gca, 'YGrid', 'on', 'GridLineStyle', '--', 'GridAlpha', 0.2);   % 开启 y 轴网格，设置网格线样式

% 设置坐标轴范围
if ~isempty(xLim)
    set(gca, 'XLim', [xLim(1), xLim(2)]);
end
if ~isempty(yLim)
    set(gca, 'YLim', [yLim(1), yLim(2)], 'YTick', yLim(1):(yLim(2) / 2):yLim(2));
end

% 设置原本 y 轴刻度不可见
set(gca, 'YTickLabel', []);  % 隐藏 y 轴的刻度标签

% 设置新的 y 轴刻度
yticks(1:length(ticks));  % y 轴位置，ticks 长度对应的刻度位置

% 设置 y 轴的刻度标签
yticklabels(ticks);  % 设置 y 轴的标签为 ticks 的内容

% % 调整刻度标签的对齐方式，确保左对齐
% ax = gca;
% ax.YTickLabelRotation = 0;  % 设置标签的旋转角度为 0°，保持水平显示
% set(gca, 'YTickLabel', ticks);  % 重新设置 y 轴标签
% % 通过增加 'HorizontalAlignment' 参数来确保刻度标签左对齐
% for i = 1:length(ticks)
%     text(-0.5, i, ticks{i}, 'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle', 'FontSize', 12, 'FontName', 'Arial');
% end

% 设置 y 轴为对数刻度
% set(gca, 'YScale', 'log');  % 将 y 轴设置为对数刻度
% set(gca, 'YTick', [0.01, 0.1, 1, 10]);  % 设置 y 轴刻度为 10^{-1}, 10^{0}, 10^{1}
% set(gca, 'YTickLabel', {'10^{-2}', '10^{-1}', '10^{0}', '10^{1}'});  % 设置刻度标签

% 设置坐标轴边框，并调整边框的线宽
set(gca, 'Box', 'on', 'LineWidth', 2);

% 设置图形的尺寸与坐标轴位置
set(gca, 'Position', gcaPos);  % 设置坐标轴位置
set(gcf, 'Units', 'centimeters', 'Position', gcfPos);  % 设置图形的单位与位置

% 设置图例
if ~isempty(legendName)
    legendHandles = legend(handles(1:length(legendName)), legendName);
    set(legendHandles, 'Orientation', 'horizontal', 'Location', 'best', 'FontName', fontName, 'FontSize', fontSize);
    % set(legendHandles, 'Position', [0.579956824216459 0.224667777324603 0.352459012118519 0.0914826474919305], 'Orientation', 'horizontal', 'FontName', fontName, 'FontSize', fontSize);
end

% 设置图形的字体格式（应在最后设置，以保证不被覆盖）
set(gca, 'FontSize', fontSize, 'FontName', fontName, 'LineWidth', 1.0);

% 如果有显示的文本内容，添加文本框
if ischar(showContent) && ~isempty(showContent)
    annotation(figure1, 'textbox', ...
        [0.15 0.20 0.80 0.10], ...
        'Color', 'r', ...
        'String', showContent, ...
        'LineStyle', 'none', ...
        'FontName', fontName, ...
        'FontSize', fontSize, ...
        'FitBoxToText', 'off');
end

end
