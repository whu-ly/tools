function [] = plotCS(fileName)
% plotCS 绘制信号相位跟踪图及半周跳信噪比分布图
%
% 输入参数:
%   fileName - 数据文件路径，应为包含格式规范数据的 .txt 文件
%
% 功能说明:
%   1. 读取数据文件
%   2. 分频点绘制相位跟踪图（Phase Lock）
%   3. 分频点绘制半周跳信噪比（SNR）统计柱状图
%
% 注意：需要自定义函数 plotCSSubFunc 和 plotBars 支持绘图

% 读取数据
data1 = readtable(fileName, 'ReadVariableNames', false);
data1 = num2cell(table2cell(data1), 1);  % 按列分组为元胞
[filePath, ~, ~] = fileparts(fileName);  % 提取文件所在路径

% 配置
sysNames = {'G','E','C'};  % 卫星系统名称：GPS、Galileo、BDS
frqNames = {'L1','E1','B1I'; 'L2','E5b','B1C'; 'L5','E5a','B2a'};  % 频点对应表

% 提取数据列
time = cell2mat(data1{2});        % 时间
sats = data1{3};                  % 卫星编号（如 G03）
frqs = cell2mat(data1{5});        % 频点编号（0/1/2）
snrs = cell2mat(data1{6});        % 信噪比
slip = cell2mat(data1{9});        % 周跳标志值

% 按频点绘图
for f = 0:2
    % 筛选当前频点数据
    idx = frqs == f;
    time1 = time(idx);
    sats1 = sats(idx);
    snrs1 = snrs(idx);
    slip1 = slip(idx);

    % 获取当前频点卫星编号并排序（G/E/C + 卫星号）
    sats1Unique = unique(sats1, 'stable');
    letters = cellfun(@(x) x(1), sats1Unique, 'UniformOutput', false);
    numbers = cellfun(@(x) str2double(x(2:end)), sats1Unique);
    [~, letterOrder] = ismember(letters, sysNames);
    [~, sortIdx] = sortrows([letterOrder(:), numbers(:)], [-1, -2]);
    sats1Unique = sats1Unique(sortIdx);

    % 将卫星编号映射为数字索引
    [~, sats1Num] = ismember(sats1, sats1Unique);
    sats1Num = sats1Num(:);

    % 周跳分组（分类: 半周跳、显著周跳、严重周跳）
    slip1tmp = zeros(size(slip1));
    for i = 1:length(slip1)
        absSlip = abs(slip1(i));
        if absSlip == 0.5
            slip1tmp(i) = 1;
        elseif absSlip >= 1 && absSlip <= 3
            slip1tmp(i) = 2;
        elseif absSlip > 3
            slip1tmp(i) = 3;
        end
    end

    % 筛选有周跳的数据点
    validIdx = slip1tmp ~= 0;
    time1new = time1(validIdx);
    sats1Numnew = sats1Num(validIdx);
    slip1new = slip1tmp(validIdx);

    % 相位跟踪图绘制
    fig = plotCSSubFunc({time1}, {sats1Num}, [], [], [], ...
        [min(sats1Num)-1, max(sats1Num)+1], ...
        "Time (s)", [], strjoin(frqNames(f+1, :), '/'), ...
        1, [], 3, sats1Unique, ...
        [time1new, sats1Numnew, slip1new]);

    % 保存相位图
    figName = fullfile(filePath, sprintf("PhaseLock_%s.bmp", strjoin(frqNames(f+1, :), '_')));
    saveas(fig, figName);

    % 半周跳 SNR 分布图
    snrDis = zeros(8, length(sysNames));  % 每个系统在不同 SNR 区间的数量
    ledName = cell(1, length(sysNames));  % 用于图例

    for s = 1:length(sysNames)
        % 筛选当前系统的半周跳 SNR
        idx_halfslip = abs(slip1) == 0.5;
        snrs2 = snrs1(idx_halfslip);
        sats2 = sats1(idx_halfslip);
        snrs2 = snrs2(startsWith(sats2, sysNames{s}));

        % 分段统计
        snrDis(:, s) = [
            sum(snrs2 <= 20);
            sum(snrs2 > 20 & snrs2 <= 30);
            sum(snrs2 > 30 & snrs2 <= 33);
            sum(snrs2 > 33 & snrs2 <= 36);
            sum(snrs2 > 36 & snrs2 <= 39);
            sum(snrs2 > 39 & snrs2 <= 42);
            sum(snrs2 > 42 & snrs2 <= 45);
            sum(snrs2 > 45);
        ];

        ledName{s} = frqNames(f+1, s);
    end

    % 绘制柱状图
    plotBars(snrDis, "SNR (dB-Hz)", "HCS number", ...
        {'<20', '20-30', '30-33', '33-36', '36-39', '39-42', '42-45', '>45'}, ...
        string(ledName), 3);

    % 保存 SNR 分布图
    figName = fullfile(filePath, sprintf("HCSSNR_%s.bmp", strjoin(frqNames(f+1, :), '_')));
    saveas(gcf, figName);
end

% 关闭所有图窗
close all;

end