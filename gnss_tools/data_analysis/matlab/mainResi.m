%% 功能：导入数据(.Residual)，分析数据质量(最多支持三频) 
% 2025/01/12, Liu Ying, modify.
% 2025/03/01, Liu Ying, 添加绘制卫星天空图
% 2025/04/10, Liu Ying, 添加绘制序列图(随信噪比变化)
% 2025/04/10, Liu Ying, 添加绘制相位跟踪及周跳情况

clear;
close all;

% 文件路径
fileName = "E:\01Prog\04CDTH\02Data\01TestSet\@TH1100@BSDVRS@UGV@OneWall\resi\IPSDLLV310T7_ROVE_20241127_02_TH1100_01\IPS_SV1.Residual";

% 配置参数
optPlotIndex       = [1, 1, 1, 0, 0, 0, 0];                % 选择：CN0, Code DD, Phase DD, Phase TD, Code/Phase/Doppler一致性
optIndexUnit       = [" (dB-Hz)", " (m)", " (cycle)", " (cycle)", " (m)", " (m)", " (m)"];
optXLabel          = ["Time (s)"];
optYLabel          = ["SNR", "CodeDDNoise", "PhaseDDNoise", "PhaseTDNoise", "CodeCons", "PhaseCons", "DopCons"];
optYLim            = [20, 60; -50, 50; 0.0, 0.5; 0.0, 0.5; -5, 5; -1, 1; -10, 10]; % Y轴范围
optStartTime       = [0]; 
optEndTime         = [0];  % 截取时间段
optIncludeSat      = []; % 仅包含某颗卫星, 和optSeriesSepSat搭配使用，如["G01"]
optExcludeSat      = ["C01";"C02";"C03";"C04";"C05";"C06"]; % 剔除指定卫星，如["G01";"C01"]
%optExcludeSat      = ["C01";"C02";"C03";"C04";"C05";"C06";"C07";"C08";"C09";"C10";"C11";"C12";"C13";"C14";"C15";"C16"]; % 剔除指定卫星，如["G01"]
optSysNames        = ["G", "R", "C", "C", "E", "J"];  % GRCCEJ
optSysNamesAbbr    = ["GPS", "GLO", "BD2", "BD3", "GAL", "QZS"];  % GRCCEJ
optFrqNames        = ["L1", "L2", "L5"; "", "", ""; "B1I", "B2I", "B3I"; "B1I", "B1C", "B2a"; "E1", "E5b", "E5a"; "", "", ""]; % GRCCEJ
optReadCommonSat   = true; % true 只读取共视卫星

% 绘图配置
optPlotSeries      = 1; % 是否绘制序列图，横轴为时间，分系统分频点绘制
optSeriesSepSat    = 1; % 绘制时间序列时是否区分卫星

optPlotSeriesSNR   = 1; % 是否绘制序列图，横轴为信噪比，分系统分频点绘制

optPlotQuartile    = 1; % 是否绘制四分位图(箱型图)，反映指标分布范围，所有频点绘制在一张图上

optPlotSky         = 1; % 是否绘制卫星天空视图，默认其颜色随载噪比变化，分系统绘制

optPlotCS          = 1; % 绘制相位跟踪及周跳情况，分频点绘制

% 添加包含目录
addpath(genpath(fullfile(pwd, 'subfunctions')));

% 获取文件目录路径
[filePath, ~, ~] = fileparts(fileName);  % 获取文件的目录路径

% 定义存储路径和文件名
satDataFile = fullfile(filePath, 'satData.mat');

% 检查文件是否存在，若不存在则读取并保存数据
if exist(satDataFile, 'file')
    load(satDataFile, 'satData');  % 加载已保存的数据
else
    satData = getDataFromFile(fileName, optStartTime, optEndTime, optExcludeSat, optReadCommonSat);
    save(satDataFile, 'satData');  % 保存数据
end

%% 分频点绘制序列图(随时间变化)，不区分卫星
for i = 1:size(optPlotIndex, 2)
    if optSeriesSepSat == 1 || optPlotSeries == 0
        break;
    end

    if optPlotIndex(i) ~= 1
        continue;
    end
    
    errorStatsTable = []; % 存储统计数据
    rowName = []; % 表格行名
    [dataAll] = getDataFromSatData(satData, [], i, optSeriesSepSat, 0); % 分系统提取
    yLabelStr = strcat(optYLabel(i), optIndexUnit(i)); % Y轴标签

    for sysIdx = 1:length(dataAll)  % 遍历每个系统
        if isempty(dataAll{sysIdx})
            continue;
        end

        % 遍历每个频率
        nFrq = size(dataAll{sysIdx},2) / 2;
        for frqIdx = 1:nFrq
            xData = dataAll{sysIdx}(:, 2 * frqIdx - 1);
            yData = dataAll{sysIdx}(:, 2 * frqIdx);
            if i == 2
                idx = abs(yData)<=50;
                xData = xData(idx);
                yData = yData(idx);
            end
            
            if all(isnan(yData))  % 若数据全为NaN，跳过
                continue;
            end

            rowName = [rowName; strcat(optSysNames(sysIdx), "_", optFrqNames(sysIdx, frqIdx))];
            
            % 绘制数据并保存为图片
            fig = plotSeries({xData}, {yData}, [], optFrqNames(sysIdx, frqIdx), [], [optYLim(i,1), optYLim(i,2)], optXLabel, yLabelStr, 1, [], 3);
            figName = strcat('Series_', optYLabel(i), '_', optSysNames(sysIdx), '_', optFrqNames(sysIdx, frqIdx), '.bmp');
            saveas(fig, fullfile(filePath, figName));  % 保存图片

            % 计算统计量
            errorStatsTable = [errorStatsTable; mean(abs(yData(~isnan(yData)))), rms(yData(~isnan(yData))), prctile(yData(~isnan(yData)), 95), std(yData(~isnan(yData)))];
        end
    end
    
    % 将统计结果保存为表格
    colName = [strcat("Mean", optIndexUnit(i)), strcat("Rms", optIndexUnit(i)), strcat("C95", optIndexUnit(i)), strcat("Std", optIndexUnit(i))];
    writetableM(errorStatsTable,3,rowName,colName,filePath,strcat('Table_',optYLabel(i)))
end

%% 分频点绘制序列图(随信噪比变化)，不区分卫星
for i = 2:size(optPlotIndex, 2)
    if optPlotSeriesSNR == 0
        break;
    end

    if optPlotIndex(i) ~= 1
        continue;
    end
    
    errorStatsTable = []; % 存储统计数据
    rowName = []; % 表格行名
    [dataAll] = getDataFromSatData(satData, 1, i, 0, 0); % 分系统提取
    yLabelStr = strcat(optYLabel(i), optIndexUnit(i)); % Y轴标签

    for sysIdx = 1:length(dataAll)  % 遍历每个系统
        if isempty(dataAll{sysIdx})
            continue;
        end

        % 遍历每个频率
        nFrq = size(dataAll{sysIdx},2) / 2;
        for frqIdx = 1:nFrq
            xData = dataAll{sysIdx}(:, 2 * frqIdx - 1);
            yData = dataAll{sysIdx}(:, 2 * frqIdx);
            if i == 2
                idx = abs(yData)<=50;
                xData = xData(idx);
                yData = yData(idx);
            end
            
            if all(isnan(yData))  % 若数据全为NaN，跳过
                continue;
            end

            rowName = [rowName; strcat(optSysNames(sysIdx), "_", optFrqNames(sysIdx, frqIdx))];
            
            % 绘制数据并保存为图片
            fig = plotSeries({xData}, {yData}, [], optFrqNames(sysIdx, frqIdx), [], [optYLim(i,1), optYLim(i,2)], "SNR (dB-Hz)", yLabelStr, 1, [], 3);
            figName = strcat('Series(SNR)_', optYLabel(i), '_', optSysNames(sysIdx), '_', optFrqNames(sysIdx, frqIdx), '.bmp');
            saveas(fig, fullfile(filePath, figName));  % 保存图片
        end
    end
end

%% 分频点绘制时间序列图，区分不同卫星
for i = 1:size(optPlotIndex, 2)
    if optSeriesSepSat == 0 || optPlotSeries == 0
        break;
    end

    if optPlotIndex(i) ~= 1
        continue;
    end
    
    [dataAll] = getDataFromSatData(satData, [], i, optSeriesSepSat, 0);  % 分系统提取
    yLabelStr = strcat(optYLabel(i), optIndexUnit(i));  % Y轴标签

    for sysIdx = 1:length(dataAll) % 遍历系统
        if isempty(dataAll{sysIdx})
            continue;
        end

        % 遍历每个频率
        for frqIdx = 1:3
            time = {}; data = {}; name = {};
            for satIdx = 1:length(dataAll{sysIdx}) % 遍历卫星
                xData = dataAll{sysIdx}{satIdx}{2}(:, 2 * frqIdx - 1);
                yData = dataAll{sysIdx}{satIdx}{2}(:, 2 * frqIdx);
                if all(isnan(xData)) || all(isnan(yData))
                    continue;
                end

                if ~isempty(optIncludeSat)&&~any(ismember(dataAll{sysIdx}{satIdx}{1}, optIncludeSat))
                   continue; 
                end

                time{end+1} = xData;
                data{end+1} = yData;
                name{end+1} = dataAll{sysIdx}{satIdx}{1};
            end
            if isempty(time) || isempty(data)
                continue
            end
            fig = plotSeries(time, data, [], name, [], [optYLim(i,1),optYLim(i,2)], optXLabel, yLabelStr, 1, [], 3);
            figName = strcat('SeriesSepSat_',optYLabel(i), '_', optSysNames(sysIdx), '_', optFrqNames(sysIdx, frqIdx), '.bmp');
            saveas(fig, fullfile(filePath, figName));  % 保存图片
        end
    end
    %close all;
end

%% 绘制四分位图，各频点占一个箱体
for i = 1:size(optPlotIndex, 2)
    if optPlotQuartile == 0
        break;
    end

    if optPlotIndex(i) ~= 1
        continue;
    end
    
    [dataAll] = getDataFromSatData(satData, [], i, 0, 0);  % 分系统提取

    data = {}; boxName = {};
    for sysIdx = 1:length(dataAll)  % 遍历每个系统
        if isempty(dataAll{sysIdx})
            continue;
        end

        % 遍历每个频率
        nFrq = size(dataAll{sysIdx},2) / 2;
        for frqIdx = 1:nFrq
            yData = dataAll{sysIdx}(:, 2 * frqIdx);
            if i == 2
                yData = yData(abs(yData)<=50);
            end
            if all(isnan(yData))  % 若数据全为NaN，跳过
                continue;
            end
            data{end+1}=abs(yData);
            boxName{end+1}=optFrqNames(sysIdx,frqIdx);
        end
    end
    % 绘制数据并保存为图片
    yLabelStr = strcat(optYLabel(i), optIndexUnit(i));  % Y轴标签
    fig = plotQuartile({data}, [], [0,optYLim(i,2)], "GNSS frequency", yLabelStr, boxName, [], 3);
    figName = strcat('BoxFig_',optYLabel(i), '.bmp');
    saveas(fig, fullfile(filePath, figName));  % 保存图片
end
% close all;  % 关闭所有图形窗口

%% 分频段绘制卫星天空视图，其颜色随载噪比变化
if optPlotSky == 1
    [dataAll] = getAzelFromSatData(satData, 1, optExcludeSat); % 与系统无关
    titles = {"L1/E1/B1C","L2/E5b/B1I","L5/E5a/B2a"};
    for f=1:3
        fig=figure;
        Init = 1;
        for i=1:size(dataAll,2)
            Init = plotSkyMap(fig,Init,dataAll{i}{2},dataAll{i}{3},abs(dataAll{i}{3+f}),dataAll{i}{1},titles{f},0);
        end
        hold off;
        title_str = regexprep(titles{f}, '[\/:*?"<>|]', '_'); % 替换非法字符
        figName = sprintf("Skymap_%s.bmp",title_str);
        saveas(fig,fullfile(filePath, figName));
    end
end


% 绘制相位跟踪及周跳情况(大、小、半周跳)
if optPlotCS == 1
    fileName = "CSEstimation.txt";
    plotCS(fullfile(filePath, fileName));
end

close all;
