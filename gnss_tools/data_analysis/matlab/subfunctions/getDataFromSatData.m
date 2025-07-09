function [dataAll] = getDataFromSatData(satData, xIndex, yIndex, sepBySat, sepBD2)
% getDataFromSatData 提取三频数据并根据卫星系统分类
%
% 输入参数:
%   satData - 卫星数据结构数组，其中每个元素包含以下字段:
%             - satName: 卫星名称 (字符型，例如 'G01', 'R05')
%             - time: 时间戳 (数值型)
%             - freqData: 频率数据 (结构数组，包含多频数据)
%   index   - 指定列索引，用于提取每个频率的数据
%   sepBySat- 1 = 区分卫星号 (每个卫星数据单独存放)
%             0 = 合并同一卫星系统的所有数据
%   sepBD2  - 1 = 区分BD2和BD3 
%
% 输出参数:
%   dataAll - 一个 cell 数组，包含六种卫星系统的数据:
%             {dataGPS, dataGLO, dataBD2, dataBD3, dataGAL, dataQZS}
%             - sepBySat = 1: 每个系统的数据是 cell 数组，每颗卫星独立存储
%             - sepBySat = 0: 每个系统的数据是数值矩阵，所有卫星合并存放
%

    % 定义所需的频率索引
    frq = [1, 2, 3];

    % 初始化数据存储
    if sepBySat == 1
        dataGPS = {}; dataGLO = {}; dataBD2 = {}; dataBD3 = {}; dataGAL = {}; dataQZS = {};
    else
        dataGPS = []; dataGLO = []; dataBD2 = []; dataBD3 = []; dataGAL = []; dataQZS = [];
    end

    % 遍历所有卫星数据，按系统分类
    for i = 1:size(satData, 2) 
        satName = char(satData(i).satName); % 获取卫星名称
        if isempty(satName)
            continue; % 跳过无效数据
        end

        % 构造数据行
        dataRow = [];
        for j = 1:length(frq)
            if isempty(xIndex)
                dataRow = [dataRow, satData(i).time, satData(i).freqData(frq(j)).data(:, yIndex)];
            else
                dataRow = [dataRow, satData(i).freqData(frq(j)).data(:, xIndex), satData(i).freqData(frq(j)).data(:, yIndex)];
            end
        end
        if sepBySat == 1
            dataRow = {satName, dataRow};
        end

        % 按卫星系统存储
        switch satName(1)
            case 'G' % GPS
                if sepBySat == 1
                    dataGPS{end+1} = dataRow;
                else
                    dataGPS = [dataGPS; dataRow];
                end
            case 'R' % GLONASS
                if sepBySat == 1
                    dataGLO{end+1} = dataRow;
                else
                    dataGLO = [dataGLO; dataRow];
                end
            case 'C' % BDS (区分 BDS-2 和 BDS-3)
                satNum = str2double(satName(2:end));
                if satNum <= 19 && sepBD2 == 1
                    if sepBySat == 1
                        dataBD2{end+1} = dataRow;
                    else
                        dataBD2 = [dataBD2; dataRow];
                    end
                else
                    if sepBySat == 1
                        dataBD3{end+1} = dataRow;
                    else
                        dataBD3 = [dataBD3; dataRow];
                    end
                end
            case 'E' % Galileo
                if sepBySat == 1
                    dataGAL{end+1} = dataRow;
                else
                    dataGAL = [dataGAL; dataRow];
                end
            case 'J' % QZSS
                if sepBySat == 1
                    dataQZS{end+1} = dataRow;
                else
                    dataQZS = [dataQZS; dataRow];
                end
        end
    end

    % 输出数据
    dataAll = {dataGPS, dataGLO, dataBD2, dataBD3, dataGAL, dataQZS};
end
