function [dataAll] = getAzelFromSatData(satData, index, excSat) 
% getAzelFromSatData 提取卫星号、方位角、高度角及多频观测数据
%
% 输入参数:
%   satData - 卫星数据结构数组，其中每个元素包含以下字段:
%             - satName: 卫星名称 (字符型，例如 'G01', 'R05')
%             - pos: 卫星方位角和高度角的矩阵，按列存储 [azimuth, elevation]
%             - freqData: 频率数据 (结构数组，包含多频观测值)
%   index   - 指定列索引，用于提取每个频率的观测数据
%
% 输出参数:
%   dataAll - 一个 cell 数组，每个单元存储一个卫星的数据，格式为:
%             {satName, azimuth, elevation, freq1, freq2, freq3}
%             其中：
%             - satName: 卫星名称 (字符型)
%             - azimuth: 方位角 (列向量)
%             - elevation: 高度角 (列向量)
%             - freq1, freq2, freq3: 观测数据，对应 freqData 结构中的指定列
%
% 说明:
%   - 本函数遍历所有卫星数据，并提取三频观测值
%   - 结果按 cell 数组存储，适用于后续数据分析

    % 定义所需的频率索引
    frq = [1, 2, 3]; 

    % 初始化存储所有卫星数据的 cell 数组
    dataAll = {};

    % 遍历所有卫星数据，提取相关信息
    for i = 1:size(satData, 2) 
        satName = char(satData(i).satName); % 获取卫星名称
        if isempty(satName)
            continue; % 跳过无效的卫星数据
        end

        %剔除不符合要求的卫星
        if ~isempty(excSat)
            ExcludeFlag=false;
            for kk=1:length(excSat)
                if satData(i).satName==excSat(kk)
                    ExcludeFlag=true;
                    break;
                end
            end
            if ExcludeFlag==true
                continue
            end
        end

        % 提取方位角、高度角及三频观测值
        dataRow = {satData(i).satName, ...
                   satData(i).pos(:,1), ... % 方位角
                   satData(i).pos(:,2), ... % 高度角
                   satData(i).freqData(frq(1)).data(:, index), ... % 频率1数据
                   satData(i).freqData(frq(2)).data(:, index), ... % 频率2数据
                   satData(i).freqData(frq(3)).data(:, index)};    % 频率3数据
        dataAll{size(dataAll,2)+1} = dataRow;
    end
end
