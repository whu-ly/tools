function plotPos(folderPath, keyword, markdownPath, tablePath)
    % 绘制 EN 固定解轨迹图并保存
    % 输入:
    % - folderPath: 包含 .pos 文件的文件夹路径
    % - keyword: 用于保存图像的标识符
    % - markdownPath: 用于保存图像路径的 Markdown 文件路径
    % - tablePath: 用于保存统计结果

    % 检查输入参数
    if nargin < 1
        folderPath = uigetdir([], '选择包含 .pos 文件的文件夹');
        if folderPath == 0
            disp('未选择文件夹，程序退出。');
            return;
        end
    end

    % 获取文件夹中所有 .pos 文件
    posFiles = dir(fullfile(folderPath, '*.pos'));
    if isempty(posFiles)
        disp('未找到 .pos 文件。');
        return;
    end

    % 初始化数据容器
    totalGNGGA = 0;  % 总 $GNGGA 行数
    totalFix = 0;   % 解状态为 4 的行数
    lat0 = 0;
    lon0 = 0;
    alt0 = 0;

    % 遍历所有.pos 文件并处理
    % 加载常量文件（如字体设置等）
    run('constant.m');
    %fontSize = fontSize_645;
    gcaPos = gcaPos_645H;
    gcfPos = gcfPos_645H;

    % 创建图形
    figure1 = figure;
    set(figure1, 'Color', 'w');  % 设置背景色为白色
    set(gcf, 'Visible', 'off'); % 隐藏图形窗口
    hold on;
    convergenceTimes = [];
    for i = 1:numel(posFiles)
        filePath = fullfile(posFiles(i).folder, posFiles(i).name);
        [lats, lons, alts, nGNGGA, nFix, convergenceTime] = convertGGA(filePath);
        convergenceTimes = [convergenceTimes; convergenceTime];
        % 累加数据
        totalGNGGA = totalGNGGA + nGNGGA;
        totalFix = totalFix + nFix;
        if isempty(lats)
            continue;
        end
        if lat0 == 0
            lat0 = mean(lats);
            lon0 = mean(lons);
            alt0 = mean(alts);
        end
        [east, north, ~] = blh2enu(lat0, lon0, alt0, lats, lons, alts);
        scatter(east, north, 20, [0, 0.5, 0], 'filled'); % 绘制轨迹
    end
    hold off;

    % 设置坐标轴标签
    xlabel("E (m)");
    ylabel("N (m)");

    % 设置坐标轴边框，并调整边框的线宽
    set(gca, 'Box', 'on', 'LineWidth', 2);

    % 设置图形的尺寸与坐标轴位置
    set(gca, 'Position', gcaPos);  % 设置坐标轴位置
    set(gcf, 'Units', 'centimeters', 'Position', gcfPos);  % 设置图形的单位与位置

    % 设置图形的字体格式（应在最后设置，以保证不被覆盖）
    set(gca, 'FontSize', fontSize, 'FontName', fontName, 'LineWidth', 1.0);

    % 手动绘制50cm刻度线并让 xLim 和 yLim 的区间一样长
    xLim = get(gca, 'XLim');
    yLim = get(gca, 'YLim');
    
    % 将 xLim 和 yLim 调整到 0.5 的倍数
    xLim(1) = floor(xLim(1) / 0.5) * 0.5;
    xLim(2) = ceil(xLim(2) / 0.5) * 0.5;
    yLim(1) = floor(yLim(1) / 0.5) * 0.5;
    yLim(2) = ceil(yLim(2) / 0.5) * 0.5;
    
    % 计算区间长度
    xRange = xLim(2) - xLim(1);
    yRange = yLim(2) - yLim(1);
    
    % 找到较大的区间长度
    maxRange = max(xRange, yRange);
    
    % 调整 xLim 和 yLim，使其区间长度相等
    if xRange < maxRange
        diff = (maxRange - xRange) / 2;
        xLim = xLim + [-diff, diff];
    elseif yRange < maxRange
        diff = (maxRange - yRange) / 2;
        yLim = yLim + [-diff, diff];
    end
    
    % 重新调整到 0.5 的倍数
    xLim(1) = floor(xLim(1) / 0.5) * 0.5;
    xLim(2) = ceil(xLim(2) / 0.5) * 0.5;
    yLim(1) = floor(yLim(1) / 0.5) * 0.5;
    yLim(2) = ceil(yLim(2) / 0.5) * 0.5;
    
    % 绘制网格线
    gridInterval = 50;  % 50 cm
    xGridCount = floor((xLim(2) - xLim(1)) * 100 / gridInterval);
    yGridCount = floor((yLim(2) - yLim(1)) * 100 / gridInterval);
    
    % 绘制 X 轴方向的网格线
    for i = 0:xGridCount
        xPos = xLim(1) + i * gridInterval / 100;  % 转换为实际坐标单位
        line([xPos, xPos], yLim, 'Color', 'k', 'LineStyle', ':');  % 竖直虚线
    end
    
    % 绘制 Y 轴方向的网格线
    for j = 0:yGridCount
        yPos = yLim(1) + j * gridInterval / 100;  % 转换为实际坐标单位
        line(xLim, [yPos, yPos], 'Color', 'k', 'LineStyle', ':');  % 水平虚线
    end
    
    % 设置调整后的坐标范围
    set(gca, 'XLim', xLim);
    set(gca, 'YLim', yLim);

    % 手动设置统计信息的位置
    text(0.02, 1.00, ...
        sprintf('单位网格: 50cm\n固定率: %.2f%%\n历元数: %d\n时段数: %d', (totalFix / totalGNGGA) * 100, totalGNGGA, numel(posFiles)), ...
        'FontSize', fontSize, 'FontName', 'fontName', 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', ...
        'Units', 'normalized', ... % 使用归一化坐标
        'BackgroundColor', 'none', 'EdgeColor', 'none');

    % 保存图像
    figPath = fullfile(folderPath, 'EN固定解轨迹图.bmp');
    saveas(figure1, figPath);  % 保存图片
    saveImageToMarkdown(figure1, figPath, markdownPath, keyword)

    % 输出统计结果
    [parentPath, ~] = fileparts(folderPath);
    [parentPath, ~] = fileparts(parentPath); % 文件名
    [~, sFILE] = fileparts(parentPath); % 文件名
    sPOS_NUM = numel(posFiles); % pos文件数量
    sEPOCH_NUM = totalGNGGA; % 历元数量
    convergenceTimes = sort(convergenceTimes); % 收敛时间
    sCT_25 = quantile(convergenceTimes, 0.25);
    sCT_50 = quantile(convergenceTimes, 0.50);
    sCT_75 = quantile(convergenceTimes, 0.75);
    sCT_MAX = quantile(convergenceTimes, 1.00);
    sFIX = (totalFix / totalGNGGA); % 总固定率
    colNames = {'FILE', 'POS_NUM', 'EPOCH_NUM', 'CT_25_s', 'CT_50_s', 'CT_75_s', 'CT_MAX_s', 'FIX'};
    dataRow = {sFILE, sPOS_NUM, sEPOCH_NUM, sprintf('%.1f', sCT_25), sprintf('%.1f', sCT_50), sprintf('%.1f', sCT_75), sprintf('%.1f', sCT_MAX), sprintf('%.3f', sFIX)};
    appendRowToTable(tablePath, dataRow, colNames);
end

function [east, north, up] = blh2enu(lat0, lon0, alt0, lats, lons, alts)
    % 原点 BLH 转 ECEF
    [x0, y0, z0] = blh2ecef(lat0, lon0, alt0);

    % BLH 转 ECEF
    [x, y, z] = blh2ecef(lats, lons, alts);

    % 计算相对位置
    dX = x - x0;
    dY = y - y0;
    dZ = z - z0;

    % 转换矩阵
    sinLat = sin(deg2rad(lat0));
    cosLat = cos(deg2rad(lat0));
    sinLon = sin(deg2rad(lon0));
    cosLon = cos(deg2rad(lon0));

    t = [-sinLon, cosLon, 0;
         -sinLat * cosLon, -sinLat * sinLon, cosLat;
          cosLat * cosLon, cosLat * sinLon, sinLat];

    enu = t * [dX'; dY'; dZ'];
    east = enu(1, :)';
    north = enu(2, :)';
    up = enu(3, :)';
end

function [x, y, z] = blh2ecef(lat, lon, h)
    % BLH 转换为 ECEF
    a = 6378137.0; % 长半轴
    e2 = 0.00669437999014; % 偏心率平方

    sinLat = sin(deg2rad(lat));
    cosLat = cos(deg2rad(lat));
    sinLon = sin(deg2rad(lon));
    cosLon = cos(deg2rad(lon));

    N = a ./ sqrt(1 - e2 * sinLat.^2);
    x = (N + h) .* cosLat .* cosLon;
    y = (N + h) .* cosLat .* sinLon;
    z = (N * (1 - e2) + h) .* sinLat;
end

function [lats, lons, alts, nGNGGA, nFix, convergenceTime] = convertGGA(filePath)
    % 解析 .pos 文件，提取固定解数据
    numRows = 36000;
    lats = zeros(numRows, 1);
    lons = zeros(numRows, 1);
    alts = zeros(numRows, 1);
    nGNGGA = 0;
    nFix = 0;

    fid = fopen(filePath, 'r');
    if fid == -1
        error('无法打开文件: %s', filePath);
    end

    year = 0; month = 0; day = 0;
    firstEpochTime = [];
    convergenceTime = 999999;
    while ~feof(fid)
        line = fgetl(fid);
        if contains(line, '$ACCUR') % $ACCUR,1,12,0,20250623,,123206.00,,,,,,,,,,*6C
            fields = strsplit(line, ','); % 用逗号分割整行
            dateStr = fields{5}; % 提取日期字段（第5个）
            year  = str2double(dateStr(1:4)); % 分解为年、月、日
            month = str2double(dateStr(5:6));
            day   = str2double(dateStr(7:8));
        end
        if contains(line, '$GNGGA') % $GNGGA,160920.70,3117.520237,N,12045.591497,E,5,21,1.3,5.365,M,0.000,M,,*49
            if year == 0
                error('未读取到年月日: %s', filePath);
                break;
            end
            nGNGGA = nGNGGA + 1;
            [~,line]=strtok(line,',');
            [strtempt,line]=strtok(line,','); %时间
            hour=str2double(strtempt(1:2));min=str2double(strtempt(3:4));sec=str2double(strtempt(5:9));
            [ gpsTime.week, gpsTime.SecOfWeek ] = YMDHMS2GPST(year, month, day, hour, min, sec);
            if isempty(firstEpochTime)
                firstEpochTime = gpsTime;
            end

            [strtempt,line]=strtok(line,','); %纬度
            lat = str2double(strtempt(1:2)) + str2double(strtempt(3:end))/60.0;
            [~,line]=strtok(line,',');
            [strtempt,line]=strtok(line,','); %经度
            lon = str2double(strtempt(1:3)) + str2double(strtempt(4:end))/60.0;
            [~,line]=strtok(line,',');
            [strtempt,line]=strtok(line,','); %解状态
            sta = str2double(strtempt);
            [~,line]=strtok(line,',');
            [~,line]=strtok(line,',');
            [strtempt,line]=strtok(line,','); %高程
            alt = str2double(strtempt);

            if sta == 4 % 判断固定解状态
                nFix = nFix + 1;
                lats(nFix) = lat;
                lons(nFix) = lon;
                alts(nFix) = alt;
                if convergenceTime == 999999
                    convergenceTime = (gpsTime.week - firstEpochTime.week) * 604800 + gpsTime.SecOfWeek - firstEpochTime.SecOfWeek;
                end
            end
        end
    end

    lats(nFix + 1:end)=[];
    lons(nFix + 1:end)=[];
    alts(nFix + 1:end)=[];

    fclose(fid);
end

function appendRowToTable(filePath, dataRow, colNames)
% filePath: 要保存的表格文件完整路径，如 'E:\data\log.xlsx'
% dataRow: 结构体或 cell 数组，表示要写入的一行数据
% colNames: 列名 cell 数组（仅用于首次写入）

    % 判断文件是否存在
    fileExists = isfile(filePath);
    
    if fileExists
        % 读取已有内容
        T = readtable(filePath);
    else
        % 创建空表并指定列名
        T = cell2table(cell(0, length(colNames)), 'VariableNames', colNames);
    end

    % 将输入的数据行转为表格（确保变量名一致）
    if iscell(dataRow)
        Tnew = cell2table(dataRow, 'VariableNames', colNames);
    elseif isstruct(dataRow)
        Tnew = struct2table(dataRow);
    else
        error('dataRow 应为 cell 数组或 struct。');
    end

    % 拼接并写入表格
    T = [T; Tnew];
    writetable(T, filePath);
end

% 年月日时分秒转换为GPS时，不考虑闰秒
function [gpsWeek, sow] = YMDHMS2GPST(year,month,day,hour,min,sec)
    if(month <= 2)
        year = year - 1;
        month = month + 12;
    end

    UT = hour + min / 60.0 + sec / 3600.0;
    UT = UT / 24.0;

    fracDay = UT - floor(UT);
    days = floor(365.25 * year) + floor(30.6001 * (month + 1)) + day + floor(UT) - 679019;

    gpsWeek = floor(((days + fracDay) - 44244) / 7);
    sow = (days - 44244 - gpsWeek * 7) * 86400 + fracDay * 86400;
end
