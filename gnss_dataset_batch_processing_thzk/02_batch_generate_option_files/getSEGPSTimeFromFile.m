function [gpsWeekStart, sowStart, gpsWeekEnd, sowEnd] = getSEGPSTimeFromFile(filename)
    % 读取文件内容
    fid = fopen(filename, 'r');
    if fid == -1
        error('无法打开文件: %s', filename);
    end
    
    % 读取起始时间（从文件开头）
    firstTime = [];
    while ~feof(fid)
        line = fgetl(fid);
        if startsWith(line, '>')
            % 解析时间信息
            timeParts = sscanf(line(2:end), '%d %d %d %d %d %f');
            year = timeParts(1);
            month = timeParts(2);
            day = timeParts(3);
            hour = timeParts(4);
            minute = timeParts(5);
            second = timeParts(6);
            % 转换为 GPS 时间
            [gpsWeek, sow] = YMDHMS2GPST(year, month, day, hour, minute, second);
            % 记录起始时间
            firstTime = [gpsWeek, sow];
            break; % 找到起始时间后立即跳出循环
        end
    end
    
    % 读取结束时间（从文件末尾）
    lastTime = [];
    fseek(fid, 0, 'eof'); % 移动到文件末尾
    fileSize = ftell(fid);
    chunkSize = 1024; % 每次读取的块大小（可根据需要调整）
    found = false;
    
    while ~found && fileSize > 0
        % 确定要读取的块大小和位置
        readSize = min(chunkSize, fileSize);
        fseek(fid, -readSize, 'cof'); % 从当前位置向前移动
        
        % 读取块数据
        chunk = fread(fid, readSize, '*char')';
        fseek(fid, -readSize, 'cof'); % 重置位置以便下次读取
        
        % 在块中查找最后一行以'>'开头的行
        lines = strsplit(chunk, '\n');
        for i = numel(lines):-1:1
            line = strtrim(lines{i});
            if startsWith(line, '>')
                timeParts = sscanf(line(2:end), '%d %d %d %d %d %f');
                if length(timeParts) == 8  % 确保读取到完整的时间信息
                    % 解析时间信息
                    year = timeParts(1);
                    month = timeParts(2);
                    day = timeParts(3);
                    hour = timeParts(4);
                    minute = timeParts(5);
                    second = timeParts(6);
                    % 转换为 GPS 时间
                    [gpsWeek, sow] = YMDHMS2GPST(year, month, day, hour, minute, second);
                    % 记录结束时间
                    lastTime = [gpsWeek, sow];
                    found = true;
                    break;
                end
            end
        end
    end
    
    fclose(fid);
    
    % 检查是否找到时间信息
    if isempty(firstTime) || isempty(lastTime)
        error('文件中未找到有效的时间标记行（以">"开头）');
    end
    
    % 输出 GPS 时间
    gpsWeekStart = firstTime(1);
    sowStart = firstTime(2);
    gpsWeekEnd = lastTime(1);
    sowEnd = lastTime(2);
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