%% GNSS 观测文件批量重命名工具
% 2025-04-13, edit by Lying

renameFiles("I:\01Prog\04CDTH\02Data\202406@CDTH@CropperGNSS\20241203@TH1100@BSDVRS@UGV@Trees\raw");

function renameFiles(folderPath)
    if ~isfolder(folderPath)
        error('路径无效：%s', folderPath);
    end

    files = dir(fullfile(folderPath, '*'));
    files = files(~[files.isdir]);  % 排除文件夹

    timeMap = containers.Map();  % key: 日期，value: 文件索引数组

    % 提取所有文件中的时间字段
    for i = 1:length(files)
        fname = files(i).name;
        % 匹配8位数字日期，比如20241220
        tokens = regexp(fname, '\d{8}', 'match');
        if ~isempty(tokens)
            dateStr = tokens{1};
            if isKey(timeMap, dateStr)
                timeMap(dateStr) = [timeMap(dateStr), i];
            else
                timeMap(dateStr) = i;
            end
        end
    end

    keysList = keys(timeMap);
    for k = 1:length(keysList)
        dateStr = keysList{k};
        fileIdxList = timeMap(dateStr);
        for j = 1:length(fileIdxList)
            idx = fileIdxList(j);
            oldName = files(idx).name;
            [~, ~, ext] = fileparts(oldName);

            seq = sprintf('%02d', j);  % 序号补零
            newName = sprintf('ROVE_%s_%s_TH1100_01.rtcm3', dateStr, seq);
            %newName = sprintf('ROVE_%s_%s_THAM03_01.rtcm3', dateStr, seq);

            oldFull = fullfile(folderPath, oldName);
            newFull = fullfile(folderPath, newName);

            % 重命名
            movefile(oldFull, newFull);
            fprintf('重命名: %s -> %s\n', oldName, newName);
        end
    end
end