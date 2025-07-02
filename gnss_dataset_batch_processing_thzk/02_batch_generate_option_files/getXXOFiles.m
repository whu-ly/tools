function [baseFiles, roveFiles, naviFiles] = getXXOFiles(folderPath)
    % 获取指定目录下所有文件
    files = dir(folderPath);

    % 初始化文件列表
    baseFiles = {};
    roveFiles = {};
    naviFiles = {}; % 存储所有 xxp/xxP 文件

    % 正则表达式匹配文件后缀
    pattern_xxo = '^\d{2}o$'; % 例如 "24o"、"25o"
    pattern_xxp = '^\d{2}[pP]$'; % 例如 "24p"、"25P"

    % 遍历所有文件
    for i = 1:length(files)
        fileName = files(i).name;
        filePath = fullfile(folderPath, fileName);

        % 提取文件名和扩展名
        [name, ext] = strtok(fileName, '.');

        % 移除 `.`，获取纯后缀
        fileExt = ext(2:end);

        % 匹配 "xxo" 文件（BASE 和 ROVE）
        if ~isempty(fileExt) && ~isempty(regexp(fileExt, pattern_xxo, 'once'))
            if startsWith(name, 'BASE', 'IgnoreCase', true)
                baseFiles{end+1} = filePath;
            elseif startsWith(name, 'ROVE', 'IgnoreCase', true)
                roveFiles{end+1} = filePath;
            end
        end

        % 匹配 "xxp" 或 "xxP" 文件（统一存入 naviFiles）
        if ~isempty(fileExt) && ~isempty(regexp(fileExt, pattern_xxp, 'once'))
            naviFiles{end+1} = filePath;
        end
    end
end
