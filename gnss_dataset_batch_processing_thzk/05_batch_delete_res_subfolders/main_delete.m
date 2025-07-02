%% # GNSS 数据集中 res 子文件夹批量删除工具
% 2025-07-12, edit by Lying

deleteSubfoldersInRes('I:\01Prog\04CDTH\02Data\202406@CDTH@CropperGNSS\TestSet');

function deleteSubfoldersInRes(rootFolder)
    % 输入：rootFolder - 字符串，目标根目录路径
    if ~isfolder(rootFolder)
        error('指定的路径不是一个有效的文件夹。');
    end

    % 查找所有名为"res"的文件夹（递归查找）
    resFolders = findResFolders(rootFolder);

    % 遍历每一个res文件夹
    for i = 1:length(resFolders)
        resPath = resFolders{i};

        % 获取res文件夹下的所有内容
        contents = dir(resPath);

        % 遍历内容，找出子文件夹并删除（排除 . 和 ..）
        for j = 1:length(contents)
            item = contents(j);
            if item.isdir && ~strcmp(item.name, '.') && ~strcmp(item.name, '..')
                folderToDelete = fullfile(resPath, item.name);
                try
                    rmdir(folderToDelete, 's');  % 删除子文件夹及其所有内容
                    fprintf('已删除子文件夹: %s\n', folderToDelete);
                catch ME
                    warning('无法删除文件夹 %s: %s', folderToDelete, ME.message);
                end
            end
        end
    end
end

function resFolders = findResFolders(rootFolder)
    % 递归查找所有名为 "res" 的文件夹
    resFolders = {};
    foldersToCheck = {rootFolder};

    while ~isempty(foldersToCheck)
        currentFolder = foldersToCheck{1};
        foldersToCheck(1) = [];

        files = dir(currentFolder);
        for k = 1:length(files)
            f = files(k);
            if f.isdir && ~strcmp(f.name, '.') && ~strcmp(f.name, '..')
                subfolderPath = fullfile(currentFolder, f.name);
                % 如果文件夹名是 "res"，加入结果
                if strcmp(f.name, 'res')
                    resFolders{end+1} = subfolderPath; %#ok<AGROW>
                else
                    foldersToCheck{end+1} = subfolderPath; %#ok<AGROW>
                end
            end
        end
    end
end
