%% GNSS 模糊度固定性能报告自动生成工具
% 2025-07-12, edit by Lying

close all;
clear all;
fclose all;

% 设置目标文件路径、版本号
baseDir = 'I:\01Prog\04CDTH\02Data\202406@CDTH@CropperGNSS\202410@CDTH@CropperGNSS@TH1100';
versionIdentifiers = 'POSRTKDLLV341';

% 其他参数设置
identifiers = {}; % 空表示遍历所有的，第一个@前的标识符和最后一个@后的标识符，如{'20240910', 'OneWall'}
readFileSize = 20; % 仅对文件大小在20kb以上的数据进行绘图和读取

% 生成报告文件路径
markdownPath = fullfile(baseDir, [versionIdentifiers, '.md']);
if exist(markdownPath, 'file')
    delete(markdownPath);
end

% 生成统计表格路径
tablePath = fullfile(baseDir, [versionIdentifiers, '.xlsx']);
if isfile(tablePath)
    delete(tablePath);
end

% 获取文件夹下的所有子文件夹
subfolders = dir(baseDir);
subfolders = subfolders([subfolders.isdir]); % 保留子文件夹

% 遍历并筛选符合条件的子文件夹
for i = 1:length(subfolders)
    folderName = subfolders(i).name;
    
    % 只关注文件夹名包含'@'字符的子文件夹
    if contains(folderName, '@')
        parts = strsplit(folderName, '@');  % 通过'@'分割文件夹名
        
        % 检查第一个和最后一个部分是否与输入标识符匹配
        if isempty(identifiers)||(strcmp(parts{1}, identifiers{1}) && strcmp(parts{end}, identifiers{2}))
            disp(['匹配的文件夹: ', folderName]);
            
            % 构造匹配的文件夹路径
            matchedFolderPath = fullfile(baseDir, folderName);
            
            % 查找该文件夹内的"res"子文件夹
            resFolder = dir(fullfile(matchedFolderPath, 'res'));
            if ~isempty(resFolder) && resFolder(1).isdir
                disp(['找到 "res" 文件夹: ', fullfile(matchedFolderPath, 'res')]);
                
                % 在res文件夹下查找包含 POSRTKDLLV313Debug_ROVE 的子文件夹
                resFolderPath = fullfile(matchedFolderPath, 'res');
                %roveFolders = dir(fullfile(resFolderPath, 'POSRTKDLLV313'));
                subFolders = dir(resFolderPath);  % 获取res文件夹下的所有子文件夹
                subFolders = subFolders([subFolders.isdir]);  % 只保留文件夹（不包括文件）
                roveFolders = subFolders;
                
                % 创建目标文件夹 "POSRTKDLLV313DebugPOS"，如果已存在则删除
                PosFoldername=['A', versionIdentifiers, 'POS'];
                newFolder = fullfile(resFolderPath, PosFoldername);
                if exist(newFolder, 'dir')
                    rmdir(newFolder, 's');  % 删除已有的文件夹及其内容
                end
                mkdir(newFolder);  % 创建新的文件夹
                
                % 遍历每个找到的POSRTKDLLV313Debug_ROVE文件夹
                for j = 1:length(roveFolders)
                    if roveFolders(j).isdir
                        roveFolderPath = fullfile(resFolderPath, roveFolders(j).name);
                        roveFolderName=roveFolders(j).name;
                        % 判断文件夹名是否包含特定标识符
                        % if ~contains(roveFolderName, versionIdentifiers)  % '标识符' 替换成你需要的关键字
                        %     continue;
                        % end 
                        if ~startsWith(roveFolderName, [versionIdentifiers '_ROVE'])
                            continue;
                        end 
                        % 查找 POSRTK.pos 文件
                        posFilePath = fullfile(roveFolderPath, 'POSRTK.pos');
                        if exist(posFilePath, 'file')
                            fileInfo = dir(posFilePath);
                            % 判断文件大小
                            if fileInfo.bytes < readFileSize * 1024
                                disp(['文件过小，跳过: ', posFilePath]);
                                continue;
                            end
                            % 复制文件并重命名
                            newFileName = [roveFolders(j).name, '@POSRTK.pos'];
                            newFilePath = fullfile(newFolder, newFileName);
                            copyfile(posFilePath, newFilePath);
                            disp(['复制文件到: ', newFilePath]);
                        else
                            disp(['未找到 POSRTK.pos 文件在文件夹: ', roveFolderPath]);
                        end
                    end
                end
                files = dir(newFolder);
                validFiles = files(~ismember({files.name}, {'.', '..'}));
                if isempty(validFiles) % 删除空文件夹
                    rmdir(newFolder);
                    fprintf('已删除空文件夹：%s\n', newFolder);
                else
                    plotPos(newFolder, versionIdentifiers, markdownPath, tablePath)
                end
            else
                disp('未找到 "res" 文件夹');
            end
        end
    end
end
