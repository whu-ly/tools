%% GNSS 配置文件批量生成工具
% 2025-04-01, edit by Lying
% 2025-07-01，加速大文件起止时间读取

clear all;

% 配置参数
filePath = "I:\01Prog\04CDTH\02Data\202406@CDTH@CropperGNSS\202410@CDTH@CropperGNSS@TH1100\20241016@TH1100@BSDVRS@UGV@TwoWalls";

% 遍历所有观测数据文件,文件后缀为xxo或xxO
[baseFiles, roveFiles, naviFiles] = getXXOFiles(filePath);

% 获取起止时间
baseTime=[];
for i=1:length(baseFiles)
    [rStWeek,rStSecs,rEtWeek,rEtSecs] = getSEGPSTimeFromFile(baseFiles{i});
    baseTime=[baseTime;rStWeek,rStSecs,rEtWeek,rEtSecs];
end
roveTime=[];
for i=1:length(roveFiles)
    [rStWeek,rStSecs,rEtWeek,rEtSecs] = getSEGPSTimeFromFile(roveFiles{i});
    roveTime=[roveTime;rStWeek,rStSecs,rEtWeek,rEtSecs];
end

% 匹配基准站文件
bestBaseIndex = findBestOverlap(baseTime, roveTime); % 每个流动站文件对应的最佳基站文件索引

% 生成配置文件(可拓展)
optFileContentName = [
    "GNSS_FilePath = "
    "GNSS_RoveFile = ";
    "GNSS_BaseFile = ";
    "GNSS_NaviFile = ";
    "GNSS_RefFile = ";
    "GNSS_RefPoss = ";
    "GNSS_ExcludedPrn = "
    "GNSS_DeviceType = "];

for i=1:length(roveFiles)
    if bestBaseIndex(i) == 0
        continue;
    end

    % 打开配置文件
    [folderPath, fileName, ~] = fileparts(roveFiles{i}); 
    outputFilePath = fullfile(folderPath, fileName + ".opt");
    fid = fopen(outputFilePath, 'w');
    if fid == -1
        error("无法创建或打开文件: " + outputFilePath);
        continue;
    end

    % 流动站观测文件
    [~, name, ext] = fileparts(roveFiles{i});
    roveFile = strcat(name, ext);

    % 基准站观测文件
    [~, name, ext] = fileparts(baseFiles{bestBaseIndex(i)});
    baseFile = strcat(name, ext);

    % 星历文件：统一设置或分别设置
    naviFile = [];
    if length(naviFiles) == 1
        naviFile = naviFiles{1};
    elseif length(naviFiles) == length(roveFiles)
        naviFile = naviFiles{i};
    else
        [~, roveFileName, ~] = fileparts(roveFiles{i});
        for j=1:length(naviFiles)
            [~, naviFileName, ~] = fileparts(naviFiles{j});
            if roveFileName == naviFileName
                naviFile = naviFiles{j};
                break;
            end
        end
    end
    if isempty(naviFile)
        fclose(fid);
        if exist(outputFilePath, 'file')
            delete(outputFilePath);
        end
        fprintf('No valid ephemeris file found: %s\n', roveFiles{i});
        continue;
    end
    [~, name, ext] = fileparts(naviFile);
    naviFile = strcat(name, ext);

    % 判断设备类型
    if contains(roveFile, 'TH1100') || contains(roveFile, 'TH1200')
        deviceType = 2;
    else
        deviceType = 0;
    end

    % 写入配置文件
    optFileContentValue = ["";roveFile;baseFile;naviFile;"";"";"C01,C02,C03,C04,C05,C06,C11,C12,C13,C16,C31,C33,C38,C39,C40";deviceType];
    %optFileContentValue = ["";roveFile;baseFile;naviFile;"";"";"C01,C02,C03,C04,C05,C06,C07,C08,C09,C10,C11,C12,C13,C14,C15,C16,C17,C18"];
    %optFileContentValue = ["";roveFile;baseFile;naviFile;"";"";""];
    optFileContent = strcat(optFileContentName, optFileContentValue);
    fprintf(fid, "# POSRTK Software Option Ctrl File, created: 2025/02/10 14:23, version: 1.0\n\n");
    fprintf(fid, "# GNSS后处理参数\n");
    for j = 1:length(optFileContent)
        fprintf(fid, "%s ;\n", optFileContent(j));
    end
    fprintf(fid, "\nGNSS_END\n");

    % 关闭配置文件
    fclose(fid);
end
