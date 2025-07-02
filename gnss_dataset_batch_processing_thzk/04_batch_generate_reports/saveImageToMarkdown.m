function saveImageToMarkdown(figHandle, imageSavePath, markdownPath, keyword)
    % saveImageToMarkdown - 保存图片并将图片信息插入到 Markdown 文件
    %
    % 参数:
    %   figHandle      - 图像句柄 (使用 gcf 获取当前图像)
    %   imageSavePath  - 保存图片的路径 (包含文件名和扩展名)
    %   markdownPath   - 目标 Markdown 文件路径
    %   keyword        - markdown 标题
    %
    % 示例:
    %   saveImageToMarkdown(gcf, 'output/example_image.png', 'output/record.md', 'Image Records');

    % 检查输入参数
    if nargin < 3
        error('需要提供图像句柄、图片保存路径和 Markdown 文件路径。');
    end
    
    % 获取图片保存目录
    [saveDir, imageName, ext] = fileparts(imageSavePath);
    
    % 确保目录存在
    if ~exist(saveDir, 'dir')
        mkdir(saveDir);
    end

    % 保存图像，300 DPI PNG 格式
    print(figHandle, imageSavePath, '-dpng', '-r300');
    disp(['图像已保存到: ', imageSavePath]);

    % 解析路径获取上两级目录名
    % 解析路径获取上两级目录名
    [parentDir, ~] = fileparts(saveDir); % 上级目录名
    %[grandParentDir, ~] = fileparts(parentDir); % 上两级目录名
    [~, GGPFolderName] = fileparts(parentDir); % 上两级目录名

    % 创建图注内容
    figureCaption = [GGPFolderName];

    % 检查 Markdown 文件是否存在
    if ~exist(markdownPath, 'file')
        % 如果不存在，创建文件并添加标题
        fid = fopen(markdownPath, 'w');
        if fid == -1
            error('无法创建 Markdown 文件: %s', markdownPath);
        end
        mdTitle = sprintf('# %s\n', keyword);
        fprintf(fid, '%s\n', mdTitle); % 初始化标题
        fclose(fid);
    end

    % 创建 Markdown 内容
    mdContent = sprintf(...
    '\n<img src="%s" alt="%s" style="width: 13cm;">\n\n<div style="text-align: center; font-weight: bold;">%s</div>\n\n', ...
    imageSavePath, imageName, figureCaption);


    % 将内容追加到 Markdown 文件
    fid = fopen(markdownPath, 'a');
    if fid == -1
        error('无法打开 Markdown 文件: %s', markdownPath);
    end
    fprintf(fid, '%s\n', mdContent);
    fclose(fid);

    disp(['Markdown 记录已更新到: ', markdownPath]);
end
