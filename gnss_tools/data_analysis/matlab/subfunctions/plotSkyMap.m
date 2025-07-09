function [Init1] = plotSkyMap(figHandle, Init, Az, El, Value, Sat, Title, type)
    % plotSkyMap 绘制多历元卫星天空视图，颜色随指标变化
    % 输入：
    %   Init: 1说明是第一个绘制卫星
    %   Az：方位角（角度）
    %   El：高度角（角度）
    %   Value: 指标（如信噪比）
    %   Sat：卫星标签，如 "G01", "E01" 等
    %   Title: 图片标题

    Init1 = Init;
    if all(isnan(Az))||all(isnan(El))||all(isnan(Value))
        return;
    end

    % 角度转弧度的常量
    D2R = pi / 180;

    % 转换为弧度，并处理高度角
    Az = Az * D2R; 
    El = -El;  % El需要取反，因为雷达图半径只能由大到小

    % 加载常量文件（如字体设置等）
    run('constant.m');

    % 设置图形大小
    %gcaPos1 = [0.06, 0.06, 0.89, 0.88]; % 无title
    %gcfPos1 = [10, 10, 8.4*2, 6.45*2];  % 使用 centimeters 单位
    gcaPos1 = [0.06, 0.06, 0.88, 0.82];  % 使用 normalized 单位
    gcfPos1 = [10, 10, 8.4*2, 7.0*2];  % 使用 centimeters 单位

    % 创建图形
    if Init == 1
        figure1 = figure(figHandle);
        set(figure1, 'Color', 'w', 'Units', 'centimeters', 'Position', gcfPos1);  % 设置图形大小和位置
        ax = polaraxes;
        ax.Position = gcaPos1;  % 设置极坐标轴的位置
        hold on;

        % 添加标题
        title(Title, 'FontSize', fontSize, 'FontName', fontName, 'FontWeight', 'bold');
    end

    % 设置雷达图的角度和半径限制
    ax.ThetaZeroLocation = 'top';  % 将零度位置设置为图的顶部
    ax.ThetaDir = 'clockwise';  % 设置角度方向为顺时针
    ax.RLim = [-90 0];  % 设置半径范围
    ax.LineWidth = 1.5;  % 设置轴线宽度为1.5

    % 设置半径刻度
    rticks([-60 -30 -10]);  % 设置半径刻度
    rticklabels('');  % 清空默认的刻度标签
    % 手动在 15° 位置绘制半径刻度标签
    theta_90 = 15 * D2R;  % 15° 转换为弧度
    r_values = [-55 -25 -5];  % 半径刻度值
    labels = {'60°', '30°', '10°'};  % 对应的标签
    for i = 1:length(r_values)
        text(theta_90, r_values(i), labels{i}, ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'middle', ...
            'FontSize', fontSize, ...
            'FontName', fontName, ...
            'Color', 'k');  % 黑色
    end

    % 设置角度刻度
    thetaticks(0:30:330);  % 设置角度刻度
    thetaticklabels({'N', '30°', '60°', 'E', '120°', '150°', 'S', '210°', '240°', 'W', '300°', '330°'});

    % 加粗半径刻度
    theta = linspace(0, 2*pi, 100);  % 生成 0 到 360° 的角度
    r = -0 * ones(size(theta));  % 半径为 -0°（图中的 0°，方向朝内）
    polarplot(theta, r, '-k', 'LineWidth', 1.5);  % 绘制虚线，颜色为黑色，线宽为 1.5
    r = -10 * ones(size(theta));  % 半径为 -10°（图中的 10°，方向朝内）
    polarplot(theta, r, '--k', 'LineWidth', 1.5);  % 绘制虚线，颜色为黑色，线宽为 1.5
    r = -30 * ones(size(theta));  % 半径为 -30°（图中的 30°，方向朝内）
    polarplot(theta, r, '-k', 'LineWidth', 1.5);  % 绘制虚线，颜色为黑色，线宽为 1.5
    r = -60 * ones(size(theta));  % 半径为 -60°（图中的 60°，方向朝内）
    polarplot(theta, r, '-k', 'LineWidth', 1.5);  % 绘制虚线，颜色为黑色，线宽为 1.5

    % 加粗角度刻度
    r_range = linspace(-90, 0, 100);  % 半径范围从 -90 到 0
    for i=1:360/30
        theta = (i - 1) * 30 * D2R;  % 将 30° 转换为弧度
        polarplot(theta * ones(size(r_range)), r_range, '-k', 'LineWidth', 1.5);  % 绘制黑色虚线
    end

    % 设置图形的字体格式（应在最后设置，以保证不被覆盖）
    set(gca, 'FontSize', fontSize, 'FontName', fontName, 'LineWidth', 1.0);

    % 绘制卫星
    % 判断卫星标签的首字母，并设置颜色
    if startsWith(Sat, 'G')
        color = color_7x1{2};  % 蓝色
    elseif startsWith(Sat, 'C')
        color = color_7x1{1};  % 红色
    elseif startsWith(Sat, 'E')
        color = color_7x1{4};  % 黄色
    else
        color = 'k';  % 默认黑色（可以根据需要修改）
    end

    % 绘制标记
    if type==0
        polarscatter(Az, El, 20, Value, 'o', 'filled', 'LineWidth', 0.5, 'MarkerFaceAlpha', 0.7);
        c = colorbar; % colormap(gca,'winter');
        c.Label.String = 'SNR (dB-Hz)'; % 设置颜色条的标签
        c.Label.FontSize = fontSize; % 使其与其他字体大小一致
        c.Label.FontName = fontName; % 使其与其他字体样式一致
    else
        polarscatter(Az, El, 50, color, 'o', 'filled', 'LineWidth', 1.5);
    end
    
    % 设置标签
    for index=length(Az):-1:1
        if isnan(Az(index)) || isnan(El(index)) || Az(index) == 0 || El(index) == 0
            %啥也不做,index在循环中-1
        else
            break;
        end
    end
    xpos = Az(index);
    ypos = El(index);

    % 在每个卫星的位置旁边显示卫星号，文字颜色与标记一致
    text(xpos, ypos, Sat, 'VerticalAlignment', 'bottom', ...
        'HorizontalAlignment', 'right', 'FontName', fontName, ...
        'FontSize', fontSize - 2, 'FontWeight', 'bold', 'Color', color);

    Init1 = 2;
end
