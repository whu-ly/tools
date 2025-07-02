function bestIndices = findBestOverlap(A, B)
    % 获取矩阵 B 的行数
    nB = size(B, 1);
    
    % 初始化存储最佳匹配 A 行索引的数组
    bestIndices = zeros(nB, 1);

    % 遍历 B 中的每一行
    for i = 1:nB
        % 获取 B 当前行的时间区间
        startB = gpsTimeToSeconds(B(i, 1), B(i, 2));
        endB = gpsTimeToSeconds(B(i, 3), B(i, 4));

        maxOverlap = 0;
        bestIdx = 0;

        % 遍历 A 中的每一行
        for j = 1:size(A, 1)
            startA = gpsTimeToSeconds(A(j, 1), A(j, 2));
            endA = gpsTimeToSeconds(A(j, 3), A(j, 4));

            % 计算重叠区间长度
            overlapStart = max(startA, startB);
            overlapEnd = min(endA, endB);
            overlap = max(0, overlapEnd - overlapStart);  % 避免负数

            % 记录最大重叠的索引
            if overlap > maxOverlap
                maxOverlap = overlap;
                bestIdx = j;
            end
        end

        % 存储最佳匹配行索引
        bestIndices(i) = bestIdx;
    end
end

function sec = gpsTimeToSeconds(week, seconds)
    % GPS 周和周内秒转换为总秒数
    sec = week * 7 * 24 * 3600 + seconds;
end
