% 步骤1: 生成20x20的全1矩阵
matrix = ones(20, 20);

% 步骤2: 保证左上角和右下角的2x2区域不变
matrix(1:2, 1:2) = NaN; % 左上角
matrix(19:20, 19:20) = NaN; % 右下角

% 步骤3: 计算目标0的数量
total_elements = numel(matrix); % 矩阵的总元素个数
num_zeros_target = round(total_elements * 0.3); % 目标0的数量（30%的比例）

% 步骤4: 按目标数量随机将1变为0，并生成聚集块
[row, col] = find(matrix == 1);  % 找到矩阵中所有1的位置
all_positions = [row, col];  % 所有为1的位置

% 随机选择指定数量的障碍中心
num_obstacles = 30; % 设置障碍的数量
obstacle_centers = all_positions(randperm(length(all_positions), num_obstacles), :);

% 设置每个障碍的影响范围
range = 2;  % 设置障碍范围，可以根据需要调整

% 对每个障碍中心进行处理
for i = 1:num_obstacles
    % 获取障碍中心的坐标
    center_row = obstacle_centers(i, 1);
    center_col = obstacle_centers(i, 2);
    
    % 找到影响范围内所有值为1的位置
    affected_rows = [];
    affected_cols = [];
    
    for r = max(1, center_row - range):min(20, center_row + range)
        for c = max(1, center_col - range):min(20, center_col + range)
            if matrix(r, c) == 1
                affected_rows = [affected_rows; r];  % 记录受影响位置的行
                affected_cols = [affected_cols; c];  % 记录受影响位置的列
            end
        end
    end
    
    % 随机将部分位置的1改为0
    num_affected = length(affected_rows);  % 受影响的位置数目
    num_to_change = floor(num_affected * 0.27);  % 修改一半的受影响位置，比例可以调整
    
    if num_to_change > 0
        % 随机选择要改为0的位置
        change_indices = randperm(num_affected, num_to_change);
        
        for j = 1:num_to_change
            r = affected_rows(change_indices(j));
            c = affected_cols(change_indices(j));
            matrix(r, c) = 0;  % 将选中的位置改为0
        end
    end
end


% 步骤: 保证左上角和右下角的2x2区域不变
matrix(1:2, 1:2) = 1; % 左上角
matrix(19:20, 19:20) = 1; % 右下角

% 步骤5: 统计0的数量和占比
num_zeros = sum(matrix(:) == 0); % 0的数量
zero_percentage = (num_zeros / total_elements) * 100; % 0的占比

% 输出结果
fprintf('矩阵中0的数量: %d\n', num_zeros);
fprintf('0的占比: %.2f%%\n', zero_percentage);

% 打开文件进行写入
fileID = fopen('object4_4.txt', 'w');
[m, n] = size(matrix);
for i = 1:m
    fprintf(fileID, '%g\t', matrix(i, 1:n-1));  % 每行元素用制表符分隔
    fprintf(fileID, '%g\n', matrix(i, n));      % 最后一个元素后换行
end
fclose(fileID);