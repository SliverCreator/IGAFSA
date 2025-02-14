% 读取txt文件中的矩阵
matrix = load('G4_1.txt');  % 假设文件为矩阵格式，矩阵按行分隔

% 统计0和1的数量
num_zeros = sum(matrix(:) == 0);  % 统计0的数量
num_ones = sum(matrix(:) == 1);   % 统计1的数量

% 计算矩阵的总元素数量
total_elements = numel(matrix);

% 计算0的占比
zero_ratio = num_zeros / total_elements;

% 设定目标0的占比为20%
target_zero_ratio = 0.30;

% 输出当前统计结果
fprintf('矩阵中0的数量: %d\n', num_zeros);
fprintf('矩阵中1的数量: %d\n', num_ones);
fprintf('0的占比: %.2f%%\n', zero_ratio * 100);

% 确保矩阵左上角和右下角的2x2区域不被修改
top_left_2x2 = matrix(1:2, 1:2);       % 左上角2x2区域
bottom_right_2x2 = matrix(end-1:end, end-1:end); % 右下角2x2区域

% 从矩阵中去除这两个区域
matrix(1:2, 1:2) = NaN;  % 使用NaN标记以免被修改
matrix(end-1:end, end-1:end) = NaN;

% 计算目标0的数量
target_num_zeros = round(target_zero_ratio * total_elements);
num_to_change = target_num_zeros - num_zeros;
fprintf('需要更改的数量: %d\n', num_to_change);

% 判断当前0的数量和目标值的关系
if num_zeros < target_num_zeros
    % 当前0的比例比设定值小，随机将1改为0
    num_to_change = target_num_zeros - num_zeros;  % 需要修改的1的数量
    [row, col] = find(matrix == 1);  % 找到矩阵中所有1的位置
    num_elements = numel(row);

    % 随机选择位置
    random_indices = randperm(num_elements, num_to_change);
    for i = 1:num_to_change
        matrix(row(random_indices(i)), col(random_indices(i))) = 0;
    end
elseif num_zeros > target_num_zeros
    % 当前0的比例比设定值大，随机将0改为1
    num_to_change = num_zeros - target_num_zeros;  % 需要修改的0的数量
    [row, col] = find(matrix == 0);  % 找到矩阵中所有0的位置
    num_elements = numel(row);

    % 随机选择位置
    random_indices = randperm(num_elements, num_to_change);
    for i = 1:num_to_change
        matrix(row(random_indices(i)), col(random_indices(i))) = 1;
    end
end

% 恢复左上角和右下角2x2区域的值
matrix(1:2, 1:2) = top_left_2x2;
matrix(end-1:end, end-1:end) = bottom_right_2x2;

% 输出修改后的矩阵
fprintf('修改后的0的占比: %.2f%%\n', sum(matrix(:) == 0) / total_elements * 100);

% 也可以使用fprintf手动保存，以确保矩阵格式为你期望的方式
% 打开文件进行写入
fileID = fopen('object3_1.txt', 'w');
[m, n] = size(matrix);
for i = 1:m
    fprintf(fileID, '%g\t', matrix(i, 1:n-1));  % 每行元素用制表符分隔
    fprintf(fileID, '%g\n', matrix(i, n));      % 最后一个元素后换行
end
fclose(fileID);