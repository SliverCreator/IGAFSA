function [ allow_area ] = allow_fun(n, ppValue,allow_area )
%计算当前人工鱼的可行域
%输入参数：
% n----矩阵维数
% Barrier----障碍物
% Xi----当前人工鱼的位置
% rightInf-Rf----可行域上阶，可以理解为视野
% 输出参数：
% allow_area----当前人工鱼的可行域
% 假设 ppValue 是当前位置的线性索引
% 假设 allow_area 是一个包含所有允许区域线性索引的数组
n = 20;
% 计算当前位置的四个斜向索引（假设矩阵是 m x n 大小）
[Xi(:,1),Xi(:,2)] = ind2sub(n, ppValue); % 将线性索引转换为行列坐标
[Xallow(:,1),Xallow(:,2)] = ind2sub(n, allow_area);
% 初始化一个空矩阵来存储合理的斜向坐标
Check = [];

% 计算四个斜向的坐标
offsets = [-1 -1; -1 1; 1 -1; 1 1]; % 对应左上、右上、左下、右下的行列偏移量
% i的1234对应左上、右上、左下、右下
for i = 1:size(offsets, 1)
    newRow = Xi(1) + offsets(i, 1);
    newCol = Xi(2) + offsets(i, 2);
    % 检查新坐标是否在矩阵的合理范围内
    if newRow >= 1 && newRow <= n && newCol >= 1 && newCol <= n
        % 如果在范围内，添加到diagonalCoords
        Check = [Check; newRow, newCol,i];
    end
end

for i = 1:size(Check, 1)
    X = Check(i,1:2);
    if ismember(X, Xallow,'rows')
        j = Check(i,3);
        switch j
            case 1
                X1 = [Xi(1)-1,Xi(2)];
                X2 = [Xi(1),Xi(2)-1];
                if ~ismember(X1, Xallow,'rows') || ~ismember(X2,Xallow,'rows')
                    Xallow = setdiff(Xallow, X,'rows', 'stable');
                end
            case 2
                X1 = [Xi(1)-1,Xi(2)];
                X2 = [Xi(1),Xi(2)+1];
                if ~ismember(X1, Xallow,'rows') || ~ismember(X2,Xallow,'rows')
                    Xallow = setdiff(Xallow, X,'rows', 'stable');
                end
            case 3
                X1 = [Xi(1)+1,Xi(2)];
                X2 = [Xi(1),Xi(2)-1];
                if ~ismember(X1, Xallow,'rows') || ~ismember(X2,Xallow,'rows')
                    Xallow = setdiff(Xallow, X,'rows', 'stable');
                end
            case 4
                X1 = [Xi(1)+1,Xi(2)];
                X2 = [Xi(1),Xi(2)+1];
                if ~ismember(X1, Xallow,'rows') || ~ismember(X2,Xallow,'rows')
                    Xallow = setdiff(Xallow, X,'rows', 'stable');
                end
        end
    end
end
allow_area = sub2ind([n n], Xallow(:,1), Xallow(:,2));
allow_area = allow_area';

% if row == 1 && col == 1   %起点
%     bottomRight = sub2ind([n n], row+1, col+1); % 右下
%     bottom = sub2ind([n n],row+1,col);%下
%     right = sub2ind([n n],row,col+1);%右
% elseif row ==1 && col > 1    % 第一行
%     bottomLeft = sub2ind([n n], row+1, col-1); % 左下
%     bottomRight = sub2ind([n n], row+1, col+1); % 右下
%     left = sub2ind([n n], row, col-1); % 左
%     bottom = sub2ind([n n],row+1,col);%下
%     right = sub2ind([n n],row,col+1);%右
% elseif row >1 && col ==1    % 第一列
%     topRight = sub2ind([n n], row-1, col+1); % 右上
%     bottomRight = sub2ind([n n], row+1, col+1); % 右下
%     bottom = sub2ind([n n],row+1,col);%下
%     right = sub2ind([n n],row,col+1);%右
%     top = sub2ind([n n], row-1, col); % 上
% elseif row ==n && col <n   %   最后一行
%     topLeft = sub2ind([n n], row-1, col-1); % 左上
%     topRight = sub2ind([n n], row-1, col+1); % 右上
%     top = sub2ind([n n], row-1, col); % 上
%     left = sub2ind([n n], row, col-1); % 左
%     right = sub2ind([n n],row,col+1);%右
% elseif row <n && col ==n     % 最后一列
%     topLeft = sub2ind([n n], row-1, col-1); % 左上
%     bottomLeft = sub2ind([n n], row+1, col-1); % 左下
%     top = sub2ind([n n], row-1, col); % 上
%     left = sub2ind([n n], row, col-1); % 左
%     bottom = sub2ind([n n],row+1,col);%下
% elseif row >1 && col >1  && row <n && col <n
%     % 获取四个斜向的线性索引
%     top = sub2ind([n n], row-1, col); % 上
%     left = sub2ind([n n], row, col-1); % 左
%     bottom = sub2ind([n n],row+1,col);%下
%     right = sub2ind([n n],row,col+1);%右
%     topLeft = sub2ind([n n], row-1, col-1); % 左上
%     topRight = sub2ind([n n], row-1, col+1); % 右上
%     bottomLeft = sub2ind([n n], row+1, col-1); % 左下
%     bottomRight = sub2ind([n n], row+1, col+1); % 右下
% end
% 
% % 检查每个斜向索引
% if exist("topLeft","var")
%     if ismember(topLeft, allow_area)
%         % 如果左上角索引在允许区域内，检查上和左索引
%         if ismember(top, allow_area) && ismember(left, allow_area)
%             % 如果上和左索引同时存在，跳出循环
%             return; % 跳出循环
%         else
%             % 如果上和左索引不同时存在，从allow_area中删除左上索引
%             allow_area = setdiff(allow_area, topLeft);
%         end
%     end
% end
% 
% if exist("topRight","var")
%     if ismember(topRight, allow_area)
%         % 如果右上角索引在允许区域内，检查上和右索引
%         if ismember(top, allow_area) && ismember(right, allow_area)
%             % 如果上和右索引同时存在，跳出循环
%             return; % 跳出循环
%         else
%             % 如果上和右索引不同时存在，从allow_area中删除右上索引
%             allow_area = setdiff(allow_area, topRight);
%         end
%     end
% end
% 
% if exist("bottomLeft","var")
%     if ismember(bottomLeft, allow_area)
%         % 如果左下角索引在允许区域内，检查下和左索引
%         if ismember(left, allow_area) && ismember(bottom, allow_area)
%             % 如果下和左索引同时存在，跳出循环
%             return; % 跳出循环
%         else
%             % 如果下和左索引不同时存在，从allow_area中删除左下索引
%             allow_area = setdiff(allow_area, bottomLeft);
%         end
%     end
% end
% 
% if exist("bottomRight","var")
%     if ismember(bottomRight, allow_area)
%         % 如果右下角索引在允许区域内，检查下和右索引
%         if ismember(bottom, allow_area) && ismember(right, allow_area)
%             % 如果下和右索引同时存在，跳出循环
%             return; % 跳出循环
%         else
%             % 如果下和右索引不同时存在，从allow_area中删除右下索引
%             allow_area = setdiff(allow_area, bottomRight);
%         end
%     end
% end
% 
% end
