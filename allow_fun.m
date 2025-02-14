function [ allow_area ] = allow_fun(n, ppValue,allow_area )
%���㵱ǰ�˹���Ŀ�����
%���������
% n----����ά��
% Barrier----�ϰ���
% Xi----��ǰ�˹����λ��
% rightInf-Rf----�������Ͻף��������Ϊ��Ұ
% ���������
% allow_area----��ǰ�˹���Ŀ�����
% ���� ppValue �ǵ�ǰλ�õ���������
% ���� allow_area ��һ��������������������������������
n = 20;
% ���㵱ǰλ�õ��ĸ�б����������������� m x n ��С��
[Xi(:,1),Xi(:,2)] = ind2sub(n, ppValue); % ����������ת��Ϊ��������
[Xallow(:,1),Xallow(:,2)] = ind2sub(n, allow_area);
% ��ʼ��һ���վ������洢�����б������
Check = [];

% �����ĸ�б�������
offsets = [-1 -1; -1 1; 1 -1; 1 1]; % ��Ӧ���ϡ����ϡ����¡����µ�����ƫ����
% i��1234��Ӧ���ϡ����ϡ����¡�����
for i = 1:size(offsets, 1)
    newRow = Xi(1) + offsets(i, 1);
    newCol = Xi(2) + offsets(i, 2);
    % ����������Ƿ��ھ���ĺ���Χ��
    if newRow >= 1 && newRow <= n && newCol >= 1 && newCol <= n
        % ����ڷ�Χ�ڣ���ӵ�diagonalCoords
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

% if row == 1 && col == 1   %���
%     bottomRight = sub2ind([n n], row+1, col+1); % ����
%     bottom = sub2ind([n n],row+1,col);%��
%     right = sub2ind([n n],row,col+1);%��
% elseif row ==1 && col > 1    % ��һ��
%     bottomLeft = sub2ind([n n], row+1, col-1); % ����
%     bottomRight = sub2ind([n n], row+1, col+1); % ����
%     left = sub2ind([n n], row, col-1); % ��
%     bottom = sub2ind([n n],row+1,col);%��
%     right = sub2ind([n n],row,col+1);%��
% elseif row >1 && col ==1    % ��һ��
%     topRight = sub2ind([n n], row-1, col+1); % ����
%     bottomRight = sub2ind([n n], row+1, col+1); % ����
%     bottom = sub2ind([n n],row+1,col);%��
%     right = sub2ind([n n],row,col+1);%��
%     top = sub2ind([n n], row-1, col); % ��
% elseif row ==n && col <n   %   ���һ��
%     topLeft = sub2ind([n n], row-1, col-1); % ����
%     topRight = sub2ind([n n], row-1, col+1); % ����
%     top = sub2ind([n n], row-1, col); % ��
%     left = sub2ind([n n], row, col-1); % ��
%     right = sub2ind([n n],row,col+1);%��
% elseif row <n && col ==n     % ���һ��
%     topLeft = sub2ind([n n], row-1, col-1); % ����
%     bottomLeft = sub2ind([n n], row+1, col-1); % ����
%     top = sub2ind([n n], row-1, col); % ��
%     left = sub2ind([n n], row, col-1); % ��
%     bottom = sub2ind([n n],row+1,col);%��
% elseif row >1 && col >1  && row <n && col <n
%     % ��ȡ�ĸ�б�����������
%     top = sub2ind([n n], row-1, col); % ��
%     left = sub2ind([n n], row, col-1); % ��
%     bottom = sub2ind([n n],row+1,col);%��
%     right = sub2ind([n n],row,col+1);%��
%     topLeft = sub2ind([n n], row-1, col-1); % ����
%     topRight = sub2ind([n n], row-1, col+1); % ����
%     bottomLeft = sub2ind([n n], row+1, col-1); % ����
%     bottomRight = sub2ind([n n], row+1, col+1); % ����
% end
% 
% % ���ÿ��б������
% if exist("topLeft","var")
%     if ismember(topLeft, allow_area)
%         % ������Ͻ����������������ڣ�����Ϻ�������
%         if ismember(top, allow_area) && ismember(left, allow_area)
%             % ����Ϻ�������ͬʱ���ڣ�����ѭ��
%             return; % ����ѭ��
%         else
%             % ����Ϻ���������ͬʱ���ڣ���allow_area��ɾ����������
%             allow_area = setdiff(allow_area, topLeft);
%         end
%     end
% end
% 
% if exist("topRight","var")
%     if ismember(topRight, allow_area)
%         % ������Ͻ����������������ڣ�����Ϻ�������
%         if ismember(top, allow_area) && ismember(right, allow_area)
%             % ����Ϻ�������ͬʱ���ڣ�����ѭ��
%             return; % ����ѭ��
%         else
%             % ����Ϻ���������ͬʱ���ڣ���allow_area��ɾ����������
%             allow_area = setdiff(allow_area, topRight);
%         end
%     end
% end
% 
% if exist("bottomLeft","var")
%     if ismember(bottomLeft, allow_area)
%         % ������½����������������ڣ�����º�������
%         if ismember(left, allow_area) && ismember(bottom, allow_area)
%             % ����º�������ͬʱ���ڣ�����ѭ��
%             return; % ����ѭ��
%         else
%             % ����º���������ͬʱ���ڣ���allow_area��ɾ����������
%             allow_area = setdiff(allow_area, bottomLeft);
%         end
%     end
% end
% 
% if exist("bottomRight","var")
%     if ismember(bottomRight, allow_area)
%         % ������½����������������ڣ�����º�������
%         if ismember(bottom, allow_area) && ismember(right, allow_area)
%             % ����º�������ͬʱ���ڣ�����ѭ��
%             return; % ����ѭ��
%         else
%             % ����º���������ͬʱ���ڣ���allow_area��ɾ����������
%             allow_area = setdiff(allow_area, bottomRight);
%         end
%     end
% end
% 
% end
