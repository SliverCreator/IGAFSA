function [ nextPosition,nextPositionH] = GridAF_prey(n,N,ppValue,X,ii,try_number,lastH, Barrier,goal, k, MAXGEN,rightInf,BX,a)
%���������
% n----����ά��
% present_position_value----ppValue�˹��㵱ǰ��դ���е�ֵ(����)
% ii----��ǰ�˹������
% try_number----����Դ���
% lastH----�ϴ��˹���ʳ��Ũ��
% Barrier----�ϰ�����
% goal----�˹���Ŀ��λ��
%���������
% nextPosition----��һʱ�̵��˹���դ�����ֵ
%nextPositionH----��һʱ�̸����˹���λ�ô���ʳ��Ũ��
%j ��ǰ�������������ڼ�������Ӧ����
nextPosition = [];
allow_area = [];%��¼������
j = 1;%��¼�����������ĸ���
present_H = lastH(ii);%��ǰλ��ʱ�̵�ʵ��Ũ��ֵ.
Xi = ppValue;

%����Ӧ����
% rightInf = ceil(rightInf*(1 - j/MAXGEN));%����j�������������ӣ�rightInf����ϵ����С��ceilΪ����ȡ��
alpha = 1;
D = eachAF_dist(n,N,Xi,X);%���㵱ǰ�˹���������������Ⱥ��ŷʽ����
if D>0
    visual = mean(D); %����Ӧ��Ȩ��Ұ
    rightInf = alpha * visual;
end

%%
%�˹���Ŀ����򣬼������ǰλ���ܱ����ߵĵ㣬�����ڸ���2����
A = 1:1:n^2;
allow = setdiff(A,Barrier);    
for i = 1:1:length(allow)
    if 0 < distance(n,ppValue,allow(i)) && distance(n,ppValue,allow(i)) <= rightInf  
        allow_area(j) = allow(i);
        j = j+1; 
    end
end
% allow_area = allow_fun(n,ppValue,allow_area);
%%
%���뷽�����ӣ�ֱ��ѡ��allow_area�����е���Сֵ
m = randsrc(1,1, [0 1; 0.2 0.8]);%������һ��1*1�ľ�����0��1֮�����ȡֵ��0�ĸ���0.2��1�ĸ���0.8
%���뷽�����ӣ�ֱ��ѡ��allow_area�����е���Сֵ
if m == 0
    %����ѡ��ֱ�������һ��
        for i = 1:1:try_number
            Xj = allow_area(uint16(rand*(length(allow_area)-1)+1));%�ڿ����������ѡ��һ��.
            % X_i = Xi + ((Xj-Xi)+(BX-Xi))/(norm((Xj-Xi)+(BX-Xi))) * rightInf * rand();
            % if ismember(ceil(X_i),allow)
            %     X_i = ceil(X_i);
            % elseif ismember(floor(X_i),allow)
            %     X_i = floor(X_i);
            % else
            %     X_i = allow_area(uint16(rand*(length(allow_area)-1)+1));
            % end
            Hj = GrideAF_foodconsistence(n,Xj,goal);
            if present_H > Hj%˵����һ����ֵ����goal����������
               nextPosition = Xj;%��ΪXj�ڿ������У����Բ����ж��Ƿ�Խ��.
               break;  %�ҵ�һ��Сֵ�ͽ���ѭ�������Եó��Ľ��Ϊ�����н⣬���������Ž�.
            end
        end
else
    H_min = present_H;%����ǰλ�õ�ʳ��Ũ�ȸ�ֵ��H_min
    for i  = 1:1:length(allow_area)%�ڿ�����Χ�ڱ�������λ��
        Xi = allow_area(i);
        Hi = GrideAF_foodconsistence(n,Xi,goal);%���������Χ����λ�õ�ʳ��Ũ��
        if Hi < H_min%�������Ŀ�����λ�õ�ʳ��Ũ�ȱȼ�¼�ĵ�ǰλ��ֵ��С����ѡ���µ�λ��
            H_min = Hi;
            nextPosition = Xi;
        end
    end
end

if isempty(nextPosition)%�ж���һλ���Ƿ�Ϊ�գ�Ϊ����ִ��
     Xj = allow_area(uint16(rand*(length(allow_area)-1)+1));%�ڿ����������ѡ��һ��.
     nextPosition = Xj;%��ΪXj�ڿ������У����Բ����ж��Ƿ�Խ��.
end

%%
nextPositionH = GrideAF_foodconsistence(n,nextPosition,goal);%���ú���������һλ�õ�ʳ��Ũ��
end

