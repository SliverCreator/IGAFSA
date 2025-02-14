function [ nextPosition,nextPositionH] = GridAF_prey(n,N,ppValue,X,ii,try_number,lastH, Barrier,goal, k, MAXGEN,rightInf,BX,a)
%输入参数：
% n----矩阵维数
% present_position_value----ppValue人工鱼当前在栅格中的值(单个)
% ii----当前人工鱼序号
% try_number----最大尝试次数
% lastH----上次人工鱼食物浓度
% Barrier----障碍矩阵
% goal----人工鱼目标位置
%输出参数：
% nextPosition----下一时刻的人工鱼栅格矩阵值
%nextPositionH----下一时刻该条人工鱼位置处的食物浓度
%j 当前迭代次数，用于计算自适应步长
nextPosition = [];
allow_area = [];%记录可行域
j = 1;%记录单个鱼可行域的个数
present_H = lastH(ii);%当前位置时刻的实物浓度值.
Xi = ppValue;

%自适应步长
% rightInf = ceil(rightInf*(1 - j/MAXGEN));%随着j迭代次数的增加，rightInf所乘系数减小，ceil为向上取整
alpha = 1;
D = eachAF_dist(n,N,Xi,X);%计算当前人工鱼与其他所有鱼群的欧式距离
if D>0
    visual = mean(D); %自适应加权视野
    rightInf = alpha * visual;
end

%%
%人工鱼的可行域，计算出当前位置周边能走的点，距离在根号2以内
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
%加入方向因子，直接选择allow_area区域中的最小值
m = randsrc(1,1, [0 1; 0.2 0.8]);%设置了一个1*1的矩阵，在0和1之间概率取值，0的概率0.2，1的概率0.8
%加入方向因子，直接选择allow_area区域中的最小值
if m == 0
    %正常选择，直接随机走一步
        for i = 1:1:try_number
            Xj = allow_area(uint16(rand*(length(allow_area)-1)+1));%在可行域中随机选择一步.
            % X_i = Xi + ((Xj-Xi)+(BX-Xi))/(norm((Xj-Xi)+(BX-Xi))) * rightInf * rand();
            % if ismember(ceil(X_i),allow)
            %     X_i = ceil(X_i);
            % elseif ismember(floor(X_i),allow)
            %     X_i = floor(X_i);
            % else
            %     X_i = allow_area(uint16(rand*(length(allow_area)-1)+1));
            % end
            Hj = GrideAF_foodconsistence(n,Xj,goal);
            if present_H > Hj%说明下一步的值距离goal更近，保留
               nextPosition = Xj;%因为Xj在可行域中，所以不用判断是否越界.
               break;  %找到一个小值就结束循环，所以得出的结果为，可行解，而不是最优解.
            end
        end
else
    H_min = present_H;%将当前位置的食物浓度赋值给H_min
    for i  = 1:1:length(allow_area)%在可行域范围内遍历所有位置
        Xi = allow_area(i);
        Hi = GrideAF_foodconsistence(n,Xi,goal);%求出可行域范围所有位置的食物浓度
        if Hi < H_min%若遍历的可行域位置的食物浓度比记录的当前位置值更小，则选择新的位置
            H_min = Hi;
            nextPosition = Xi;
        end
    end
end

if isempty(nextPosition)%判断下一位置是否为空，为空则执行
     Xj = allow_area(uint16(rand*(length(allow_area)-1)+1));%在可行域中随机选择一步.
     nextPosition = Xj;%因为Xj在可行域中，所以不用判断是否越界.
end

%%
nextPositionH = GrideAF_foodconsistence(n,nextPosition,goal);%调用函数计算下一位置的食物浓度
end

