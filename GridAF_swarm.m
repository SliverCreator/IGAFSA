function  [ nextPosition,nextPositionH ] = GridAF_swarm(n,N,ppValue,ii,visual,delta,try_number,lastH,Barrier,goal, j, MAXGEN,rightInf,BX,a)
%输入参数：
% n----矩阵维数
% N----人工鱼总数
% ppValue----所有人工鱼群的位置
% ii----当前人工鱼群的编号
% visual----感知范围
% delta----拥挤因子
% try_number----尝试次数
% LastH----当前人工鱼上一次的食物浓度
% Barrier----障碍物矩阵
% goal----人工鱼目标位置
% 输出参数：
% nextPosition----下一时刻该条人工鱼的位置
% nextPositionH----下一时刻该条人工鱼位置处的食物浓度
%j , MAXGEN用于传递prey函数的参数，与本函数无关
sumx = 0;%记录视野中人工鱼X轴数据之和
sumy = 0;%记录视野中人工鱼Y轴数据之和

Xi = ppValue(ii);%当前人工鱼的位置
D = eachAF_dist(n,N,Xi,ppValue);%计算当前人工鱼与其他所有鱼群的欧式距离
visual = mean(D); %自适应加权视野
index = find(D > 0 & D < visual);%找到视野中的其他鱼群
Nf = length(index);%确定视野之内的鱼群总数
j = 1;%记录单个鱼可行域的个数
%%
%计算当前人工鱼的可行域
A = 1:1:n^2;
allow = setdiff(A,Barrier);    
for i = 1:1:length(allow)
    if 0 < distance(n,Xi,allow(i)) && distance(n,Xi,allow(i)) <= rightInf  
        allow_area(j) = allow(i);
        j = j+1;
    end
end
% allow_area = allow_fun(n,Xi,allow_area);
%%
if Nf > 0          %Nf > 0说明视野之内有其他人工鱼群，则可以进行群聚行为
    for i = 1:1:Nf
        [row,col] = ind2sub(n,ppValue(index(i))); 
        [array_x,array_y] = arry2orxy(n,row,col);
        sumx = sumx+array_x;
        sumy = sumy+array_y;
    end
    %因为存在非整数的可能性，所以要把他们规整为整数处理,只要存在小数位就加一位处理，（逢小数，进一原则）
    avgx = ceil(sumx/Nf);%得到X轴的均值，并且整数化
    avgy = ceil(sumy/Nf);%得到Y轴的均值,并且整数化
    %因为将坐标反对应到矩阵值，再求食物浓度
    Xc = sub2ind([n,n],avgy,avgx);%算出中心点位置，注意，坐标的x，y与矩阵的行列是交叉对应的
    Hc = GrideAF_foodconsistence(n,Xc,goal); %算出中心点和终点的距离
    Hi = lastH(ii);%当前人工鱼的实物浓度
    if Hc/Nf <= Hi*delta     %如果中心位置的食物浓度比当前位置高，并且不拥挤，则向中心位置走一步      PS：H值越小，则离目标越近
        %在可行域中，随机走一步，比较到中心点距离与当前值到中心点距离，谁小就取谁
        for i = 1:1:try_number
            % Xnext = allow_area(uint16(rand*(length(allow_area)-1)+1));%在可行域中随机选择一步.
            Xnext = Xi + ((Xc-Xi)+(BX-Xi))/(norm((Xc-Xi)+(BX-Xi))) * rightInf * rand();
            % if ismember(ceil(Xnext),allow)
            %     Xnext = ceil(Xnext);
            % elseif ismember(floor(Xnext),allow)
            %     Xnext = floor(Xnext);
            % else
            %     Xnext = allow_area(uint16(rand*(length(allow_area)-1)+1));
            % end
            if distance(n,Xnext,Xc) < distance(n,Xi,Xc)
                nextPosition = Xnext;
                nextPositionH = GrideAF_foodconsistence(n,nextPosition,goal);
                break;
            else
                nextPosition = Xi;
                nextPositionH = GrideAF_foodconsistence(n,nextPosition,goal);
            end
        end
    else     %如果中心位置的食物浓度没有当前位置的食物浓度高，则进行觅食行为
        [nextPosition,nextPositionH] = GridAF_prey(n,N,ppValue(ii),ppValue,ii,try_number,lastH, Barrier,goal, j, MAXGEN,rightInf,BX,a);
    end
    
else                %否则，Nf < 0说明视野范围内没有其他人工鱼，那么就执行觅食行为  
    [nextPosition,nextPositionH] = GridAF_prey(n,N,ppValue(ii),ppValue,ii,try_number,lastH, Barrier,goal, j, MAXGEN,rightInf,BX,a);
end

