function [waypoint_x, waypoint_y, waypoint_yaw] = improveAF_main()
%%
% 本函数为改进人工鱼群法主函数
% 输入：无
% 输出：waypoint的x坐标，y坐标，艏向
% 对于传统人工鱼群算法，在栅格环境中的优化.
% 改进方法如下：
% 自适应视野和步长
% 在觅食行为中加入方向因子和概率算子
% 基于弗洛伊德算法的路径优化
%%
close all;
clear all;
clc;
%%
%画出机器人的障碍环境，并标注start,goal位置
tic;
a = image_process();
% a = logical(a);
n = size(a,1);
b = a;
b(end+1,end+1) = 0;
figure;
colormap([0 0 0;1 1 1])
pcolor(b); % 赋予栅格颜色
set(gca,'XTick',0:10:n,'YTick',0:10:n);  % 设置坐标
% axis image xy

% text(0.5,0.5,'START','Color','red','FontSize',10);%显示start字符
% text(n+0.5,n+1.5,'GOAL','Color','red','FontSize',10);%显示goal字符

% case 1
% x_start = 92;
% y_start = 70;
% x_goal = 36;
% y_goal = 22;

%case 2
x_start = 96;
y_start = 74;
x_goal = 31;
y_goal = 23;

hold on

%画出起始点和目标点位置
scatter(x_start+0.5,y_start+0.5,'MarkerEdgeColor',[1 0 0],'MarkerFaceColor',[1 0 0], 'LineWidth',1);%start point
scatter(x_goal+0.5,y_goal+0.5,'MarkerEdgeColor',[0 1 0],'MarkerFaceColor',[0 1 0], 'LineWidth',1);%goal point

hold on

%%
%障碍空间矩阵集合B
Barrier = (find(a==0))';
%%
%初始化
N = 10;%人工鱼数量
try_number = 8;
MAXGEN = 500;%最大迭代次数
visual = 10; %初始值
delta = 0.618;
start = (x_start-1)*n+y_start;%人工鱼群开始位置
DistMin = sqrt((1-n)^2+(n-1)^2);%最短距离标志位
goal = (x_goal-1)*100+y_goal;%目标位置
shift = 1;%觅食行为和{群聚，追尾}行为的模式切换，
shiftFreq = 8;%觅食行为和{群聚，追尾}行为的切 换频率
rightInf = sqrt(2);
arrayValue = [20];%起点先放入要显示的向量中.
%最终waypoint的坐标
x = [];
y = [];
for i =1:1:N
    ppValue(i) = start;%传递人工鱼群位置的变量
end
H =GrideAF_foodconsistence(n,ppValue,goal);
count = 1;%记录执行觅食行为的次数
runDist = 0;%记录行走路径的总长度
runDist_part = 0;%记录每一条可行的行走路径长度，用于比较
BestH = zeros(1,MAXGEN);%记录每一次迭代中的最优H值 
index = [];%记录找到路径的鱼群
%-----------------------------------------至此变量参数初始化全部结束------------------------------------------------------
for j = 1:1:MAXGEN
    switch shift
%------------------------------------觅食行为----------------------------------------------------------------------------
        case 1
            for i = 1:1:N
               [nextPosition,nextPositionH] = GridAF_prey(n,ppValue(i),i,try_number,H,Barrier,goal, j, MAXGEN,rightInf);
               %需要记录下每条鱼对应的位置，以及食物浓度.以便下次更新.
               position(j,i) = nextPosition;%position存放所有鱼群的位置.
               H(i) = nextPositionH;
            end
            disp('prey!!!!')
%------------------------------------群聚行为---------------------------------------------------------------------------    
         case 2     
            for i = 1:1:N
                [nextPosition_S,nextPositionH_S] = GridAF_swarm(n,N,position(j-1,:),i,visual,delta,try_number,H,Barrier,goal, j, MAXGEN,rightInf);
                [nextPosition_F,nextPositionH_F] = GridAF_follow(n,N,position(j-1,:),i,visual,delta,try_number,H,Barrier,goal, j, MAXGEN,rightInf);
                if nextPositionH_F < nextPositionH_S
                    nextPosition = nextPosition_F;
                    nextPositionH = nextPositionH_F;
                else
                    nextPosition = nextPosition_S;
                    nextPositionH = nextPositionH_S;
                end
                 position(j,i) = nextPosition;%position存放所有鱼群的位置.
                 H(i) = nextPositionH;
            end
            disp('swarm & follow!!!')
    end
%-----------------------------------------------------------------------------------------------------------------
    count = count+1;%所有人工鱼都完成了一次觅食行为
    if rem(count,shiftFreq) == 0 %因为count从1开始记，所以5时，正好4次觅食行为
        shift = 2;
    else
        shift = 1;
    end
    
    %要更新ppValue的值
    ppValue =  position(j,:);
    %当在position中找到人工鱼到达goal处时，则跳出循环
    index = find(position(j,:)==goal);
    if ~isempty(index)
        break;
    end
end

%如果是因为最大迭代次数而结束的循环，则说明没有路径到达
if MAXGEN <= j
    disp('There is no way can arrive to the goal!!!');
else

%%
%在所有可行路径中找出最短路径
    for i = 1:1:length(index)
        arrayValue =[start;position(:,index(i))]';
%计算出行走路径的总长度
        for j = 1:1:length(arrayValue)-1
            d = distance(n,arrayValue(j),arrayValue(j+1));
            runDist_part = runDist_part + d;
        end
        transimit(i) = runDist_part;%记录所有可行路径的总长度   
        runDist_part = 0;
    end
    [runDist,runMin_index] = min(transimit);
    arrayValue = [start;position(:,index(runMin_index))]';
    
    for i =1:1:length(arrayValue)
        BestH(i) = goal-arrayValue(i);%记录最优可行解的迭代图
    end
    
    [row,col] = ind2sub(n,arrayValue);%栅格中的数值转化成数组行列值
    Optimal_path = [];
    [array_x,array_y] = arry2orxy(n,row,col);%将矩阵下标转换为坐标轴xy形式
    Optimal_path = [array_x',array_y'];
    % 使用 unique 函数删除重复的行
    [Optimal_path] = unique(Optimal_path, 'rows', 'stable');
    Optimal_path = flipud(Optimal_path);
    for i = 1:size(Optimal_path,1)
        if i == size(Optimal_path,1)
            Optimal_path(i,:) = [];
        else
            Optimal_path(i,3) = Optimal_path(i+1,1);
            Optimal_path(i,4) = Optimal_path(i+1,2);
        end
    end
    %路径优化
    % arrayValue = path_smooth(arrayValue);
    % [waypoint_x, waypoint_y] = DrawPath(x, y, n,arrayValue);%画出行走路径
    % DrawPath(n,arrayValue);%画出行走路径
    % waypoint_num = length(waypoint_x);
    % waypoint_yaw = [0];
    % for i = 1:waypoint_num-1
    %     angle = atan((waypoint_y(i+1)-waypoint_y(i))/(waypoint_x(i+1)-waypoint_x(i)));
    %     waypoint_yaw = [waypoint_yaw angle];
    % end
    %     fprintf('行走长度为: %f\n',runDist)%显示行走路径的长度
end
%% 预备变量
k=1;
CLOSED=[];
MAX = a;
MAX_X=size(MAX,2);                                %%%  获取列数，即x轴长度
MAX_Y=size(MAX,1);                                %%%  获取行数，即y轴长度
for j=1:MAX_X
    for i=1:MAX_Y
        if (MAX(i,j)==1)
          %%plot(i+.5,j+.5,'ks','MarkerFaceColor','b'); 原来是红点圆表示
          % fill([i,i+1,i+1,i],[j,j,j+1,j+1],'k');  %%%改成 用黑方块来表示障碍物
          CLOSED(k,1)=i;  %%% 将障碍点保存到CLOSE数组中
          CLOSED(k,2)=j; 
          k=k+1;
%         else
%           fill([i,i+1,i+1,i],[j,j,j+1,j+1],[1,1,1]);  %%%用黑方块来表示障碍物  
        end
    end
end
Obs_Closed=CLOSED;

xStart = Optimal_path(1,1);      yStart = Optimal_path(1,2); 
xTarget = Optimal_path(1,1);    yTarget = Optimal_path(1,2);

%% 路径优化
    Num_obs=size(Obs_Closed,1);
    CLOSED = Obs_Closed;
    Num_Opt=size(Optimal_path,1); 

    %%%  优化折线
 Optimal_path_one=Line_OPEN_ST(Optimal_path,CLOSED,Num_obs,Num_Opt);
     % 使用 unique 函数删除重复的行
 [~,ia] = unique(Optimal_path_one(:,1:2), 'rows', 'stable');
 % 重建整个 n*4 矩阵，仅包含唯一的行
 Optimal_path_one = Optimal_path_one(ia, :);
%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%   把路径提取出来  %%%%%%%%%%%%%%%%%%%%%
 Optimal_path_try=[1 1 1 1];
 Optimal_path=[1 1 ];%node_l=[1 1];
 i=1;q=1;
 x_g=Optimal_path_one(Num_Opt,3);y_g=Optimal_path_one(Num_Opt,4);
 
 Optimal_path_try(i,1)= Optimal_path_one(q,1);
 Optimal_path_try(i,2)= Optimal_path_one(q,2);
 Optimal_path_try(i,3)= Optimal_path_one(q,3);
 Optimal_path_try(i,4)= Optimal_path_one(q,4);
 
   while (Optimal_path_try(i,3)~=x_g || Optimal_path_try(i,4)~=y_g)
      i=i+1;
      q=Optimal_index(Optimal_path_one,Optimal_path_one(q,3),Optimal_path_one(q,4));
      
      Optimal_path_try(i,1)= Optimal_path_one(q,1);
      Optimal_path_try(i,2)= Optimal_path_one(q,2);
      Optimal_path_try(i,3)= Optimal_path_one(q,3);
      Optimal_path_try(i,4)= Optimal_path_one(q,4);
      
   end
 %%%%%%%%%%%%%%%     反过来排列路线节点      %%%%%%%%%%%%%%%%%%%%%

 n=size(Optimal_path_try,1);
   for i=1:1:n
       Optimal_path(i,1)=Optimal_path_try(n,3);
       Optimal_path(i,2)=Optimal_path_try(n,4);
%        Optimal_path(i,3)=Optimal_path_try(n,1);
%        Optimal_path(i,4)=Optimal_path_try(n,2);
       n=n-1;
   end
   num_op=size(Optimal_path,1)+1;
   Optimal_path(num_op,1)=Optimal_path_try(1,1);
   Optimal_path(num_op,2)=Optimal_path_try(1,2);
 %  plot(Optimal_path(:,1)+.5,Optimal_path(:,2)+.5,'linewidth',1); %5%绘线

 
 % 二次折线优化
 Optimal_path_two=Line_OPEN_STtwo(Optimal_path,CLOSED,Num_obs,num_op);
      % 使用 unique 函数删除重复的行
 [~,ia] = unique(Optimal_path_two(:,1:2), 'rows', 'stable');
  % 重建整个 n*4 矩阵，仅包含唯一的行
 Optimal_path_two = Optimal_path_two(ia, :);
 num_optwo=size(Optimal_path_two,1)+1;
 Optimal_path_two(num_optwo,1)=xStart;
 Optimal_path_two(num_optwo,2)=yStart;
 % plot(Optimal_path_two(:,1)+.5,Optimal_path_two(:,2)+.5,'linewidth',1); %5%绘线
 
 
 % 三次折线
  j=num_optwo;
 Optimal_path_two2=[xStart yStart];
 for i=1:1:num_optwo
     Optimal_path_two2(i,1)=Optimal_path_two(j,1);
     Optimal_path_two2(i,2)=Optimal_path_two(j,2);
     j=j-1;
 end
  Optimal_path_three=Line_OPEN_STtwo(Optimal_path_two2,CLOSED,Num_obs,num_optwo);
      % 使用 unique 函数删除重复的行
  [~,ia] = unique(Optimal_path_three(:,1:2), 'rows', 'stable');
   % 重建整个 n*4 矩阵，仅包含唯一的行
 Optimal_path_three = Optimal_path_three(ia, :);
  num_opthree=size(Optimal_path_three,1)+1;
  Optimal_path_three(num_opthree,1)=xTarget;
  Optimal_path_three(num_opthree,2)=yTarget;
 % plot(Optimal_path_three(:,1)+.5,Optimal_path_three(:,2)+.5,'b','linewidth',2);
  L_obst=2;% 每隔2个取点
  Obst_d_d_line=Line_obst(Optimal_path_three,L_obst);
      % 使用 unique 函数删除重复的行
  [~,ia] = unique(Obst_d_d_line(:,1:2), 'rows', 'stable');
   % 重建整个 n*4 矩阵，仅包含唯一的行
 Obst_d_d_line = Obst_d_d_line(ia, :);
  num_opthree=size(Obst_d_d_line,1)+1;
  Obst_d_d_line(num_opthree,1)=xTarget;
  Obst_d_d_line(num_opthree,2)=yTarget;
  
  NewOptimal_path=Obst_d_d_line;
        % 使用 unique 函数删除重复的行
  [~,ia] = unique(NewOptimal_path(:,1:2), 'rows', 'stable');
   % 重建整个 n*4 矩阵，仅包含唯一的行
 NewOptimal_path = NewOptimal_path(ia, :);
%  plot(NewOptimal_path(:,1)+.5,NewOptimal_path(:,2)+.5,'r','linewidth',2);
  
 
%  plot(NewOptimal_path(:,1)+.5,NewOptimal_path(:,2)+.5,'b','linewidth',2); %5%绘线 'b',,'b'
 
 
% Num_OPEN=size(OPEN,1) %%%% 遍历节点数
%  %Num_OPtimal=size(Optimal_path,1);  %%%% 实际路径节点数
%  Num_NewOptimal_path=size(NewOptimal_path,1);  %%%% 新的实际节点数
%  zhuan_num=Num_NewOptimal_path-2
  S=0;
 j = size(NewOptimal_path,1) ;
 for i=1:1:(j-1)  %%%% 求路径所用的实际长度
     Dist=sqrt( ( NewOptimal_path(i,1) - NewOptimal_path(i+1,1) )^2 + ( NewOptimal_path(i,2) - NewOptimal_path(i+1,2))^2);
     S=S+Dist;
 end
distanceX=S;
ia_size=size(NewOptimal_path,1);
Path=[ ];
for i=1:1:ia_size
   Path(i,1) = NewOptimal_path(i,1);
   Path(i,2) = NewOptimal_path(i,2);
end
h1=plot(Path(:,1)+0.5,Path(:,2)+0.5,' r ', 'linewidth', 2);
legend(h1,'IAFSA','Location','northwest');
fprintf('IAFSA行走长度为: %f\n',distanceX);
toc;
end
