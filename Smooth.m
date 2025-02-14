function [Path,distanceX] = Smooth(a,Optimal_path,D)
%% 预备变量
k=1;
CLOSED=[];
MAX = 1-a;
MAX_X=size(MAX,2);                                %%%  获取列数，即x轴长度
MAX_Y=size(MAX,1);                                %%%  获取行数，即y轴长度
for j=1:MAX_X    %列遍历
    for i=1:MAX_Y   %行遍历
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

xStart = Optimal_path(end,3);      yStart = Optimal_path(end,4);
xTarget = Optimal_path(1,1);    yTarget = Optimal_path(1,2);

%% 路径优化
Num_obs=size(Obs_Closed,1);
CLOSED = Obs_Closed;
Num_Opt=size(Optimal_path,1);

%%%  优化折线
Optimal_path_one=Line_OPEN_ST(Optimal_path,CLOSED,Num_obs,Num_Opt,D);
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
% plot(Optimal_path(:,1)+.5,Optimal_path(:,2)+.5,'linewidth',1); %5%绘线


% 二次折线优化
Optimal_path_two=Line_OPEN_STtwo(Optimal_path,CLOSED,Num_obs,num_op,D);
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
Optimal_path_three=Line_OPEN_STtwo(Optimal_path_two2,CLOSED,Num_obs,num_optwo,D);

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
plot(NewOptimal_path(:,1)+.5,NewOptimal_path(:,2)+.5,'r','linewidth',2);


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

% waypoint_x = Path(:,1);
% waypoint_y = Path(:,2);
% waypoint_num = length(waypoint_x);
% waypoint_yaw = [];
% for i = 1:waypoint_num-1
%     angle = atan((waypoint_y(i+1)-waypoint_y(i))/(waypoint_x(i+1)-waypoint_x(i)));
%     waypoint_yaw = [waypoint_yaw;angle];
% end