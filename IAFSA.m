%%
% ���ڴ�ͳ�˹���Ⱥ�㷨����դ�񻷾��е��Ż�.
% �Ľ��������£�
% ����Ӧ��Ұ
% ����Ӧ�����������˹���Ⱥ��Ϊ�����Ӻ�����
% ����ʳ��Ϊ�м��뷽������
%%
% function [runDist,arrayValue] = IAFSA()
tic;
close all;
% clearvars -except result;
clear all;
clc;
%%
%���������˵��ϰ�����������עstart,goalλ��
% a = rand(20)>0.2;%�ڰ׸���ռ����
a = load('object3_2.txt');
% ��ʼ��������
count_zeros = 0;
count_ones = 0;

% ���������е�ÿ��Ԫ��
for i = 1:size(a, 1)
    for j = 1:size(a, 2)
        if a(i, j) == 0
            count_zeros = count_zeros + 1; % ���Ԫ����0������0�ļ���
        elseif a(i, j) == 1
            count_ones = count_ones + 1; % ���Ԫ����1������1�ļ���
        end
    end
end

n = size(a,1);
b = a;
b(end+1,end+1) = 0;
figure(1);
colormap([0 0 0;1 1 1])
pcolor(b); % ����դ����ɫ

set(gca,'XTick',10:10:n,'YTick',10:10:n);  % ��������
axis image xy

text(0.5,0.5,'START','Color','red','FontSize',10);%��ʾstart�ַ�
text(n+0.5,n+1.5,'GOAL','Color','red','FontSize',10);%��ʾgoal�ַ�

hold on;

% speed = 0.5;
% angle = -pi/2;
% [nrows, ncols] = size(a);
% for row = 1:nrows
%     for col = 1:ncols
%         if a(row, col) == 1  % ����õ��������
%             % ����һ�����µļ�ͷ�������'Vertices'��'Faces'�����˼�ͷ����״��λ��
%             quiver(col+0.5, row+0.5, speed*cos(angle), speed*sin(angle), 'MaxHeadSize', 10, 'Color', 'b','AutoScaleFactor',1); 
%         end
%     end
% end
% 
% 

%%
%�ϰ��ռ���󼯺�B,0Ϊ�ϰ���
Barrier = (find(a==0))';

%%
%����������

%%
%main
%%
% arrayValue = [13,34,256];
% %�жϽ��Ƿ����ϰ�����
% NB = length(arrayValue);
% for i = 1:1:NB
%     if ismember(arrayValue(i),Barrier) ==1
%         arrayValue(i) = randi([1 400],1); %ɾ���������ϰ��ﴦ������.������1-400��������� 
%     end
% end
N = 30;%�˹�������
try_number = 8;
MAXGEN = 100;%����������
visual = 10; %��ʼֵ
delta = 0.618;
start = 1;%�˹���Ⱥ��ʼλ��
DistMin = sqrt((1-n)^2+(n-1)^2);%��̾����־λ
goal = n*n;%Ŀ��λ��
shift = 1;%��ʳ��Ϊ��{Ⱥ�ۣ�׷β}��Ϊ��ģʽ�л���
shiftFreq = 4;%��ʳ��Ϊ��{Ⱥ�ۣ�׷β}��Ϊ���� ��Ƶ��
rightInf = sqrt(2);
% arrayValue = [20 39 58 77 96 115 134 153 172 192 212 232 252 272 291 310 
%                 329 328 327 326 325 324 344 364 363 362 361 381];%��ʼ��
arrayValue = [20];%����ȷ���Ҫ��ʾ��������.
for i =1:1:N
    ppValue(i) = start;%�����˹���Ⱥλ�õı���
end
position = ppValue;
H = GrideAF_foodconsistence(n,ppValue,goal);
[BH,BHindex] = min(H);
BX = ppValue(BHindex);
count = 1;%��¼ִ����ʳ��Ϊ�Ĵ���
runDist = 0;%��¼����·�����ܳ���
runDist_part = 0;%��¼ÿһ�����е�����·�����ȣ����ڱȽ�
BestH = zeros(1,MAXGEN);%��¼ÿһ�ε����е�����Hֵ 
index = [];%��¼�ҵ�·������Ⱥ
% endpath = 1;%��¼�Ż�������·���ĵڼ�����
% path_optimum = [];%��¼�Ż����·��
% Rf = 4*sqrt(2);%�Ż�������Ұ�뾶
% kcount = 0;%����Ѱ�ż���                                                  
% dupilcate = 1;%�Ż���������
%-----------------------------------------���˱���������ʼ��ȫ������------------------------------------------------------
for j = 1:1:MAXGEN
    switch shift
%------------------------------------��ʳ��Ϊ----------------------------------------------------------------------------
        case 1
            for i = 1:1:N
                if j == 1
                    [nextPosition,nextPositionH] = GridAF_prey(n,N,ppValue(i),position(j,:),i,try_number,H,Barrier,goal, j, MAXGEN,rightInf,BX,a);
                else
                    [nextPosition,nextPositionH] = GridAF_prey(n,N,ppValue(i),position(j-1,:),i,try_number,H,Barrier,goal, j, MAXGEN,rightInf,BX,a);
                end
                %��Ҫ��¼��ÿ�����Ӧ��λ�ã��Լ�ʳ��Ũ��.�Ա��´θ���.
                position(j,i) = nextPosition;%position���������Ⱥ��λ��.
                H(i) = nextPositionH;
            end
%             disp('prey!!!!')
%------------------------------------Ⱥ����Ϊ---------------------------------------------------------------------------    
         case 2     
            for i = 1:1:N
                [nextPosition_S,nextPositionH_S] = GridAF_swarm(n,N,position(j-1,:),i,visual,delta,try_number,H,Barrier,goal, j, MAXGEN,rightInf,BX,a);
                [nextPosition_F,nextPositionH_F] = GridAF_follow(n,N,position(j-1,:),i,visual,delta,try_number,H,Barrier,goal, j, MAXGEN,rightInf,BX,a);
                if nextPositionH_F < nextPositionH_S
                    nextPosition = nextPosition_F;
                    nextPositionH = nextPositionH_F;
                else
                    nextPosition = nextPosition_S;
                    nextPositionH = nextPositionH_S;
                end
                 position(j,i) = nextPosition;%position���������Ⱥ��λ��.
                 H(i) = nextPositionH;
            end 
%             disp('swarm & follow!!!')
    end
%-----------------------------------------------------------------------------------------------------------------
    count = count+1;%�����˹��㶼�����һ����ʳ��Ϊ
    if rem(count,shiftFreq) == 0 %��Ϊcount��1��ʼ�ǣ�����5ʱ������4����ʳ��Ϊ
        shift = 2;
    else
        shift = 1;
    end
    %Ҫ����ppValue��ֵ
    ppValue =  position(j,:);
    [BH,BHindex] = min(H);
    BX = ppValue(BHindex);
    %����position���ҵ��˹��㵽��goal��ʱ��������ѭ��
    index = find(position(j,:)==goal);
    if ~isempty(index)
        break;
    end

end

%%�������Ϊ������������������ѭ������˵��û��·������
if MAXGEN <= j
    disp('There is no way can arrive to the goal!!!');
else

%�����п���·�����ҳ����·��
    for i = 1:1:length(index)
        arrayValue =[start;position(:,index(i))]';
%----------------------���������·�����ܳ���-----------------------------
        for j = 1:1:length(arrayValue)-1
            d = distance(n,arrayValue(j),arrayValue(j+1));
            runDist_part = runDist_part + d;
        end
        transimit(i) = runDist_part;%��¼���п���·�����ܳ���   
        runDist_part = 0;
    end
    [runDist,runMin_index] = min(transimit(:));
    arrayValue = [start;position(:,index(runMin_index))]';
    fprintf('IGAFSA���߳���Ϊ: %f \n',runDist);
    
    for i =1:1:length(arrayValue)
        BestH(i) = goal-arrayValue(i);%��¼���ſ��н�ĵ���ͼ
    end

    DrawPath(n,arrayValue);%��������·��
    [row,col] = ind2sub(n,arrayValue);%դ���е���ֵת������������ֵ
    Optimal_path = [];
    [array_x,array_y] = arry2orxy(n,row,col);%�������±�ת��Ϊ������xy��ʽ
    Optimal_path = [array_x',array_y'];
    % ʹ�� unique ����ɾ���ظ�����
    [Optimal_path] = unique(Optimal_path, 'rows', 'stable');
    IAFSA_path = Optimal_path;
    % 
    % ��·�����з�ת�����ϳ�n*4�ṹ
    Optimal_path = flipud(Optimal_path);
    for i = 1:size(Optimal_path,1)
        if i == size(Optimal_path,1)
            Optimal_path(i,:) = [];
        else
            Optimal_path(i,3) = Optimal_path(i+1,1);
            Optimal_path(i,4) = Optimal_path(i+1,2);
        end
    end

end
% h1=plot(IAFSA_path(:,1)+0.5,IAFSA_path(:,2)+0.5,' r ', 'linewidth', 2);
% 
% [Path,distanceX] = Smooth(a,Optimal_path,1);
% h2 = plot(Path(:,1)+0.5,Path(:,2)+0.5,'b','LineWidth',2);
% fprintf('IGAFSA+Smooth���߳���Ϊ: %f \n',distanceX);

% [newPath,distanceB] = bezierSmooth(Path);
% h3 = plot(newPath(:,1)+0.5, newPath(:,2)+0.5,'g','LineWidth',2);
% fprintf('IGAFSA+Smooth+bezier���߳���Ϊ: %f \n',distanceB);
% legend([h1,h2,h3],'IGAFSA','IGAFSA1','IGAFSA2','Location','northwest');


% figure;
% BestH = BestH';
% n1 = plot(BestH,'r','LineWidth',2);
% hold on;
% save('D:\Code-for-IAFSA\IAFSA\IGAFSA.mat','Path','newPath');
% load('D:\Code-for-IAFSA\IAFSA\IGAFSA.mat','newPath');
time = toc;
fprintf('��������ʱ�䣺 %.2f ��\n', time);
% fprintf('IAFSA���߳���Ϊ: %f\n',runDist); % �������ʱ��
% PS:��ֵ�����0.5��Ϊ�˱�֤�������������դ�����Ĵ�


