function [smoothPath,distanceX] = bezierSmooth(Path)
    newPath = [];
    index = [];
    smoothPath = [];
    % 遍历每一对相邻点,记录转折点
    for i = 1:size(Path,1)-2
        x_n1 = Path(i+1,1) - Path(i,1) ;  %%向量1
        y_n1 = Path(i+1,2) - Path(i,2) ;
        x_n2 = Path(i+2,1) - Path(i+1,1) ;   %%向量2
        y_n2 = Path(i+2,2) - Path(i+1,2) ;
        angle=myangle(x_n1,y_n1,x_n2,y_n2); %%%判断两个向量的夹角
        if angle > 0.01
            index = [index;i+1];
        end
    end
    newPath = [newPath;Path(1:index(1)-1,:)];
    for i = 1:length(index)-1
        newPath = [newPath;Path(index(i):index(i+1)-1,:)];
        if index(i)+1 == index(i+1)
            numPoints = 1;
            t = 1 / (numPoints + 1);
            interpolatedPoint = (1 - t) * Path(index(i),:) + t * Path(index(i+1),:);
            newPath = [newPath;interpolatedPoint];
        end
    end
    newPath = [newPath;Path(index(end):end,:)];
    
    index = [];
    % 遍历每一对相邻点,记录转折点
    for i = 1:size(newPath,1)-2
        x_n1 = newPath(i+1,1) - newPath(i,1) ;  %%向量1
        y_n1 = newPath(i+1,2) - newPath(i,2) ;
        x_n2 = newPath(i+2,1) - newPath(i+1,1) ;   %%向量2
        y_n2 = newPath(i+2,2) - newPath(i+1,2) ;
        angle=myangle(x_n1,y_n1,x_n2,y_n2); %%%判断两个向量的夹角
        if angle > 0.01
            index = [index;i+1];
        end
    end

    smoothPath = [smoothPath;newPath(1:(index(1)-1),:)];
    for i = 1:size(index)
        ControlPoint = [];
        numPoints = 2;
        % 生成在两点间线性插值的点
        for j = 1:numPoints
            % 计算插值点
            t = j / (numPoints + 1);
            interpolatedPoint = (1 - t) * newPath(index(i)-1,:) + t * newPath(index(i),:);
            ControlPoint = [ControlPoint; interpolatedPoint];
        end

        for j = 1:numPoints
            % 计算插值点
            t = j / (numPoints + 1);
            interpolatedPoint = (1 - t) * newPath(index(i),:) + t * newPath(index(i)+1,:);
            ControlPoint = [ControlPoint; interpolatedPoint];
        end

        P0 = ControlPoint(1, :);
        P1 = ControlPoint(2, :);
        P2 = ControlPoint(3, :);
        P3 = ControlPoint(4, :);
        t = linspace(0, 1, 100);
        BezierPoints = zeros(length(t), 2);
        for j = 1:length(t)
            BezierPoints(j, :) = (1-t(j)).^3 .* P0 + 3*(1-t(j)).^2 .* t(j) .* P1 + 3*(1-t(j)) .* t(j).^2 .* P2 + t(j).^3 .* P3;
        end
        smoothPath = [smoothPath; BezierPoints];


        if i < length(index)
            smoothPath = [smoothPath;newPath(index(i)+1:(index(i+1)-1),:)];
        else
            smoothPath = [smoothPath;newPath(index(i)+1:end,:)];
        end
    end
    S=0;
    j = size(smoothPath,1) ;
    for i=1:1:(j-1)  %%%% 求路径所用的实际长度
        Dist=sqrt( ( smoothPath(i,1) - smoothPath(i+1,1) )^2 + ( smoothPath(i,2) - smoothPath(i+1,2))^2);
        S=S+Dist;
    end
    distanceX=S;
end
