function [d,intersections,angles,lidarx,lidary] = lidarSegmentIntersections(sensorRes,maxDistance,xObstacleSegments,yObstacleSegments)
%雷达线的交点

%% 计算角度分辨率
angularRes = 2*pi/sensorRes;
angles = (0:angularRes:(2*pi-angularRes))';
ZEROS = zeros(sensorRes,1);

%% 获取最大距离向量
d = maxDistance + angles.*0;

lidarx = [ZEROS,-sin(angles - pi/2).*d];
lidary = [ZEROS, cos(angles - pi/2).*d];

%% 找到难以察觉的交点
intersections = localLIDAR2SegsIntersect(lidarx,lidary,xObstacleSegments,yObstacleSegments);

%% 给定交点，调整距离
for i = 1:sensorRes
    intersection = intersections{i};
    if ~isempty(intersection)
        d(i) = norm(intersection);
    end
end

%% recompute the lidar segments
lidarx = [ZEROS,-sin(angles - pi/2).*d];
lidary = [ZEROS, cos(angles - pi/2).*d];

function intersections = localLIDAR2SegsIntersect(lidarx,lidary,segsx,segsy)
for i = size(lidarx,1):-1:1
    intersections{i} = localLine2SegsIntersect(lidarx(i,:),lidary(i,:),segsx,segsy);
end

function intersection = localLine2SegsIntersect(lineSeg1x,lineSeg1y,segsx,segsy)
intersection = [];
minNorm = inf;
for i = 1:size(segsx,1)
    intersection_ = localLine2LineIntersect(lineSeg1x,lineSeg1y,segsx(i,:),segsy(i,:));
    if ~isempty(intersection_)
        n = norm(intersection_);
        if n < minNorm
            intersection = intersection_;
            minNorm = n;
        end
    end
end

function intersection = localLine2LineIntersect(lineSeg1x,lineSeg1y,lineSeg2x,lineSeg2y)

intersection = [];

x1 = lineSeg1x(1);
x2 = lineSeg1x(2);
y1 = lineSeg1y(1);
y2 = lineSeg1y(2);

x3 = lineSeg2x(1);
x4 = lineSeg2x(2);
y3 = lineSeg2y(1);
y4 = lineSeg2y(2);

den = ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1));

uA = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3))/den;
uB = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3))/den;

U = [uA,uB];
if all(U >=0 & U <= 1)
    intersection(1) = x1 + (uA * (x2-x1));
    intersection(2) = y1 + (uA * (y2-y1));
end