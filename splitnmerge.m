%% Split and Merge for line 

%% small home 용 이미지 파일 load
load('SplitNMergeVariable.mat');

global stats
cc = 2;
threshold_dist = 10;

%% Initializing, 각 connected component 별 point, start&end point, line 할당 

for i=1 : length(stats)
    stats(i).Line.PixelList = stats(i).PixelList;

    a = global_max_point.location(cc,1);
    b = global_max_point.location(cc,2);
    c = global_max_point.location(cc,3);
    d = global_max_point.location(cc,4);
    
    if c==a
        A = 0; 
    else
        A = (d-b)/(c-a);
    end
    B = b - A*a;
    [stats(i).Line.LineList.Line] = [A B];
    [stats(i).Line.LineList.EndPoint] = [a b c d];
    
    stats(i).Line.PixelList(:,3) = zeros(length(stats(i).Line.PixelList(1)),1);
    stats(i).Line.PixelList(:,4) = zeros(length(stats(i).Line.PixelList(1)),1);
end


%% 2. if 가장먼거리점 거리> threshold이상 일경우 split

for i=1 : length(stats(cc).Line.PixelList)
    stats(cc).Line.PixelList(i,3) = abs(B+A*stats(cc).Line.PixelList(i,1)-stats(cc).Line.PixelList(i,2))/sqrt(1+A*A);
end
[max_dist,max_point_index] = max(stats(cc).Line.PixelList(:,3));

if max_dist > threshold_dist
    SplitLine(max_point_index);
end

%% debugging 용 이미지제작용

% scatter(x,y,'filled'); grid on; hold on
% plot(x,B+A*x,'r','LineWidth',2);
% 
% r = 4;
% xc = stats(cc).PixelList(max_point_index,1);
% yc = stats(cc).PixelList(max_point_index,2);
% 
% theta = linspace(0,2*pi);
% x = r*cos(theta) + xc;
% y = r*sin(theta) + yc;
% plot(x,y)
% axis equal
