%% Split and Merge for line 

%% small home 용 이미지 파일 load
load('SplitNMergeVariable.mat');

global stats
cc = 4;
threshold_dist = 10;


%% 1. initializing Starting point 이용해서 초기선분 잡기

a = global_max_point.location(cc,1);
b = global_max_point.location(cc,2);
c = global_max_point.location(cc,3);
d = global_max_point.location(cc,4);

A = (d-b)/(c-a);
B = b - A*a;

x = stats(cc).PixelList(:,1);
y = stats(cc).PixelList(:,2);

[stats(cc).LineList.Line] = [A B];
[stats(cc).LineList.EndPoint] = [a b c d];


%% 2. if 가장먼거리점 거리> threshold이상 일경우 split

stats(cc).PixelList(:,3) = zeros(length(stats(cc).PixelList(1)),1);
stats(cc).PixelList(:,4) = zeros(length(stats(cc).PixelList(1)),1);
for i=1 : length(stats(cc).PixelList)
    stats(cc).PixelList(i,3) = abs(B+A*stats(cc).PixelList(i,1)-stats(cc).PixelList(i,2))/sqrt(1+A*A);
end
[max_dist,max_point_index] = max(stats(cc).PixelList(:,3));

if max_dist > threshold_dist
    stats.(cc) = SplitLine(max_point_index);
end

%% debugging 용 이미지제작용

scatter(x,y,'filled'); grid on; hold on
plot(x,B+A*x,'r','LineWidth',2);

r = 4;
xc = stats(cc).PixelList(max_point_index,1);
yc = stats(cc).PixelList(max_point_index,2);

theta = linspace(0,2*pi);
x = r*cos(theta) + xc;
y = r*sin(theta) + yc;
plot(x,y)
axis equal
