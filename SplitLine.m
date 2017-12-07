function stats = SplitLine(cc, current_line, max_point_index)
% SplitLine(4, 1, 1716)
% SplitLine(2, 1, 823)
%% Recursively Split line 

%% ㄱ. Initializing

% load('stats.mat');

global stats
threshold_dist = 10;

%% ㄴ. line AC, CB split

% 점 둘로 나누기
stats(cc).Line(current_line).LineList.EndPoint(2, 1:2) = stats(cc).Line(current_line).LineList.EndPoint(1, 3:4);
stats(cc).Line(current_line).LineList.EndPoint(1, 3:4) = stats(cc).Line(current_line).PixelList(max_point_index,1:2);
stats(cc).Line(current_line).LineList.EndPoint(2, 3:4) = stats(cc).Line(current_line).PixelList(max_point_index,1:2);


% 첫번째 선분 기울기 계산
a = stats(cc).Line(current_line).LineList.EndPoint(1,1);
b = stats(cc).Line(current_line).LineList.EndPoint(1,2);
c = stats(cc).Line(current_line).LineList.EndPoint(1,3);
d = stats(cc).Line(current_line).LineList.EndPoint(1,4);

if c==a
	A = 0; 
else
    A = (d-b)/(c-a);
end
B = b - A*a;

stats(cc).Line(current_line).LineList.Line(1,:) = [A B];


% 디버깅용
x = stats(cc).PixelList(:,1);
y = stats(cc).PixelList(:,2);
scatter(x,y,'filled'); grid on; hold on
plot(x,B+A*x,'r','LineWidth',2);

% 두번째 선분 기울기 계산
a = stats(cc).Line(current_line).LineList.EndPoint(2,1);
b = stats(cc).Line(current_line).LineList.EndPoint(2,2);
c = stats(cc).Line(current_line).LineList.EndPoint(2,3);
d = stats(cc).Line(current_line).LineList.EndPoint(2,4);

if c==a
	A = 0; 
else
    A = (d-b)/(c-a);
end
B = b - A*a;

stats(cc).Line(current_line).LineList.Line(2,:) = [A B];

plot(x,B+A*x,'r','LineWidth',2);

%% ㄷ. 각각 point에서 새로운 선분까지 거리재서 가까운곳에 point 할당

% pixel들 line에 대응시키기전 임시로 할당할 배열 선언
statsA = [];
statsB = [];

for i = 1 : length(stats(cc).Line(current_line).PixelList)
    A1 = stats(cc).Line(current_line).LineList.Line(1,1);
    B1 = stats(cc).Line(current_line).LineList.Line(1,2);
    A2 = stats(cc).Line(current_line).LineList.Line(2,1);
    B2 = stats(cc).Line(current_line).LineList.Line(2,2);
    x = stats(cc).Line(current_line).PixelList(1,1);
    y = stats(cc).Line(current_line).PixelList(1,2);
    stats(cc).Line(current_line).PixelList(1,3) = abs(B1+A1*x-y)/sqrt(1+A1*A1);
    stats(cc).Line(current_line).PixelList(1,4) = abs(B2+A2*x-y)/sqrt(1+A2*A2);
    [~,min_line_index] = min(stats(cc).Line(current_line).PixelList(1,3:4));
    if min_line_index == 1      % 첫번째 라인에 할당된 경우
        statsA = [statsA; stats(cc).Line(current_line).PixelList(1,1:2)];
    else                        % 두번째 라인에 할당된 경우
        statsB = [statsB; stats(cc).Line(current_line).PixelList(1,1:2)];
    end
    stats(cc).Line(current_line).PixelList(1,:) = [];
end

% stats 새로운 point 추가

new_line = length(stats(cc).Line)+1;

stats(cc).Line(current_line).PixelList = statsA;
stats(cc).Line(current_line).PixelList(:,3) = zeros(length(stats(cc).Line(current_line).PixelList(1)),1);
stats(cc).Line(current_line).PixelList(:,4) = zeros(length(stats(cc).Line(current_line).PixelList(1)),1);
stats(cc).Line(new_line).PixelList = statsB;
stats(cc).Line(new_line).PixelList(:,3) = zeros(length(stats(cc).Line(new_line).PixelList(1)),1);
stats(cc).Line(new_line).PixelList(:,4) = zeros(length(stats(cc).Line(new_line).PixelList(1)),1);

% line list 갱신
stats(cc).Line(new_line).LineList.Line(1,:) = stats(cc).Line(current_line).LineList.Line(2,:);
stats(cc).Line(current_line).LineList.Line(2,:) = [];
stats(cc).Line(new_line).LineList.EndPoint(1,:) = stats(cc).Line(current_line).LineList.EndPoint(2,:);
stats(cc).Line(current_line).LineList.EndPoint(2,:) = [];

% if 가장먼거리점 거리> threshold이상 일경우 recursive split
A = stats(cc).Line(current_line).LineList.Line(1,1);
B = stats(cc).Line(current_line).LineList.Line(1,2);
for i=1 : length(stats(cc).Line(current_line).PixelList)
    x = stats(cc).Line(current_line).PixelList(i,1);
    y = stats(cc).Line(current_line).PixelList(i,2);
    stats(cc).Line(current_line).PixelList(i,3) = abs(B+A*x-y)/sqrt(1+A*A);
end
[max_dist,max_point_index] = max(stats(cc).Line(current_line).PixelList(:,3));
if max_dist > threshold_dist
    SplitLine(cc, current_line, max_point_index);
end

A = stats(cc).Line(new_line).LineList.Line(1,1);
B = stats(cc).Line(new_line).LineList.Line(1,2);
for i=1 : length(stats(cc).Line(new_line).PixelList)
    x = stats(cc).Line(new_line).PixelList(i,1);
    y = stats(cc).Line(new_line).PixelList(i,2);
    stats(cc).Line(new_line).PixelList(i,3) = abs(B+A*x-y)/sqrt(1+A*A);
end
[max_dist,max_point_index] = max(stats(cc).Line(new_line).PixelList(:,3));
if max_dist > threshold_dist
    SplitLine(cc, new_line, max_point_index);
end

end