function stats = SplitLine(max_point_index)
%% Recursively Split line 

%% ㄱ. Initializing

% load('stats.mat');

global stats
cc = 4

%% ㄴ. line AC, CB split

% 점 둘로 나누기
stats(cc).LineList.EndPoint(2, 1:2) = stats(cc).LineList.EndPoint(1, 3:4);
stats(cc).LineList.EndPoint(1, 3:4) = stats(cc).PixelList(max_point_index,1:2);
stats(cc).LineList.EndPoint(2, 3:4) = stats(cc).PixelList(max_point_index,1:2);


% 첫번째 선분 기울기 계산
a = stats(cc).LineList.EndPoint(1,1);
b = stats(cc).LineList.EndPoint(1,2);
c = stats(cc).LineList.EndPoint(1,3);
d = stats(cc).LineList.EndPoint(1,4);

A = (d-b)/(c-a);
B = b - A*a;

stats(cc).LineList.Line(1,:) = [A B];

x = stats(cc).PixelList(:,1);
y = stats(cc).PixelList(:,2);
scatter(x,y,'filled'); grid on; hold on
plot(x,B+A*x,'r','LineWidth',2);

% 두번째 선분 기울기 계산
a = stats(cc).LineList.EndPoint(2,1);
b = stats(cc).LineList.EndPoint(2,2);
c = stats(cc).LineList.EndPoint(2,3);
d = stats(cc).LineList.EndPoint(2,4);

A = (d-b)/(c-a);
B = b - A*a;

stats(cc).LineList.Line(2,:) = [A B];

plot(x,B+A*x,'r','LineWidth',2);

%% ㄷ. 각각 point에서 새로운 선분까지 거리재서 가까운곳에 point 할당

% pixel들 line에 대응시키기전 임시로 할당할 배열 선언
statsA = [];
statsB = [];

for i=1 : length(stats(cc).PixelList)
    stats(cc).PixelList(1,3) = abs(stats(cc).LineList.Line(1,2)+stats(cc).LineList.Line(1,1)*stats(cc).PixelList(1,1)-stats(cc).PixelList(1,2))/sqrt(1+stats(cc).LineList.Line(1,1)*stats(cc).LineList.Line(1,1));
    stats(cc).PixelList(1,4) = abs(stats(cc).LineList.Line(2,2)+stats(cc).LineList.Line(2,1)*stats(cc).PixelList(1,1)-stats(cc).PixelList(1,2))/sqrt(1+stats(cc).LineList.Line(2,1)*stats(cc).LineList.Line(2,1));
    [~,min_line_index] = min(stats(cc).PixelList(1,3:4));
    if min_line_index == 1      % 첫번째 라인에 할당된 경우
        statsA = [statsA; stats(cc).PixelList(1,1:2)];
    else
        statsB = [statsB; stats(cc).PixelList(1,1:2)];
    end
    stats(cc).PixelList(1,:) = [];
end

% stats 새로운 point 추가

stats(cc).PixelList = statsA;
stats(length(stats)+1).PixelList = statsB;

% line list 갱신
stats(length(stats)).LineList.Line(1,:) = stats(cc).LineList.Line(2,:);
stats(cc).LineList.Line(2,:) = [];
stats(length(stats)).LineList.EndPoint(1,:) = stats(cc).LineList.EndPoint(2,:);
stats(cc).LineList.EndPoint(2,:) = [];

[max_dist,max_point_index] = max(stats(cc).PixelList(:,3));
