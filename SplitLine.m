function SplitLine(cc, current_line, max_point_index)
% SplitLine(4, 1, 1716)
% SplitLine(4, 2, 1566)
%% Recursively Split line 

%% ㄱ. Initializing
global stats threshold_dist

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

% % 디버깅용
% x = stats(cc).PixelList(:,1);
% y = stats(cc).PixelList(:,2);
% scatter(x,y,'filled'); grid on; hold on
% plot(x,B+A*x,'r','LineWidth',2);

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

% 새로생긴 두번째 선분 기울기,y절편, end point 정보 입력
new_line = length(stats(cc).Line)+1;
stats(cc).Line(new_line).LineList.Line(1,:) = [A B];
stats(cc).Line(new_line).LineList.EndPoint(1,:) = [a b c d];
stats(cc).Line(current_line).LineList.EndPoint(2,:) = [];

% plot(x,B+A*x,'r','LineWidth',2);

%% ㄷ. 각각 point에서 새로운 선분까지 거리재서 가까운곳에 point 할당

% 각각의 point와 line 거리측정용 공간 할당
stats(cc).PixelList(:,1+2:new_line+2) = zeros(length(stats(cc).PixelList),new_line);

% 모든 line에 대해 각각의 point의 거리 측정
for i = 1 : length(stats(cc).Line)
	A = stats(cc).Line(i).LineList.Line(1,1);
	B = stats(cc).Line(i).LineList.Line(1,2);
    for j = 1 : length(stats(cc).PixelList)
        x = stats(cc).PixelList(j,1);
        y = stats(cc).PixelList(j,2);
        stats(cc).PixelList(j,i+2) = abs(B+A*x-y)/sqrt(1+A*A);
    end
end

% pixel들 line에 대응시키기전 임시로 할당할 3차원 배열 선언
temp_stats = [];
temp_stats = struct('PixelList',cell(new_line, 1));

% 가장 가까운 line으로 점들 재할당
for i = 1 : length(stats(cc).PixelList)
        [~,min_line_index] = min(stats(cc).PixelList(i,1+2:length(stats(cc).Line)+2));
        temp_stats(min_line_index).PixelList = cat(1,temp_stats(min_line_index).PixelList , [stats(cc).PixelList(i,1:2) stats(cc).PixelList(i, min_line_index+2)]);
end

% stats 새로운 point 추가
for i = 1 : length(stats(cc).Line)
    stats(cc).Line(i).PixelList = temp_stats(i).PixelList;
end

% 디버깅용 이미지
x = stats(cc).PixelList(:,1);
y = stats(cc).PixelList(:,2);
scatter(x,y,'filled'); grid on; hold on
for i=1 : length(stats(cc).Line)
   x = [stats(cc).Line(i).LineList.EndPoint(1) stats(cc).Line(i).LineList.EndPoint(3)];
   y = [stats(cc).Line(i).LineList.EndPoint(2) stats(cc).Line(i).LineList.EndPoint(4)];
   line('XData',x,'YData',y)
   hold on
end
hold off

% if 가장먼거리점 거리> threshold이상 일경우 recursive split
for i = 1 : length(stats(cc).Line)
    [max_dist,max_point_index] = max(stats(cc).Line(i).PixelList(:,3));
    if max_dist > threshold_dist
        SplitLine(cc, i, max_point_index);
    end
end

end