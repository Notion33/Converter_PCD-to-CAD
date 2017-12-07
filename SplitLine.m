function stats = SplitLine(cc, current_line, max_point_index)
% SplitLine(4, 1, 1716)
% SplitLine(2, 1, 823)
%% Recursively Split line 

%% ��. Initializing

% load('stats.mat');

global stats
threshold_dist = 10;

%% ��. line AC, CB split

% �� �ѷ� ������
stats(cc).Line(current_line).LineList.EndPoint(2, 1:2) = stats(cc).Line(current_line).LineList.EndPoint(1, 3:4);
stats(cc).Line(current_line).LineList.EndPoint(1, 3:4) = stats(cc).Line(current_line).PixelList(max_point_index,1:2);
stats(cc).Line(current_line).LineList.EndPoint(2, 3:4) = stats(cc).Line(current_line).PixelList(max_point_index,1:2);


% ù��° ���� ���� ���
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


% ������
x = stats(cc).PixelList(:,1);
y = stats(cc).PixelList(:,2);
scatter(x,y,'filled'); grid on; hold on
plot(x,B+A*x,'r','LineWidth',2);

% �ι�° ���� ���� ���
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

%% ��. ���� point���� ���ο� ���б��� �Ÿ��缭 �������� point �Ҵ�

% pixel�� line�� ������Ű���� �ӽ÷� �Ҵ��� �迭 ����
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
    if min_line_index == 1      % ù��° ���ο� �Ҵ�� ���
        statsA = [statsA; stats(cc).Line(current_line).PixelList(1,1:2)];
    else                        % �ι�° ���ο� �Ҵ�� ���
        statsB = [statsB; stats(cc).Line(current_line).PixelList(1,1:2)];
    end
    stats(cc).Line(current_line).PixelList(1,:) = [];
end

% stats ���ο� point �߰�

new_line = length(stats(cc).Line)+1;

stats(cc).Line(current_line).PixelList = statsA;
stats(cc).Line(current_line).PixelList(:,3) = zeros(length(stats(cc).Line(current_line).PixelList(1)),1);
stats(cc).Line(current_line).PixelList(:,4) = zeros(length(stats(cc).Line(current_line).PixelList(1)),1);
stats(cc).Line(new_line).PixelList = statsB;
stats(cc).Line(new_line).PixelList(:,3) = zeros(length(stats(cc).Line(new_line).PixelList(1)),1);
stats(cc).Line(new_line).PixelList(:,4) = zeros(length(stats(cc).Line(new_line).PixelList(1)),1);

% line list ����
stats(cc).Line(new_line).LineList.Line(1,:) = stats(cc).Line(current_line).LineList.Line(2,:);
stats(cc).Line(current_line).LineList.Line(2,:) = [];
stats(cc).Line(new_line).LineList.EndPoint(1,:) = stats(cc).Line(current_line).LineList.EndPoint(2,:);
stats(cc).Line(current_line).LineList.EndPoint(2,:) = [];

% if ����հŸ��� �Ÿ�> threshold�̻� �ϰ�� recursive split
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