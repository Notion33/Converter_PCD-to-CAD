function SplitLine(cc, current_line, max_point_index)
% SplitLine(4, 1, 1716)
% SplitLine(4, 2, 1566)
%% Recursively Split line 

%% ��. Initializing
global stats threshold_dist

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

% % ������
% x = stats(cc).PixelList(:,1);
% y = stats(cc).PixelList(:,2);
% scatter(x,y,'filled'); grid on; hold on
% plot(x,B+A*x,'r','LineWidth',2);

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

% ���λ��� �ι�° ���� ����,y����, end point ���� �Է�
new_line = length(stats(cc).Line)+1;
stats(cc).Line(new_line).LineList.Line(1,:) = [A B];
stats(cc).Line(new_line).LineList.EndPoint(1,:) = [a b c d];
stats(cc).Line(current_line).LineList.EndPoint(2,:) = [];

% plot(x,B+A*x,'r','LineWidth',2);

%% ��. ���� point���� ���ο� ���б��� �Ÿ��缭 �������� point �Ҵ�

% ������ point�� line �Ÿ������� ���� �Ҵ�
stats(cc).PixelList(:,1+2:new_line+2) = zeros(length(stats(cc).PixelList),new_line);

% ��� line�� ���� ������ point�� �Ÿ� ����
for i = 1 : length(stats(cc).Line)
	A = stats(cc).Line(i).LineList.Line(1,1);
	B = stats(cc).Line(i).LineList.Line(1,2);
    for j = 1 : length(stats(cc).PixelList)
        x = stats(cc).PixelList(j,1);
        y = stats(cc).PixelList(j,2);
        stats(cc).PixelList(j,i+2) = abs(B+A*x-y)/sqrt(1+A*A);
    end
end

% pixel�� line�� ������Ű���� �ӽ÷� �Ҵ��� 3���� �迭 ����
temp_stats = [];
temp_stats = struct('PixelList',cell(new_line, 1));

% ���� ����� line���� ���� ���Ҵ�
for i = 1 : length(stats(cc).PixelList)
        [~,min_line_index] = min(stats(cc).PixelList(i,1+2:length(stats(cc).Line)+2));
        temp_stats(min_line_index).PixelList = cat(1,temp_stats(min_line_index).PixelList , [stats(cc).PixelList(i,1:2) stats(cc).PixelList(i, min_line_index+2)]);
end

% stats ���ο� point �߰�
for i = 1 : length(stats(cc).Line)
    stats(cc).Line(i).PixelList = temp_stats(i).PixelList;
end

% ������ �̹���
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

% if ����հŸ��� �Ÿ�> threshold�̻� �ϰ�� recursive split
for i = 1 : length(stats(cc).Line)
    [max_dist,max_point_index] = max(stats(cc).Line(i).PixelList(:,3));
    if max_dist > threshold_dist
        SplitLine(cc, i, max_point_index);
    end
end

end