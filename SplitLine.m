function SplitLine(cc, current_line, max_point_index)
% SplitLine(4, 1, 1716)
% SplitLine(4, 2, 1566)
%% Recursively Split line 

%% ��. Initializing
global stats threshold_dist L

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

connected_temp_image = L==cc;

statsR = stats(cc).Line(current_line).LineList.EndPoint(1);
statsC = stats(cc).Line(current_line).LineList.EndPoint(2);
D11 = bwdistgeodesic(connected_temp_image, statsR,statsC);

statsR = stats(cc).Line(current_line).LineList.EndPoint(3);
statsC = stats(cc).Line(current_line).LineList.EndPoint(4);
D12 = bwdistgeodesic(connected_temp_image, statsR,statsC);

D1 = D11 + D12;
D1 = round(D1 * 8) / 8;

D1(isnan(D1)) = inf;
paths = imregionalmin(D1);
% figure(1)
% imshow(paths)

Path_Distance_1 = bwdistgeodesic(connected_temp_image, paths);

% figure(2)
% imshow(connected_temp_image)
% figure(3)
% imshow(Path_Distance_1)


statsR = stats(cc).Line(new_line).LineList.EndPoint(1);
statsC = stats(cc).Line(new_line).LineList.EndPoint(2);
D11 = bwdistgeodesic(connected_temp_image, statsR,statsC);

statsR = stats(cc).Line(new_line).LineList.EndPoint(3);
statsC = stats(cc).Line(new_line).LineList.EndPoint(4);
D12 = bwdistgeodesic(connected_temp_image, statsR,statsC);

D2 = D11 + D12;
D2 = round(D2 * 8) / 8;

D2(isnan(D2)) = inf;
paths = imregionalmin(D2);
% figure(2)
% imshow(paths)
Path_Distance_2 = bwdistgeodesic(connected_temp_image, paths);

temp1 = [];
temp2 = [];

% point�� ���� ���� �ִܰŸ� path�� �Ҵ�

for i = 1 : length(Path_Distance_1(:,1))
    for j = 1 : length(Path_Distance_1(1,:))
        if Path_Distance_1(i,j) < 5 && ~isnan(Path_Distance_1(i,j))
            temp1 = [temp1(:,:) ; j i];
        end
        if Path_Distance_2(i,j) < 5 && ~isnan(Path_Distance_1(i,j))
            temp2 = [temp2(:,:) ; j i];
        end
    end
end

stats(cc).Line(current_line).PixelList = temp1;
stats(cc).Line(new_line).PixelList = temp2;

% ������ �̹���
% x = stats(cc).PixelList(:,1);
% y = stats(cc).PixelList(:,2);
% scatter(x,y,'filled'); grid on; hold on
% for i=1 : length(stats(cc).Line)
%    x = [stats(cc).Line(i).LineList.EndPoint(1) stats(cc).Line(i).LineList.EndPoint(3)];
%    y = [stats(cc).Line(i).LineList.EndPoint(2) stats(cc).Line(i).LineList.EndPoint(4)];
%    line('XData',x,'YData',y)
%    hold on
% end
% hold off

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