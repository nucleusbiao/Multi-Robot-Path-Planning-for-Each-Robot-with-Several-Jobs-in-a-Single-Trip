function [ ] = drawGridPicture( map )
% ��ʾդ��ͼ
[n,m] = size(map);
b = map;
b(end+1,end+1) = 0;
pcolor(b); % ����դ����ɫ
% set(gca,'XTick',10:10:n,'YTick',10:10:m);  % ��������
axis image
axis ij
 
% indexMap = map2Index( map );
% indexMap(find(map==0)) = 0;
% %����ֵ��դ��ͼ����ʾ����
% for i = 1:1:n*m
%     [row,col] = ind2sub(n,i);
%     k = length(row);
%     for j = 1:1:k
%         array_x(j) = col(j);
%         array_y(j) = row(j);
%     end
%     text(array_x+0.2,array_y+0.5,num2str(indexMap(i)));
% end
end