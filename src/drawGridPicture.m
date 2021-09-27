function [ ] = drawGridPicture( map )
% 显示栅格图
[n,m] = size(map);
b = map;
b(end+1,end+1) = 0;
pcolor(b); % 赋予栅格颜色
% set(gca,'XTick',10:10:n,'YTick',10:10:m);  % 设置坐标
axis image
axis ij
 
% indexMap = map2Index( map );
% indexMap(find(map==0)) = 0;
% %将数值在栅格图上显示出来
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