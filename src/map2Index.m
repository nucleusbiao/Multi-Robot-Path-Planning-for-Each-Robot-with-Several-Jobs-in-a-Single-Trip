function [ indexMap ] = map2Index( map )
% ����0��1��ɵ�map�������
[n,m] = size(map);
indexMap = map;
for i = 1:n
    for j = 1:m
        indexMap(i,j) = (max(indexMap(:))+indexMap(i,j));
    end
end
indexMap = indexMap-1;
end

