function [ subscript ] = index2Subscript( indexMap, Index )
% ��������±�
[x,y] = ind2sub(size(indexMap),find(indexMap==Index));
subscriptArray = [x,y];
subscriptArray = neatenVertexColumn( subscriptArray );
subscript = subscriptArray(1,:);
end