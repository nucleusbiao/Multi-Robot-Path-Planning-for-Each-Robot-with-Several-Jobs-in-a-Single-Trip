function [ vertexConnected, nodes_num ] = grid2VertexConnect( gridMatrix )
% 将栅格图形式的数据，转换为点连接的数据
% gridMatrix中0代表可以走的路径，1代表block
vertexConnected = [];
gridRow = size(gridMatrix,1);
gridColumn = size(gridMatrix,2);
nodes_num = gridRow*gridColumn; % 顶点的数量
for i = 1:gridRow-1
    for j = 1:gridColumn-1
        if (gridMatrix(i,j)==1) && (gridMatrix(i,j+1)==1)
            vertexConnected(end+1,1:2) = (i-1)*gridColumn+[j,j+1];
        end
        if (gridMatrix(i,j)==1) && (gridMatrix(i+1,j)==1)
            vertexConnected(end+1,1:2) = (i-1)*gridColumn+[j,j+gridColumn];
        end
    end
end
for i = 1:gridRow-1
    if (gridMatrix(i,end)==1) && (gridMatrix(i+1,end)==1)
        vertexConnected(end+1,1:2) = i*gridColumn+[0,gridColumn];
    end
end
for i = 1:gridColumn-1
    if (gridMatrix(end,i)==1) && (gridMatrix(end,i+1)==1)
        vertexConnected(end+1,1:2) = (gridRow-1)*gridColumn+[i,i+1];
    end
end
blockPosition = find(gridMatrix'==0);
nodes_num = nodes_num-size(blockPosition,1);
for i = size(blockPosition,1):-1:1
    vertexConnected(find(vertexConnected>blockPosition(i))) =  vertexConnected(find(vertexConnected>blockPosition(i)))-1;
end
vertexConnected = neatenVertexRow( vertexConnected );
vertexConnected = neatenVertexColumn( vertexConnected );