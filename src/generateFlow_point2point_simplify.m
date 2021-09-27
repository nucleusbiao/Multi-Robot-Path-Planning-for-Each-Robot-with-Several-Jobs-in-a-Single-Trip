function [ edges_start_end, edges_normal_num, nodes_num_graph ] = generateFlow_point2point_simplify( nodes_num, vertexConnected )
 % vertexConnected中为有边相连的两个顶点。格式是[1,3;2,3]，意为顶点1、3相连，顶点2、3相连。
nodes_num_graph = nodes_num;
edges_normal_num = nodes_num + 2*size(vertexConnected, 1);
edges_start_end = [];
jointArray(:,1) = vertexConnected(:,2);
jointArray(:,2) = vertexConnected(:,1);
jointArray = [vertexConnected;jointArray;[1:nodes_num]',[1:nodes_num]'];
jointArray = neatenVertexColumn( jointArray );
for i = 1:nodes_num
    start_i = jointArray(all(jointArray(:,1) ==i,2),:);
    start_i = neatenVertexRow(start_i);
    for j = 1:size(start_i, 1)
        if start_i(j,1)==start_i(j,2)
            edges_start_end(1,end+1) = i;
            edges_start_end(2,end) = i+nodes_num_graph;
        else
            edges_start_end(1,end+1) = i;
            edges_start_end(2,end) = start_i(j,1)+start_i(j,2)-i+nodes_num_graph;
        end
    end
end
end