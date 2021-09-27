function [ ] = drawPathPicture( x, robotNum, nodes_num_TE, edges_normal_TE, edges_start_end_TE, allPointGraph )
% 画得到的路径
colorArray = colorMaker();
k = 1:nodes_num_TE;
for i = 1:robotNum
    feasible_edges = edges_start_end_TE(1,:).*x((i-1)*edges_normal_TE+1:i*edges_normal_TE)';
    feasible_edges_end = edges_start_end_TE(2,:).*x((i-1)*edges_normal_TE+1:i*edges_normal_TE)';
    s = feasible_edges(find(feasible_edges > 0));
    t = feasible_edges_end(find(edges_start_end_TE(2,:).*x((i-1)*edges_normal_TE+1:i*edges_normal_TE)' > 0));
    lineGraph = zeros(nodes_num_TE,nodes_num_TE);
    for j = 1:size(t,2)
        lineGraph(s(j),t(j)) = 1;
    end
    gplot(lineGraph(k,k),allPointGraph(k,:),colorArray(i));
end
end