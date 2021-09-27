function [ allPointGraph ] = drawMaxFlowPicture_simplify( nodes_num, TimeExtend, nodes_num_T1, nodes_num_TE )
% 画出时间增广后的最大流图
x1 = 1:nodes_num;
allPointGraph = zeros(nodes_num_TE,2);
allPointGraph(end-nodes_num+1:end,1) = x1;
allPointGraph(end-nodes_num+1:end,2) = 1;

allPointGraph(end-nodes_num*2+1:end-nodes_num,1) = x1;
allPointGraph(end-nodes_num*2+1:end-nodes_num,2) = 2;
for i = 1:TimeExtend-1
    allPointGraph((i-1)*nodes_num_T1+1:i*nodes_num_T1,1) = x1;
    allPointGraph((i-1)*nodes_num_T1+1:i*nodes_num_T1,2) = TimeExtend-i+2;
end
scatter(allPointGraph(:,1),allPointGraph(:,2),50,'filled');
for i = 1:nodes_num
    text(x1(i)-0.1,allPointGraph(i,2)+1,num2str(i));
end
for i= 0:TimeExtend
    text(x1(end)+0.8,allPointGraph(i*nodes_num+1,2),sprintf('T%d',i));
end
axis([0,max(allPointGraph(:,1))+1,0,max(allPointGraph(:,2))+2])
end