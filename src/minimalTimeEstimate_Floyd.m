function [ minimalTimeExtend ] = minimalTimeEstimate_Floyd( vertexConnected, nodes_num, robotStartNodes, robotEndNodes, nameNodes )
% ��Floyd�㷨����ⵥ��������ʱ���������ʱ��
MGraph = zeros(nodes_num);
for i = 1:size(vertexConnected,1)
    MGraph(vertexConnected(i,1),vertexConnected(i,2)) = 1;
    MGraph(vertexConnected(i,2),vertexConnected(i,1)) = 1;
end
MGraph(MGraph == 0) = inf;
for i = 1:nodes_num
    MGraph(i,i) = 0;
end
D = MGraph; % ������������С·��������ֵ
P = zeros(nodes_num,nodes_num);
for i = 1:nodes_num
    P(:,i) = i; % ·������
end
for k = 1:nodes_num
    for v = 1:nodes_num
        for w = 1:nodes_num
            if D(v,w) > D(v,k)+D(k,w)
                D(v,w) = D(v,k)+D(k,w);
                P(v,w) = P(v,k);
            end
        end
    end
end

path = [nameNodes;[robotStartNodes',robotEndNodes']];
path = sortrows(path,1);
lastStartPoint = inf;
timeExtendArray = [];
for i = 1:size(path,1)
    if lastStartPoint ~= path(i,1)
        timeExtendArray(end+1) = D(path(i,1), path(i,2));
        lastStartPoint = path(i,1);
    else
        timeExtendArray(end) = timeExtendArray(end) + D(path(i-1,2), path(i,2));
    end
end
minimalTimeExtend = max(timeExtendArray);