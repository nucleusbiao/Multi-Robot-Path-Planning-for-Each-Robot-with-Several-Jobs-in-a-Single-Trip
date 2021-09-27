function [ minimalTimeExtend, path ] = minimalTimeEstimate_Astar( map , robotStartNodes, robotEndNodes, nameNodes )
% ��A*�㷨��������е�������ʱ���������ʱ�� & ���е�������·��
indexMap = map2Index( map );
minimalTimeExtend = 0;
for i = 1:size(robotStartNodes,2)
    source = index2Subscript( indexMap, robotStartNodes(i) );
    path{i} = source;
    if ~isempty(nameNodes)
        nameNodesI = nameNodes(find(nameNodes(:,1)==robotStartNodes(i)),:);
        for j = 1:size(nameNodesI,1)
            if j == 1
                source = index2Subscript( indexMap, nameNodesI(j,1) );
            else
                source = index2Subscript( indexMap, nameNodesI(j-1,2) );
            end
            goal = index2Subscript( indexMap, nameNodesI(j,2) );
            pathI = Astar( map ,source, goal );
            path{i} = [path{i}; pathI];
        end
        if ~isempty(nameNodesI)
            source = index2Subscript( indexMap, nameNodesI(j,2) );
        end
    end
    goal = index2Subscript( indexMap, robotEndNodes(i) );
    pathI = Astar( map ,source, goal );
    path{i} = [path{i}; pathI]; 
    minimalTimeExtend = max(size(path{i}, 1)-1, minimalTimeExtend);
end
end