clear
close all
clc

load map7NoNameTest.mat
% load map7NameTest.mat
[ robotStartNodes, robotEndNodes, robotNum, nameNodesStart ] = neatenNodes( robotStartNodes, robotEndNodes, nameNodes );
[vertexConnected, nodes_num] = grid2VertexConnect( map ); % 有边相连的两个顶点。格式是[1,3;2,3]，意为顶点1、3相连，顶点2、3相连。
[ TimeExtend, ~ ] = minimalTimeEstimate_Astar( map , robotStartNodes, robotEndNodes, nameNodes );

indexMap = map2Index( map );
tryTimesLimit = 5;

x = [];
tryTimes = 0;
while isempty(x)
    [ edges_start_end_T1, edges_normal_T1, nodes_num_T1 ] = generateFlow_point2point_simplify( nodes_num, vertexConnected );
    [edges_start_end_TE,edges_normal_TE,nodes_num_TE,robotEndNodes_TE] = timeExtendParameter(TimeExtend,edges_start_end_T1,edges_normal_T1,nodes_num_T1,nodes_num,robotEndNodes);
    f = setCoefficient_simplify( robotNum,edges_normal_T1,TimeExtend,nodes_num,edges_start_end_T1 );
    intcon = 1:edges_normal_TE*robotNum;
    [ A, b ] = setInequationConstraints_simplify(edges_normal_T1,noNameNodes,robotStartNodes,nodes_num,TimeExtend,nameNodes,edges_start_end_T1,vertexConnected );
    [ Aeq, beq ] = setEquationConstraints( nodes_num_TE, edges_normal_TE, edges_start_end_TE, robotStartNodes, robotEndNodes_TE );
    lb = zeros(edges_normal_TE*robotNum,1);
    ub = ones(edges_normal_TE*robotNum,1);
    x = intlinprog(f,intcon,A,b,Aeq,beq,lb,ub); % x是走过的边
    if isempty(x)
        TimeExtend = TimeExtend+1
    end
    tryTimes = tryTimes+1;
    if (tryTimes > tryTimesLimit) && isempty(x)
         error('是不是地图给的不够大？增大第8行的mapGap试试');
    end
end
x = eliminateResultError( x );

for i = 1:robotNum % 把除起始点外的其他路径返回
    feasible_edges = edges_start_end_TE(1,:).*x((i-1)*edges_normal_TE+1:i*edges_normal_TE)';
    feasible_edges_end = edges_start_end_TE(2,:).*x((i-1)*edges_normal_TE+1:i*edges_normal_TE)';
    s = feasible_edges(find(feasible_edges > 0));
    t = feasible_edges_end(find(edges_start_end_TE(2,:).*x((i-1)*edges_normal_TE+1:i*edges_normal_TE)' > 0));
    st = [s(1),t];
    pathPoint{i} = zeros(1, size(st,2));
    for j = 1:size(st,2)
        pathPoint{i}(j) = st(j)-(j-1)*nodes_num;
    end
    path{i} = zeros(size(st,2),2);
    for j = 1:size(st,2)
        path{i}(j,:) = index2Subscript( indexMap, pathPoint{i}(j) );
    end
end
TimeExtend = calTimeExtend( path )

if ~isempty(nameNodes)
    assignNodes = nameNodes;
else
    assignNodes1 = [];
    assignNodes = [];
    for i = 1:size(noNameNodes,2)
        noNameNodesI = [];
        for j = 1:robotNum
            noNameNodesILine = find(pathPoint{j}==noNameNodes(i));
            if ~isempty(noNameNodesILine)
                noNameNodesI = [noNameNodesI;[robotStartNodes(j),noNameNodes(i),noNameNodesILine(1)]];
            end
        end
        noNameNodesI = sortrows(noNameNodesI,3);
        assignNodes1 = [assignNodes1;noNameNodesI(1,:)];
    end
    for i = 1:robotNum
        assignNodes2 = assignNodes1(find(assignNodes1(:,1)==robotStartNodes(i)),:);
        assignNodes2 = sortrows(assignNodes2,3);
        assignNodes2(:,3) = [];
        assignNodes = [assignNodes;assignNodes2];
    end
end

figure
hold on
drawPathAnimation( map,path,robotStartNodes,assignNodes,noNameNodes )
hold off