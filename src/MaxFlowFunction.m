function [ pathColl,TimeExtendDivide ] = MaxFlowFunction(map,TimeExtendCollision,noNameNodesCollision,nameNodesCollision,robotStartNodesCollision,robotEndNodesCollision)
% 最大流算法求解碰撞区域
TimeExtendDivide = TimeExtendCollision;
indexMap = map2Index( map );
tryTimesLimit = 5;
mapGap = 1;
[ noNameNodesDivide,nameNodesDivide,robotStartNodesDivide,robotEndNodesDivide,vertexConnectedDivide,nodes_numDivide,robotNumDivide,indexMapDivide,indexMapCollision ]...
    = collision2Divide( mapGap,map,noNameNodesCollision,nameNodesCollision,robotStartNodesCollision,robotEndNodesCollision );

x = [];
tryTimes = 0;
while isempty(x)
    [ edges_start_end_T1, edges_normal_T1, nodes_num_T1 ] = generateFlow_point2point_simplify( nodes_numDivide, vertexConnectedDivide );
    [edges_start_end_TE,edges_normal_TE,nodes_num_TE,robotEndNodes_TE] = timeExtendParameter(TimeExtendDivide,edges_start_end_T1,edges_normal_T1,nodes_num_T1,nodes_numDivide,robotEndNodesDivide);
    f = setCoefficient_simplify( robotNumDivide,edges_normal_T1,TimeExtendDivide,nodes_numDivide,edges_start_end_T1 );
    intcon = 1:edges_normal_TE*robotNumDivide;
    [ A, b ] = setInequationConstraints_simplify(edges_normal_T1,noNameNodesDivide,robotStartNodesDivide,nodes_numDivide,TimeExtendDivide,nameNodesDivide,edges_start_end_T1,vertexConnectedDivide );
    [ Aeq, beq ] = setEquationConstraints( nodes_num_TE, edges_normal_TE, edges_start_end_TE, robotStartNodesDivide, robotEndNodes_TE );
    lb = zeros(edges_normal_TE*robotNumDivide,1);
    ub = ones(edges_normal_TE*robotNumDivide,1);
    x = intlinprog(f,intcon,A,b,Aeq,beq,lb,ub); % x是走过的边
    if isempty(x)
        TimeExtendDivide = TimeExtendDivide+1
    end
    tryTimes = tryTimes+1;
    if (tryTimes > tryTimesLimit) && isempty(x)
%         error('是不是地图给的不够大？增大第8行的mapGap试试');
        mapGap = mapGap+1;
        fprintf('mapGap=%d \n\n', mapGap);
        [ noNameNodesDivide,nameNodesDivide,robotStartNodesDivide,robotEndNodesDivide,vertexConnectedDivide,nodes_numDivide,robotNumDivide,indexMapDivide,indexMapCollision ]...
            = collision2Divide( mapGap,map,noNameNodesCollision,nameNodesCollision,robotStartNodesCollision,robotEndNodesCollision );
        TimeExtendDivide = TimeExtendDivide-tryTimes
        tryTimes = 0;
        tryTimesLimit = tryTimesLimit+2;
    end
end
x = eliminateResultError( x );
% figure;
% hold on
% allPointGraph = drawMaxFlowPicture_simplify( nodes_numDivide, TimeExtendDivide, nodes_num_T1, nodes_num_TE );
% drawPathPicture( x, robotNumDivide, nodes_num_TE, edges_normal_TE, edges_start_end_TE, allPointGraph );
% title(['TimeExtend=' num2str(TimeExtendDivide)]);
% hold off

for i = 1:robotNumDivide % 把除起始点外的其他路径返回
    feasible_edges = edges_start_end_TE(1,:).*x((i-1)*edges_normal_TE+1:i*edges_normal_TE)';
    feasible_edges_end = edges_start_end_TE(2,:).*x((i-1)*edges_normal_TE+1:i*edges_normal_TE)';
    s = feasible_edges(find(feasible_edges > 0));
    t = feasible_edges_end(find(edges_start_end_TE(2,:).*x((i-1)*edges_normal_TE+1:i*edges_normal_TE)' > 0));
    terminalPoint{i} = zeros(1, size(t,2));
    for j = 1:size(t,2)
        terminalPoint{i}(j) = t(j)-j*nodes_numDivide;
    end
    path = numConvertBetDivideColl( terminalPoint{i},indexMapDivide,indexMapCollision );
    pathColl{i} = zeros(size(s,2),2);
    for j = 1:size(s,2)
        pathColl{i}(j,:) = index2Subscript( indexMap, path(j) );
    end
end
end