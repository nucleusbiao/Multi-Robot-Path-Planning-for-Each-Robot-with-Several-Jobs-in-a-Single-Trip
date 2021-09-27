function [ excutionTime, TimeExtend ] = MRPP_simplify_experimentsFunc( map, robotStartNodes, robotEndNodes, noNameNodes, nameNodes )
tic;
finishedDiviedeTime = [];
finishedDiviedeID = [];
[ robotStartNodes, robotEndNodes, robotNum, nameNodesStart ] = neatenNodes( robotStartNodes, robotEndNodes, nameNodes );
[vertexConnected, nodes_num] = grid2VertexConnect( map ); % 有边相连的两个顶点。格式是[1,3;2,3]，意为顶点1、3相连，顶点2、3相连。

[ TimeExtend, path ] = minimalTimeEstimate_Astar( map , robotStartNodes, robotEndNodes, nameNodes );
TimeExtend
% TimeExtend = minimalTimeEstimate_Floyd( vertexConnected, nodes_num, robotStartNodes, robotEndNodes, nameNodes );
[ TimeExtend, path, assignNodes ] = assignNoNameNodes( path, noNameNodes, map, robotStartNodes, nameNodes );

pathSupplement = pathSupplementFuc( path );
[divideTime, divideID] = divideAstarPath( pathSupplement );
if ~isempty(divideTime)
    finishedDiviedeTime(end+1,:) = divideTime;
    finishedDiviedeID{end+1} = divideID;
end
while(~isempty(divideTime))
    [TimeExtendCollision,noNameNodesCollision,nameNodesCollision,robotStartNodesCollision,robotEndNodesCollision,robotNumCollision] = checkPassNodes( map, pathSupplement, divideTime, divideID, noNameNodes, nameNodes, robotStartNodes );
    % PS：变量后有Collision代表是碰撞区域在整体map中显示；变量后有Divide代表将该区域抠出，自成一体。
    [pathColl,TimeExtendDivide] = MaxFlowFunction(map,TimeExtendCollision,noNameNodesCollision,nameNodesCollision,robotStartNodesCollision,robotEndNodesCollision);
    for i = 1:size(divideID,2)
        robotIndex = divideID(i);
        pathSupplement{robotIndex} = [pathSupplement{robotIndex}(1:divideTime(1),:);pathColl{i};pathSupplement{robotIndex}(divideTime(2)+1:end,:)];
        for j = divideTime(1)+TimeExtendDivide-1:-1:divideTime(1)
            if size(pathSupplement{robotIndex},1)>j && isequal(pathSupplement{robotIndex}(j+1,:),pathSupplement{robotIndex}(j,:))
                pathSupplement{robotIndex}(j,:) = [];
            else
                break
            end
        end
    end
    pathSupplement = pathSupplementFuc( pathSupplement );
    [divideTime, divideID] = divideAstarPath( pathSupplement );
    if ~isempty(divideTime)
        finishedDiviedeTime(end+1,:) = divideTime;
        finishedDiviedeID{end+1} = divideID;
    else
        finishedDiviedeTime = [];
        finishedDiviedeID = [];
    end
    if size(finishedDiviedeTime,1)>=4 && isequal(finishedDiviedeTime(end,:),finishedDiviedeTime(end-2,:)) && isequal(finishedDiviedeTime(end-1,:),finishedDiviedeTime(end-3,:))...
            && isequal(finishedDiviedeID{end},finishedDiviedeID{end-2}) && isequal(finishedDiviedeID{end-1},finishedDiviedeID{end-3}) % 为避免两个碰撞轮流出现且永不能消失的情况
        divideTime = [min(finishedDiviedeTime(end,1),finishedDiviedeTime(end-1,1)),max(finishedDiviedeTime(end,2),finishedDiviedeTime(end-1,2))];
        divideID = unique([finishedDiviedeID{end},finishedDiviedeID{end-1}]);
    end
end
fprintf('processing time=%d \n\n', toc);
excutionTime = toc;
pathSupplement = pathCondenseFuc( pathSupplement );
TimeExtend = calTimeExtend( pathSupplement );

figure
hold on
drawPathAnimation( map,pathSupplement,robotStartNodes,assignNodes,noNameNodes )
hold off
end