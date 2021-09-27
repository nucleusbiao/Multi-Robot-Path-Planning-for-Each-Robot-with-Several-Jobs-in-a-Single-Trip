function [ noNameNodesDivide,nameNodesDivide,robotStartNodesDivide,robotEndNodesDivide,vertexConnectedDivide,nodes_numDivide,robotNumDivide,indexMapDivide,indexMapCollision ]...
    = collision2Divide( mapGap,map,noNameNodesCollision,nameNodesCollision,robotStartNodesCollision,robotEndNodesCollision )
% 根据mapGap的大小，将碰撞区域中的数据向抠出区域转换
noNameNodesDivide = noNameNodesCollision;
nameNodesDivide = nameNodesCollision;
robotStartNodesDivide = robotStartNodesCollision;
robotEndNodesDivide = robotEndNodesCollision;
indexMap = map2Index( map );
subscriptStarts = zeros(size(robotStartNodesCollision,2),2);
subscriptEnds = zeros(size(robotStartNodesCollision,2),2);
for i = 1:size(robotStartNodesCollision,2)
    subscriptStarts(i,:) = index2Subscript( indexMap, robotStartNodesCollision(i) ); % 保存抠下来地图的左上角点在整个地图中的行、列
    subscriptEnds(i,:) = index2Subscript( indexMap, robotEndNodesCollision(i) ); % 保存抠下来地图的右下角点在整个地图中的行、列
end
subscriptNoNameS1 = [];
subscriptNoNameS2 = [];
subscriptNoNameE1 = [];
subscriptNoNameE2 = [];
if ~isempty(noNameNodesCollision)
    subscriptNoName = zeros(size(noNameNodesCollision,2),2);
    for i = 1;size(noNameNodesCollision,2)
        subscriptNoName(i,:) = index2Subscript( indexMap, noNameNodesCollision(i) );
    end
    subscriptNoNameS1 = min(subscriptNoName(:,1));
    subscriptNoNameS2 = min(subscriptNoName(:,2));
    subscriptNoNameE1 = max(subscriptNoName(:,1));
    subscriptNoNameE2 = max(subscriptNoName(:,2));
end
subscriptNameS1 = [];
subscriptNameS2 = [];
subscriptNameE1 = [];
subscriptNameE2 = [];
if ~isempty(nameNodesCollision)
    subscriptName = zeros(size(nameNodesCollision,1),2);
    for i = 1:size(nameNodesCollision,1)
        subscriptName(i,:) = index2Subscript( indexMap, nameNodesCollision(i,2) );
    end
    subscriptNameS1 = min(subscriptName(:,1));
    subscriptNameS2 = min(subscriptName(:,2));
    subscriptNameE1 = max(subscriptName(:,1));
    subscriptNameE2 = max(subscriptName(:,2));
end
subscriptStart = [min([subscriptStarts(:,1);subscriptEnds(:,1);subscriptNoNameS1;subscriptNameS1]), min([subscriptStarts(:,2);subscriptEnds(:,2);subscriptNoNameS2;subscriptNameS2])] - mapGap;
subscriptEnd = [max([subscriptStarts(:,1);subscriptEnds(:,1);subscriptNoNameE1;subscriptNameE1]), max([subscriptStarts(:,2);subscriptEnds(:,2);subscriptNoNameE2;subscriptNameE2])] + mapGap;
subscriptStart(find(subscriptStart<1)) = 1;
if subscriptEnd(1)>size(map,1)
    subscriptEnd(1) = size(map,1);
end
if subscriptEnd(2)>size(map,2)
    subscriptEnd(2) = size(map,2);
end
mapDivide = map;
mapDivide(subscriptEnd(1)+1:end,:) = [];
mapDivide(1:subscriptStart(1)-1,:) = [];
mapDivide(:,subscriptEnd(2)+1:end) = [];
mapDivide(:,1:subscriptStart(2)-1) = [];
indexMapDivide = map2Index( mapDivide );
indexMapDivide(find(mapDivide==0)) = 0;
indexMapCollision = indexMap; % 抠下来地图的各点在原地图中的编号，用以校正调度结果
indexMapCollision(subscriptEnd(1)+1:end,:) = [];
indexMapCollision(1:subscriptStart(1)-1,:) = [];
indexMapCollision(:,subscriptEnd(2)+1:end) = [];
indexMapCollision(:,1:subscriptStart(2)-1) = [];
indexMapCollision(find(mapDivide==0)) = 0;
noNameNodesDivide = numConvertBetDivideColl( noNameNodesDivide,indexMapCollision,indexMapDivide );
nameNodesDivide = numConvertBetDivideColl( nameNodesDivide,indexMapCollision,indexMapDivide );
robotStartNodesDivide = numConvertBetDivideColl( robotStartNodesDivide,indexMapCollision,indexMapDivide );
robotEndNodesDivide = numConvertBetDivideColl( robotEndNodesDivide,indexMapCollision,indexMapDivide );
[vertexConnectedDivide, nodes_numDivide] = grid2VertexConnect( mapDivide );
robotNumDivide = size(robotStartNodesDivide, 2);
end