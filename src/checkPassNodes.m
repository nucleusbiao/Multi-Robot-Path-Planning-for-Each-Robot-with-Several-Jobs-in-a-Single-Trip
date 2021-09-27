function [TimeExtendCollision,noNameNodesCollision,nameNodesCollision,robotStartNodesCollision,robotEndNodesCollision,robotNumCollision] = checkPassNodes(map,path,divideTime,divideID,noNameNodes,nameNodes,robotStartNodes)
% 查找时间内是否有经过的nameNodes和noNameNodes点。PS:不需要检查起点和终点
noNameNodesCollision = [];
nameNodesCollision = [];
TimeExtendCollision = divideTime(1,2)-divideTime(1,1);
indexMap = map2Index( map );
robotStartNodesCollision = zeros(1,size(divideID,2));
robotEndNodesCollision = zeros(1,size(divideID,2));
for i = 1:size(divideID,2)
    passPoint = zeros(1,TimeExtendCollision-1);
    dividePath{i} = path{divideID(i)}(divideTime(1):divideTime(2),:);
    robotStartNodesCollision(i) = indexMap(dividePath{i}(1,1),dividePath{i}(1,2));
    robotEndNodesCollision(i) = indexMap(dividePath{i}(end,1),dividePath{i}(end,2));
    for j = 2:TimeExtendCollision
        passPoint(j-1) = indexMap(dividePath{i}(j,1),dividePath{i}(j,2));
    end
    noNameNodesCollision = [noNameNodesCollision, intersect(passPoint,noNameNodes)];
    if ~isempty(nameNodes)
        nameNodesI = nameNodes(find(nameNodes(:,1)==robotStartNodes(divideID(i))),2);
        for j = 1:size(nameNodesI,1)
            nameNodesCollisionI = intersect(passPoint,nameNodesI(j));
            if ~isempty(nameNodesCollisionI)
                nameNodesCollision = [nameNodesCollision; [robotStartNodesCollision(i),nameNodesCollisionI]];
            end
        end
    end
end
noNameNodesCollision = unique(noNameNodesCollision);
if isempty(noNameNodesCollision)
    noNameNodesCollision = [];
end
if isempty(nameNodesCollision)
    nameNodesCollision = [];
end
robotNumCollision = size(robotStartNodesCollision, 2);
end