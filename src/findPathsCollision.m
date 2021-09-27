function [ pathsCollisionTime, pathsCollisionID ] = findPathsCollision( twoPathsCollisionTime, twoPathsCollisionID )
% 从两两相撞的时间和ID中总结出每个相撞点和相撞时间
interval = 1; % 设置两个相撞点相距多少时长以内时会被总结在一起

pathsCollisionTime(1,1:2) = twoPathsCollisionTime{1}(1); % 每行代表相撞的起始时间和结束时间
pathsCollisionID{1} = twoPathsCollisionID(1,:);
for i = 2:size(twoPathsCollisionTime{1},1) % 总结第一个对碰撞
    insertFlag = 0;
    for j = 1:size(pathsCollisionTime,1)
        if (twoPathsCollisionTime{1}(i)>=pathsCollisionTime(j,1)-interval) && (twoPathsCollisionTime{1}(i)<=pathsCollisionTime(j,2)+interval)
            pathsCollisionTime(j,1) = min(pathsCollisionTime(j,1),twoPathsCollisionTime{1}(i));
            pathsCollisionTime(j,2) = max(pathsCollisionTime(j,2),twoPathsCollisionTime{1}(i));
            insertFlag = 1;
            break
        end
    end
    if insertFlag == 0
        pathsCollisionTime(end+1,:) = twoPathsCollisionTime{1}(i);
        pathsCollisionID{end+1} = twoPathsCollisionID(1,:);
    end
end
[ pathsCollisionTime, pathsCollisionID ] = neatenPathsCollision( pathsCollisionTime, pathsCollisionID );
for i = 2:size(twoPathsCollisionID,1) % 总结后续的碰撞
    for j = 1:size(twoPathsCollisionTime{i},1)
        insertFlag = 0;
        for k = 1:size(pathsCollisionTime,1)
            if (twoPathsCollisionTime{i}(j)>=pathsCollisionTime(k,1)-interval) && (twoPathsCollisionTime{i}(j)<=pathsCollisionTime(k,2)+interval) && ...
                    (size(unique([pathsCollisionID{k},twoPathsCollisionID(i,:)]),2) < size(pathsCollisionID{k},2)+size(twoPathsCollisionID(i,:),2))
                pathsCollisionTime(k,1) = min(pathsCollisionTime(k,1),twoPathsCollisionTime{i}(j));
                pathsCollisionTime(k,2) = max(pathsCollisionTime(k,2),twoPathsCollisionTime{i}(j));
                pathsCollisionID{k} = unique([pathsCollisionID{k},twoPathsCollisionID(i,:)]);
                insertFlag = 1;
                break
            end
        end
        if insertFlag == 0
            pathsCollisionTime(end+1,:) = twoPathsCollisionTime{i}(j);
            pathsCollisionID{end+1} = twoPathsCollisionID(i,:);
        end
        [ pathsCollisionTime, pathsCollisionID ] = neatenPathsCollision( pathsCollisionTime, pathsCollisionID );
    end
end

collisionTimeMeet = zeros(1,size(pathsCollisionTime,1)-1); % 再尝试找到距离相近的碰撞，并整理到一起
for i = 1:size(pathsCollisionTime)-1
    collisionTimeMeet(i) = pathsCollisionTime(i+1,1)-pathsCollisionTime(i,2);
end
if ~isempty(collisionTimeMeet)
    [ pathsCollisionTime,pathsCollisionID ] = neatenPathsCollisionFusion( collisionTimeMeet,pathsCollisionTime,pathsCollisionID );
end
end