function [ divideTime, divideID ] = divideAstarPath( path )
% ��A*������·�����в�֣��ҳ���ײ��ʱ��ͻ����˱�ţ������ص�һ����ײ�Ļ����˱����Ҫ�ָ��ʱ��ǩ
timeGap = 1;
divideTime = [];
divideID = [];
twoPathsCollisionTime = [];
twoPathsCollisionID = [];
pathsCollisionTime = [];
for i = 1:size(path,2)-1
    for j = i+1:size(path,2)
        twoPathsMeetCollisionTimeI = find2PathsMeetCollision( path{i}, path{j}); % �ҳ�meet a vertex��ײ��·���غ���ײ
        if ~isempty(twoPathsMeetCollisionTimeI)
            twoPathsCollisionTime{end+1} = twoPathsMeetCollisionTimeI;
            twoPathsCollisionID(end+1,:) = [i,j];
        end
        twoPathsHeadOnCollisionTimeI = find2PathsHeadOnCollision( path{i}, path{j} ); % �ҳ�head-on��ײ
        if ~isempty(twoPathsHeadOnCollisionTimeI)
            twoPathsCollisionTime{end+1} = twoPathsHeadOnCollisionTimeI;
            twoPathsCollisionID(end+1,:) = [i,j];
        end
    end
end
if ~isempty(twoPathsCollisionTime)
    [ pathsCollisionTime, pathsCollisionID ] = findPathsCollision( twoPathsCollisionTime, twoPathsCollisionID );
    divideTimeArray = zeros(size(pathsCollisionTime));
end

for i = 1:size(pathsCollisionTime,1)
    if pathsCollisionTime(i,1) <= timeGap+1
        divideTimeArray(i,1) = 1;
    else
        divideTimeArray(i,1) = pathsCollisionTime(i,1)-timeGap;
    end
    divideTimeArray(i,2) = pathsCollisionTime(i,2)+timeGap;
end

if ~isempty(twoPathsCollisionTime)
    divideID = pathsCollisionID{1};
    divideTime = divideTimeArray(1,:);
end
end