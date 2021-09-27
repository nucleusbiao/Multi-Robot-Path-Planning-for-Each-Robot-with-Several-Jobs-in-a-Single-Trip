function [ TimeExtend, path, assignNodes ] = assignNoNameNodes( path, noNameNodes, map, robotStartNodes, nameNodes )
% 把非指派性的点分配给路径时长最短、离的最近的路径，并用A*算法进行路径规划
searchDist = 4; % 搜寻距离初始化，寻找距离非指派点此距离内的已有路径
searchDistIncrease = 2; % 搜寻不到时，以此数值增长搜寻距离
if ~isempty(noNameNodes) % 设置尝试次数上限，咋设置呢？？？
    tryTimesLimit = ceil(((size(map,1)+size(map,1))/2-searchDist)/searchDistIncrease);
elseif ~isempty(nameNodes)
    tryTimesLimit = 5;
end
if ~isempty(nameNodes)
    assignNodes = nameNodes;
else
    assignNodes = int16.empty(0,2);
end
indexMap = map2Index( map );
robotNum = size(path,2);
pathLength = zeros(1,robotNum);
subscriptNNN = zeros(size(noNameNodes,2),2);
for i = 1:size(noNameNodes,2)
    assignFlag = 0;
    for j = 1:robotNum
        pathLength(j) = size(path{j},1);
    end
    [~,pathLengthQueue] = sort(pathLength);
    subscriptNNN(i,:) = index2Subscript( indexMap, noNameNodes(i) );
    for j = 1:robotNum % 先检查非指派点是否已经被经过
        pathI = path{pathLengthQueue(j)};
        robotIndex = robotStartNodes(pathLengthQueue(j));
        for k = 1:size(pathI,1)
            if (pathI(k,1)==subscriptNNN(i,1))&&(pathI(k,2)==subscriptNNN(i,2))
                if isempty(assignNodes) || isempty(find(assignNodes(:,1)==robotIndex)) % 该路径上只有起始点和终点时
                    assignNodes = [assignNodes; [robotIndex,noNameNodes(i)]];
                    assignNodes = sortrows(assignNodes ,1);
                    assignFlag = 1;
                    break
                else % 该路径有路过必过点时
                    assignNodesIline = find(assignNodes(:,1)==robotIndex);
                    assignNodesI = assignNodes(assignNodesIline,2)';
                    subscriptANI = zeros(size(assignNodesI,2),2);
                    ANIline = zeros(1,size(assignNodesI,2));
                    for g = 1:size(assignNodesI,2)
                        subscriptANI(g,:) = index2Subscript( indexMap, assignNodesI(g) );
                        ANIlines = intersect((find(pathI(:,1)==subscriptANI(g,1))),(find(pathI(:,2)==subscriptANI(g,2))));
                        ANIline(g) = ANIlines(1);
                        if ANIline(g) >= k % 能插在已有必过点之前
                            assignNodes = [assignNodes(1:assignNodesIline(g)-1,:);[robotIndex,noNameNodes(i)];assignNodes(assignNodesIline(g):end,:)];
                            assignFlag = 1;
                            break
                        elseif g == size(assignNodesI,2) % 只能插在已有必过点之后
                            assignNodes = [assignNodes(1:assignNodesIline(g),:);[robotIndex,noNameNodes(i)];assignNodes(assignNodesIline(g)+1:end,:)];
                            assignFlag = 1;
                            break
                        end
                    end
                end
            end
            if assignFlag == 1
                break
            end
        end
        if assignFlag == 1
            break
        end
    end
    
    % 未被其他路径经过，开始进行分配并规划路径
    tryTimes = 0;
    while(assignFlag == 0)
        for j = 1:robotNum
            pathI = path{pathLengthQueue(j)};
            robotIndex = robotStartNodes(pathLengthQueue(j));
            for k = 1:size(pathI,1)
                if (abs(pathI(k,1)-subscriptNNN(i,1))+abs(pathI(k,2)-subscriptNNN(i,2))) <= searchDist
                    if isempty(assignNodes) || isempty(find(assignNodes(:,1)==robotIndex)) % 该路径上只有起始点和终点时
                        source1 = index2Subscript( indexMap, robotIndex );
                        goal1 = subscriptNNN(i,:);
                        path1 = Astar( map ,source1, goal1 );
                        goal2 = pathI(end,:);
                        path2 = Astar( map ,goal1, goal2 );
                        path{pathLengthQueue(j)} = [source1;path1;path2];
                        assignNodes = [assignNodes; [robotIndex,noNameNodes(i)]];
                        assignNodes = sortrows(assignNodes ,1);
                        assignFlag = 1;
                        break
                    else % 该路径有路过必过点时
                        assignNodesIline = find(assignNodes(:,1)==robotIndex);
                        assignNodesI = assignNodes(assignNodesIline,2)';
                        subscriptANI = zeros(size(assignNodesI,2),2);
                        ANIline = zeros(1,size(assignNodesI,2));
                        for g = 1:size(assignNodesI,2)
                            subscriptANI(g,:) = index2Subscript( indexMap, assignNodesI(g) );
                            ANIlines = intersect((find(pathI(:,1)==subscriptANI(g,1))),(find(pathI(:,2)==subscriptANI(g,2))));
                            ANIline(g) = ANIlines(1);
                            if ANIline(g) >= k % 能插在已有必过点之前
                                if g == 1
                                    source1 = index2Subscript( indexMap, robotIndex);
                                else
                                    source1 = subscriptANI(g-1,:);
                                end
                                goal1 = subscriptNNN(i,:);
                                path1 = Astar( map ,source1, goal1 );
                                goal2 = subscriptANI(g,:);
                                path2 = Astar( map ,goal1, goal2 );
                                if g == 1
                                    path{pathLengthQueue(j)}=[source1;path1;path2;pathI(ANIline(g)+1:end,:)];
                                else
                                    path{pathLengthQueue(j)}=[pathI(1:ANIline(g-1),:);path1;path2;pathI(ANIline(g)+1:end,:)];
                                end
                                assignNodes = [assignNodes(1:assignNodesIline(g)-1,:);[robotIndex,noNameNodes(i)];assignNodes(assignNodesIline(g):end,:)];
                                assignFlag = 1;
                                break
                            elseif g == size(assignNodesI,2) % 只能插在已有必过点之后
                                source1 = subscriptANI(end,:);
                                goal1 = subscriptNNN(i,:);
                                path1 = Astar( map ,source1, goal1 );
                                goal2 = pathI(end,:);
                                path2 = Astar( map ,goal1, goal2 );
                                path{pathLengthQueue(j)}=[pathI(1:ANIline(g),:);path1;path2];
                                assignNodes = [assignNodes(1:assignNodesIline(g),:);[robotIndex,noNameNodes(i)];assignNodes(assignNodesIline(g)+1:end,:)];
                                assignFlag = 1;
                                break
                            end
                        end
                    end
                end
                if assignFlag == 1
                    break
                end
            end
            if assignFlag == 1
                break
            end
        end
        if assignFlag == 1
            break
        end
        searchDist = searchDist+searchDistIncrease;
        if tryTimes > tryTimesLimit
            error('咋离的这么远呢，是不是有问题啊');
        end
        tryTimes = tryTimes+1;
    end
    
    % 解决：随着某机器人路径非指派点的指定，导致在该路径中后面的非指派点在前面被经过
    while(1)
        pathI = path{pathLengthQueue(j)};
        assignNodesIline = find(assignNodes(:,1)==robotIndex);
        assignNodesI = assignNodes(assignNodesIline,2)';
        subscriptANI = zeros(size(assignNodesI,2),2);
        ANIline = zeros(1,size(assignNodesI,2));
        for g = 1:size(assignNodesI,2)
            subscriptANI(g,:) = index2Subscript( indexMap, assignNodesI(g) );
            ANIlines = intersect((find(pathI(:,1)==subscriptANI(g,1))),(find(pathI(:,2)==subscriptANI(g,2))));
            ANIline(g) = ANIlines(1);
        end
        if ~isequal(ANIline, sort(ANIline))
            [~,ANIlineOrder] = sort(ANIline);
            assignNodesINew = zeros(size(assignNodesI));
            for m = 1:size(assignNodesI,2)
                assignNodesINew(m) = assignNodesI(ANIlineOrder(m));
            end
            assignNodes(assignNodesIline(1):assignNodesIline(end),2) = assignNodesINew';
            robotEndNodesI = indexMap(pathI(end,1),pathI(end,2));
            [ ~, pathINew ] = minimalTimeEstimate_Astar( map , robotIndex, robotEndNodesI, [robotIndex*ones(size(assignNodesINew,2),1),assignNodesINew'] );
            path{pathLengthQueue(j)} = pathINew{1};
        else
            break
        end
    end
end

TimeExtend = calTimeExtend( path );
end