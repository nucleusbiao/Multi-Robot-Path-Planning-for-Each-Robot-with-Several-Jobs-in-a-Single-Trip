function [ path ] = Astar( map ,source, goal )
% 用A*算法来求解单个机器人时的最大运行时间 & 单机器人路径
connection = [0 1 0; % 2为机器人，1为可以寻找的路径，0为不可寻找的路径。现在设置为只能寻找上下左右四个位置，不能斜着走。
              1 2 1;
              0 1 0];

openList = [source 0 heuristic(source,goal) 0+heuristic(source,goal) -1]; % 开放列表，即将要进行查询的。内容是[坐标x 坐标y 0 h cost 该点的父位置（其中-1代表出发点）]
closed = ones(size(map)); % 1为已访问过，0为还没有访问过
closedList = []; % 封闭列表，查询过的
pathFound = false;
while size(openList,1) > 0
     [A, I] = min(openList,[],1);
     n = openList(I(5),:); % 选取cost最小的
     openList = [openList(1:I(5)-1,:);openList(I(5)+1:end,:)]; % 把正在查询的从open list中删掉
     if n(1)==goal(1) && n(2)==goal(2) % 已到目标点
         pathFound = true;
         break;
     end
     [robotX,robotY] = find(connection==2);
     [moveX,moveY] = find(connection==1);
     for moveXI = 1:size(moveX,1) % 遍历所有可以走的路径
         newPos = [n(1)+moveX(moveXI)-robotX n(2)+moveY(moveXI)-robotY];
         if checkPosition( newPos,map ) % 如果要去的点可以到达的话（即没有block）
              if closed(newPos(1),newPos(2)) ~= 0 % 要去的点之前没有被查询过
                  historicCost = n(3)+historic(n(1:2),newPos);
                  heuristicCost = heuristic(newPos,goal);
                  totalCost = historicCost+heuristicCost;
                  add = true; % not already in queue with better cost
                  if length(find((openList(:,1)==newPos(1)).*(openList(:,2)==newPos(2)))) >= 1
                      I = find((openList(:,1)==newPos(1)) .* (openList(:,2)==newPos(2)));
                      if openList(I,5) < totalCost
                          add = false;
                      else
                          openList = [openList(1:I-1,:);openList(I+1:end,:);];
                          add = true;
                      end
                  end
                  if add
                      openList = [openList;newPos historicCost heuristicCost totalCost size(closedList,1)+1]; % 添加新点
                  end
              end
         end           
     end
     closed(n(1),n(2)) = 0;
     closedList = [closedList;n]; % 更新封闭列表
end
if ~pathFound
    error('no path found')
end

path = [n(1:2)]; % 从父位置开始找路径
previous = n(6);
while previous > 0
    path = [closedList(previous,1:2);path];
    previous = closedList(previous,6);
end

pathLength = 0;
for i = 1:length(path)-1
    pathLength = pathLength + historic(path(i,:),path(i+1,:)); 
end
path(1,:) = [];
end