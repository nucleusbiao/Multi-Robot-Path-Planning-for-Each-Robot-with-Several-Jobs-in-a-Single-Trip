function [ path ] = Astar( map ,source, goal )
% ��A*�㷨����ⵥ��������ʱ���������ʱ�� & ��������·��
connection = [0 1 0; % 2Ϊ�����ˣ�1Ϊ����Ѱ�ҵ�·����0Ϊ����Ѱ�ҵ�·������������Ϊֻ��Ѱ�����������ĸ�λ�ã�����б���ߡ�
              1 2 1;
              0 1 0];

openList = [source 0 heuristic(source,goal) 0+heuristic(source,goal) -1]; % �����б�����Ҫ���в�ѯ�ġ�������[����x ����y 0 h cost �õ�ĸ�λ�ã�����-1��������㣩]
closed = ones(size(map)); % 1Ϊ�ѷ��ʹ���0Ϊ��û�з��ʹ�
closedList = []; % ����б���ѯ����
pathFound = false;
while size(openList,1) > 0
     [A, I] = min(openList,[],1);
     n = openList(I(5),:); % ѡȡcost��С��
     openList = [openList(1:I(5)-1,:);openList(I(5)+1:end,:)]; % �����ڲ�ѯ�Ĵ�open list��ɾ��
     if n(1)==goal(1) && n(2)==goal(2) % �ѵ�Ŀ���
         pathFound = true;
         break;
     end
     [robotX,robotY] = find(connection==2);
     [moveX,moveY] = find(connection==1);
     for moveXI = 1:size(moveX,1) % �������п����ߵ�·��
         newPos = [n(1)+moveX(moveXI)-robotX n(2)+moveY(moveXI)-robotY];
         if checkPosition( newPos,map ) % ���Ҫȥ�ĵ���Ե���Ļ�����û��block��
              if closed(newPos(1),newPos(2)) ~= 0 % Ҫȥ�ĵ�֮ǰû�б���ѯ��
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
                      openList = [openList;newPos historicCost heuristicCost totalCost size(closedList,1)+1]; % ����µ�
                  end
              end
         end           
     end
     closed(n(1),n(2)) = 0;
     closedList = [closedList;n]; % ���·���б�
end
if ~pathFound
    error('no path found')
end

path = [n(1:2)]; % �Ӹ�λ�ÿ�ʼ��·��
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