function [ map, robotStartNodes, robotEndNodes, nameNodes, noNameNodes ] = generateRandomData( mapSize )
% ���������������
obstacleRatio = 5; % �������õ�ͼ���ϰ���ı����Ǽ���֮һ
robotNum = ceil(1.5*sqrt((mapSize(1)+mapSize(2))/2)); % ���û���������
taskNumArray = randi([robotNum-2,robotNum+2],1,robotNum); % ����ÿ�������˵���������
taskNumArray(find(taskNumArray<1)) = 1;
taskNum = sum(taskNumArray);
map = randi([0,obstacleRatio-1],mapSize(1),mapSize(2));
map(find(map>=1)) = 1;
indexMap = map2Index( map );
% indexMap(find(map==0)) = 0;
nodes_num = indexMap(end,end);
robotStartNodes = randperm(nodes_num,robotNum);
robotStartNodes = sort(robotStartNodes);
robotEndNodes = randperm(nodes_num,robotNum);
flag = 0;
while(1) % ȷ��ÿ�������˵������յ㲻ͬ
    for i = 1:robotNum
        if robotStartNodes(i) == robotEndNodes(i)
            robotEndNodes = randperm(nodes_num,robotNum);
            break
        else
            if i == robotNum
                flag = 1;
            end
        end
    end
    if flag == 1
        break
    end
end

nameNodes = zeros(taskNum,2);
nameNodesEnd = 1;
nameNodes(nameNodesEnd:nameNodesEnd+taskNumArray(1)-1,:) = [ones(taskNumArray(1),1)*robotStartNodes(1),randperm(nodes_num,taskNumArray(1))'];
nameNodesEnd = nameNodesEnd+taskNumArray(1);
while(1) % ȷ������㲻���κ���ʼ��
    b1 = intersect(nameNodes(1:nameNodesEnd-1,2),robotStartNodes);
    b2 = intersect(nameNodes(1:nameNodesEnd-1,2),robotEndNodes);
    if isempty(b1) && isempty(b2)
        break
    else
        nameNodes(1:nameNodesEnd-1,2) = randperm(nodes_num,taskNumArray(1))';
    end
end
for i = 2:robotNum
    nameNodes(nameNodesEnd:nameNodesEnd+taskNumArray(i)-1,:) = [ones(taskNumArray(i),1)*robotStartNodes(i),randperm(nodes_num,taskNumArray(i))'];
    while(1) % ȷ����ͬ�����˲�����䵽��ͬ�������
        a = intersect(nameNodes(1:nameNodesEnd-1,2),nameNodes(nameNodesEnd:nameNodesEnd+taskNumArray(i)-1,2));
        if ~isempty(a)
            nameNodes(nameNodesEnd:nameNodesEnd+taskNumArray(i)-1,2) = randperm(nodes_num,taskNumArray(i))';
            continue
        end
        
        b1 = intersect(nameNodes(nameNodesEnd:nameNodesEnd+taskNumArray(i)-1,2),robotStartNodes); % ȷ������㲻���κ���ʼ��
        b2 = intersect(nameNodes(nameNodesEnd:nameNodesEnd+taskNumArray(i)-1,2),robotEndNodes);
        if isempty(b1) && isempty(b2)
            break
        else
            nameNodes(nameNodesEnd:nameNodesEnd+taskNumArray(i)-1,2) = randperm(nodes_num,taskNumArray(i))';
        end
    end
    nameNodesEnd = nameNodesEnd+taskNumArray(i);
end
noNameNodes = randperm(nodes_num,taskNum);
while(1) % ȷ������㲻���κ���ʼ��
    b1 = intersect(noNameNodes,robotStartNodes);
    b2 = intersect(noNameNodes,robotEndNodes);
    if isempty(b1) && isempty(b2)
        break
    else
        noNameNodes = randperm(nodes_num,taskNum);
    end
end
end