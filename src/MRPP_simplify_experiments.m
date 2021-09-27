clear
close all
clc
load('..\ָ������\1010Name.mat');
N = 50;
originalData = originalData2; % ������׺Ϊ1����ָ��������noName������׺Ϊ2��ָ��������Name��
clear originalData2

mapSize = [originalData(1,1),originalData(1,2)];
anonymousOrNot = originalData(1,3); % �����Ƿ�������1Ϊ�ǣ�����ָ�������񣩣�0Ϊ�񣨼�ָ�������񣩱�����׺Ϊ2
taskLines = originalData(1,4);
dataRow = (size(originalData,1)-1)/N;
ourResults = zeros(N,2);
for i = 5:N
    map = originalData(2+(i-1)*dataRow:1+(i-1)*dataRow+mapSize(1),:);
    robotStartNodes = originalData(2+(i-1)*dataRow+mapSize(1),:);
    robotStartNodes(find(robotStartNodes==0)) = [];
    robotEndNodes = originalData(3+(i-1)*dataRow+mapSize(1),:);
    3+(i-1)*dataRow+mapSize(1)+1
    robotEndNodes(find(robotEndNodes==0)) = [];
    if anonymousOrNot == 0
        nameNodesArray = originalData(4+(i-1)*dataRow+mapSize(1):3+(i-1)*dataRow+mapSize(1)+taskLines,:);% ָ��������
        nameNodes = zeros(taskLines*mapSize(2)/2,2);
        for j = 1:taskLines/2
            nameNodes(1+(j-1)*mapSize(2):j*mapSize(2),:) = nameNodesArray(1+(j-1)*2:j*2,:)';
        end
        nameNodes(find(nameNodes(:,1)==0),:) = [];
        noNameNodes = [];
    else
        noNameNodesArray = originalData(4+(i-1)*dataRow+mapSize(1):3+(i-1)*dataRow+mapSize(1)+taskLines,:);% ��ָ��������
        noNameNodes = zeros(1,taskLines*mapSize(2));
        for j = 1:taskLines
            noNameNodes(1+(j-1)*mapSize(2):j*mapSize(2)) = noNameNodesArray(j,:)';
        end
        noNameNodes(find(noNameNodes==0)) = [];
        nameNodes = [];
    end
    [ excutionTime, TimeExtend ] = MRPP_simplify_experimentsFunc( map, robotStartNodes, robotEndNodes, noNameNodes, nameNodes );
    ourResults(i,:) = [excutionTime, TimeExtend];
end
save 1010NameResult.mat ourResults