function [ map, robotStartNodes, robotEndNodes, noNameNodes, nameNodes ] = generateOriginalData( mapSize, randomOrNot, anonymousOrNot )
% 此函数用于返回需要的数据。注意看注释，注释对每个数据进行了详细介绍。

if mapSize == [0,0]
    map = ones(4,3); % 为栅格地图，1代表可以走的路径，0代表block
    map(1,[1,3]) = 0;
    map(2,3) = 0;
    map(3,1) = 0;
    map(4,[1,3]) = 0;
%     robotStartNodes = [1,3]; % 需要robotEndNodes数组的目标点值一一对应。
%     robotEndNodes = [4,5];
%     noNameNodes = [6]; % 必须要路过的非指派的点。格式为[2,5]，意为机器人必须经过顶点2、5。
%     nameNodes = [3,5;3,2]; % 必须要路过的指派的点。格式是[1,5;1,2;2,6]，意为机器人1必须依次到顶点5、2，机器人2必须到顶点6。
    robotStartNodes = [1,3]; % 需要robotEndNodes数组的目标点值一一对应。
    robotEndNodes = [5,6];
    noNameNodes = [2]; % 必须要路过的非指派的点。格式为[2,5]，意为机器人必须经过顶点2、5。
    nameNodes = [1,6; 1,4]; % 必须要路过的指派的点。格式是[1,5;1,2;2,6]，意为机器人1必须依次到顶点5、2，机器人2必须到顶点6。
elseif mapSize == [6,6]
    map = ones(6,6);
    map(2:4,2) = zeros(3,1);
    map(1,4:5) = zeros(1,2);
    map(3:5,4) = zeros(3,1);
    robotStartNodes = [1,2,4];
    robotEndNodes = [26,23,19];
    noNameNodes = [];
    nameNodes = [1,11;1,19;4,2];
else 
    if randomOrNot == 1
        [ map, robotStartNodes, robotEndNodes, nameNodes, noNameNodes ] = generateRandomData( mapSize );
%         figure
%         colormap(gray(256));
%         drawGridPicture( map );
        if anonymousOrNot == 1
            nameNodes = [];
        elseif anonymousOrNot == 0
            noNameNodes = [];
        end
    elseif randomOrNot == 0
        if mapSize == [10,10]
            if anonymousOrNot == 1
                load map10NoNameTest.mat
            elseif anonymousOrNot == 0
                load map10NameTest.mat
            end
        elseif mapSize == [20,20]
            load('map20Test.mat');
            if anonymousOrNot == 1
                nameNodes = [];
            elseif anonymousOrNot == 0
                noNameNodes = [];
            end
        elseif mapSize == [100,100]
            if anonymousOrNot == 1
                load('map100NoNameTest.mat');
            elseif anonymousOrNot == 0
                
            end
        end
    end
end
end