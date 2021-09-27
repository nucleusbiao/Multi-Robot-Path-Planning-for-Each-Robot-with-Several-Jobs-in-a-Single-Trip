function [ twoPathsMeetCollisionTime ] = find2PathsMeetCollision( path1, path2 )
% 找出两个路径meet碰撞（多机器人汇于一点）的位置
pathSubtraction = path1 - path2;
twoPathsMeetCollisionTime = find(all(pathSubtraction==0,2)==1);
end