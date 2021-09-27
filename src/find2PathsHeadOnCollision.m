function [ twoPathsHeadOnCollisionTime ] = find2PathsHeadOnCollision( path1, path2 )
% 找出两个路径head-on碰撞（相向而行）的位置
pathSubtraction = path1 - path2;
twoPathsHeadOnCollisionTime = [];
for i = 1:size(pathSubtraction,1)-1
    if ~isequal(pathSubtraction(i,:),[0,0])
        if (pathSubtraction(i,1)+pathSubtraction(i+1,1)==0) && (pathSubtraction(i,2)+pathSubtraction(i+1,2)==0)
            twoPathsHeadOnCollisionTime = [twoPathsHeadOnCollisionTime;i;i+1];
        end
    end
end

end