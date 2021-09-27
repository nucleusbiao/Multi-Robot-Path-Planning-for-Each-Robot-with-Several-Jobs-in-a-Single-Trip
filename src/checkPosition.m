function [ feasible ] = checkPosition( newPos,map )
% 用以检查要去的点是否可以到达（即没有block）
if newPos(1)>=1 && newPos(1)<=size(map,1) && newPos(2)>=1 && newPos(2)<=size(map,2)
    if map(newPos(1),newPos(2)) == 1
        feasible = true;
    else
        feasible = false;
    end
else
    feasible = false;
end
end