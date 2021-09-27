function [ x ] = eliminateResultError( x )
% 有时结果会出现误差，用此函数消除
abc = find(x~=0);
for i = 1:size(abc,1)
    if (x(abc(i))~=1)&&(abs(x(abc(i))-1)<1e-5)
        x(abc(i)) = 1;
    end
end
end