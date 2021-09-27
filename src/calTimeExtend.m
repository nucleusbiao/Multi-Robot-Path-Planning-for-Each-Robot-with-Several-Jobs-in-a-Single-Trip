function [ TimeExtend ] = calTimeExtend( path )
% 根据path计算增广的时间大小
TimeExtendArray = zeros(1,size(path,2));
for i = 1:size(path,2)
    TimeExtendArray(i) = size(path{i},1);
end
TimeExtend = max(TimeExtendArray)-1;
end