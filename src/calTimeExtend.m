function [ TimeExtend ] = calTimeExtend( path )
% ����path���������ʱ���С
TimeExtendArray = zeros(1,size(path,2));
for i = 1:size(path,2)
    TimeExtendArray(i) = size(path{i},1);
end
TimeExtend = max(TimeExtendArray)-1;
end