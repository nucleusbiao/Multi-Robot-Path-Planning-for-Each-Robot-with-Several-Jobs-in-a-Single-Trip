function [ x ] = eliminateResultError( x )
% ��ʱ�����������ô˺�������
abc = find(x~=0);
for i = 1:size(abc,1)
    if (x(abc(i))~=1)&&(abs(x(abc(i))-1)<1e-5)
        x(abc(i)) = 1;
    end
end
end