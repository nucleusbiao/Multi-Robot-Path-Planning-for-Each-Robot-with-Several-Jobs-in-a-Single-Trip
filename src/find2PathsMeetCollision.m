function [ twoPathsMeetCollisionTime ] = find2PathsMeetCollision( path1, path2 )
% �ҳ�����·��meet��ײ��������˻���һ�㣩��λ��
pathSubtraction = path1 - path2;
twoPathsMeetCollisionTime = find(all(pathSubtraction==0,2)==1);
end