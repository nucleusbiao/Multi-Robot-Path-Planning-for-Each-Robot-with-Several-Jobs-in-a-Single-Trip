function [ robotStartNodes, robotEndNodes, robotNum, nameNodesStart ] = neatenNodes( robotStartNodes, robotEndNodes, nameNodes )
% �����������õĵ�
robotStartEndNodes = neatenVertexColumn([robotStartNodes;robotEndNodes]')';
robotStartNodes = robotStartEndNodes(1,:);
robotEndNodes = robotStartEndNodes(2,:);
robotNum = size(robotStartNodes,2);
if ~isempty(nameNodes)
    nameNodesStart = unique(nameNodes(:,1));
else
    nameNodesStart = [];
end
end