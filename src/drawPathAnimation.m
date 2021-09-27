function [ ] = drawPathAnimation( map,path,robotStartNodes,assignNodes,noNameNodes )
% ��ʾ���������ߵĶ���
robotNum = size(robotStartNodes,2);
colorArray = colorMaker();
refreshNum = 10;
pauseTime = 1/(refreshNum*100);
timeEnd = size(path{1},1);

colormap(gray(256));
drawGridPicture( map );
robotxShift = [0.3,0.7,0.7,0.3]; % x��ƫ����
robotyShift = [0.3,0.3,0.7,0.7];
t=deg2rad(0:360);
robotGoalxShift=cos(t)/5+0.5;
robotGoalyShift=sin(t)/5+0.5;
for i = 1:robotNum % �������ˣ������Σ����յ㣨Բ�Σ�
    robotx(i,:) = path{i}(1,2)+robotxShift;
    roboty(i,:) = path{i}(1,1)+robotyShift;
    robot(i)=fill(robotx(i,:),roboty(i,:),colorArray(i)); % ��������
    set(robot(i),{'LineStyle'},{'none'}); % �ѱ�ȥ��
%     robotLabel(i) = text(x(i,1)+0.1,y(i,1)+0.1,num2str(i),'color','white'); % �ӻ����˱��
    robotGoal(i)=fill(path{i}(end,2)+robotGoalxShift,path{i}(end,1)+robotGoalyShift,colorArray(i)); % �յ�
    set(robotGoal(i),{'LineStyle'},{'none'});
end
indexMap = map2Index( map );
indexMap(find(map==0)) = 0;
jobxShift = [0.3,0.45,0.5,0.55,0.7,0.55,0.5,0.45];
jobyShift = [0.5,0.45,0.3,0.45,0.5,0.55,0.7,0.55];
if ~isempty(noNameNodes) % �����񻭳��Ľ��ǣ�ȫ����������ʱ
    jobNum = size(assignNodes,1);
    jobBase = zeros(jobNum,2); % �����׼λ��
    for i = 1:jobNum
        jobBase(i,:) = index2Subscript( indexMap, assignNodes(i,2) );
    end
    jobINum = zeros(1,robotNum);
    for i = 1:robotNum
        jobINum(i) = size(find(assignNodes(:,1)==robotStartNodes(i)),1);
    end
    for i = 1:jobNum
        jobx(i,:) = jobBase(i,2)+jobxShift;
        joby(i,:) = jobBase(i,1)+jobyShift;
        job(i)=fill(jobx(i,:),joby(i,:),colorArray(7));
    end
else % ȫ�Ƿ���������ʱ
    jobNum = size(assignNodes,1);
    jobBase = zeros(jobNum,2);
    for i = 1:jobNum
        jobBase(i,:) = index2Subscript( indexMap, assignNodes(i,2) );
    end
    jobINum = zeros(1,robotNum);
    for i = 1:robotNum
        jobINum(i) = size(find(assignNodes(:,1)==robotStartNodes(i)),1);
        jobEndLine = sum(jobINum(1:i))-jobINum(i);
        for j = 1:jobINum(i)
            jobx(jobEndLine+j,:) = jobBase(jobEndLine+j,2)+jobxShift;
            joby(jobEndLine+j,:) = jobBase(jobEndLine+j,1)+jobyShift;
            job(jobEndLine+j)=fill(jobx(jobEndLine+j,:),joby(jobEndLine+j,:),colorArray(i));
            set(job(jobEndLine+j),{'LineStyle'},{'none'}); % �ѱ�ȥ��
            jobLabel(jobEndLine+j) = text(jobx(jobEndLine+j,1)+0.12,joby(jobEndLine+j,1),num2str(j),'color','black','FontSize',15);
        end
    end
end
jobInPath = zeros(1,jobNum); % �������path�е�λ��
for i = 1:robotNum
    jobEndLine = sum(jobINum(1:i))-jobINum(i);
    assignNodesI = assignNodes(find(assignNodes(:,1)==robotStartNodes(i)),2)';
    subscriptANI = zeros(size(assignNodesI,2),2);
    for j = 1:size(assignNodesI,2)
        subscriptANI(j,:) = index2Subscript( indexMap, assignNodesI(j) );
        ANIlines = intersect((find(path{i}(:,1)==subscriptANI(j,1))),(find(path{i}(:,2)==subscriptANI(j,2))));
        for k = 1:size(ANIlines,1)
            if ANIlines(k) > jobInPath(jobEndLine+1:jobEndLine+j)
                jobInPath(jobEndLine+j) = ANIlines(k);
                break
            end
        end
    end
end          
pause(pauseTime)
for time = 2:timeEnd
    for j = 1:refreshNum
        for i = 1:robotNum
            robotx(i,:) = path{i}(time-1,2)+(path{i}(time,2)-path{i}(time-1,2))/refreshNum*j+robotxShift;
            roboty(i,:) = path{i}(time-1,1)+(path{i}(time,1)-path{i}(time-1,1))/refreshNum*j+robotyShift;
            set(robot(i),'xdata',robotx(i,:));
            set(robot(i),'ydata',roboty(i,:));
%             delete(robotLabel(i));
%             robotLabel(i) = text(x(i,1)+0.1,y(i,1)+0.1,num2str(i),'color','white');
            pause(pauseTime)
        end
    end
    jobReached = find(jobInPath==time);
    if ~isempty(jobReached) % �ѵ��������ɾ��
        for i = 1:size(jobReached,2)
            delete(job(jobReached(i)));
            if isempty(noNameNodes)
                delete(jobLabel(jobReached(i)));
            end
        end
    end
end
end