function [pthObj,elapsedTime_RRTStar,omap3D] = RRTStar(start,goal,mapData)
start_time = tic;

omap3D = mapData.omap3D;
omap3D.FreeThreshold = 0.65;
inflate(omap3D,1)

ss = stateSpaceSE3([1 size(mapData.maze3d,1);1 size(mapData.maze3d,2);1 size(mapData.maze3d,3);inf inf;inf inf;inf inf;inf inf]);
sv = validatorOccupancyMap3D(ss, ...
     Map = omap3D, ...
     ValidationDistance = 0.1);
planner = plannerRRTStar(ss, sv, ...
          'MaxConnectionDistance', 5, ... % Increase connection distance if needed
          'MaxIterations', 5000, ... % Increase the number of iterations
          'GoalReachedFcn', @(~,s,g)(norm(s(1:3)-g(1:3))<1), ...
          'GoalBias', 0.1); % Increase goal bias for more focused goal sampling


%planner.MaxConnectionDistance = 3;
%planner.MaxNumTreeNodes = 4000;
start2 = [start 1 0 0 0];
goal2 = [goal 1 0 0 0 ];
%disp(start2);

rng(1,'twister') % repeatable result
[pthObj,solnInfo] = plan(planner,start2,goal2);

 % figure;
  %plot3(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),solnInfo.TreeData(:,3),'.-')
elapsedTime_RRTStar = toc(start_time);


%show(map)
%hold on
%plot(start(1), start(2), 'ro', 'MarkerSize', 10);
%plot(goal(1), goal(2), 'go', 'MarkerSize', 10);
% Tree expansion
%plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-')
% Draw path
%plot(pthObj.States(:,1),pthObj.States(:,2),'b-','LineWidth',2)











end