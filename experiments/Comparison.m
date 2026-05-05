clc; clear; close all;
% Author: H.CHERIET USTO-MB

% ---- Map list (put your filenames here)
currentFile = mfilename('fullpath');
[scriptDir,~,~] = fileparts(currentFile);

projectRoot = fullfile(scriptDir, '..');

mapFiles = {
    fullfile(projectRoot,"src","maps","mapsparse.mat")
    fullfile(projectRoot,"src","maps","mapdense.mat")
    fullfile(projectRoot,"src","maps","mapnormal.mat")
    fullfile(projectRoot,"src","maps","maplarge.mat")
};

% ---- Start/Goal (can also vary per map if needed)
start = [1 1 5];
goal  = [98 96 7];

results = struct();

for m = 1:length(mapFiles)

    fprintf("\n===== Running Map %d =====\n", m);

    data = load(mapFiles{m});
    omap3D = data.omap3D;
    maze3d = data.maze3d;

    figure('Name', mapFiles{m});
    show(omap3D); hold on;

    %% ---- A*
    t1 = tic;
    pathA = Astar3D(start, goal, maze3d, 1);
    timeA = toc(t1);

    %% ---- RRT*
    t2 = tic;
    [pthObj,~,~] = RRTStar(start, goal, data);
    pathR = pthObj.States;
    timeR = toc(t2);

    %% ---- PSO
    t3 = tic;
    sol = PSO(start, goal, data);
    pathP = [start; sol.x' sol.y' sol.z'; goal];
    timeP = toc(t3);

    %% ---- Metrics
    LA = path_length(pathA);
    LR = path_length(pathR);
    LP = path_length(pathP);

    TA = turning_radius(pathA);
    TR = turning_radius(pathR);
    TP = turning_radius(pathP);

    fprintf("A*:   L=%.2f  T=%.2f  Time=%.3f\n", LA, TA, timeA);
    fprintf("RRT*: L=%.2f  T=%.2f  Time=%.3f\n", LR, TR, timeR);
    fprintf("PSO:  L=%.2f  T=%.2f  Time=%.3f\n", LP, TP, timeP);

    %% ---- Store results
    results(m).map = mapFiles{m};

    results(m).Astar.length = LA;
    results(m).Astar.turn   = TA;
    results(m).Astar.time   = timeA;

    results(m).RRT.length = LR;
    results(m).RRT.turn   = TR;
    results(m).RRT.time   = timeR;

    results(m).PSO.length = LP;
    results(m).PSO.turn   = TP;
    results(m).PSO.time   = timeP;

    %% ---- Plot
    plot3(pathA(:,1),pathA(:,2),pathA(:,3),'c','LineWidth',2)
    plot3(pathR(:,1),pathR(:,2),pathR(:,3),'g','LineWidth',2)
    plot3(pathP(:,1),pathP(:,2),pathP(:,3),'r','LineWidth',2)

    scatter3(start(1),start(2),start(3),100,'b','filled')
    scatter3(goal(1),goal(2),goal(3),100,'m','filled')

    legend('','A*','RRT*','PSO')
    grid on;

end

%% ---- Save results
save('results_all_maps.mat','results');


lengths = [];
times   = [];
turns   = [];

for i=1:length(results)
    lengths = [lengths;
        results(i).Astar.length ...
        results(i).RRT.length ...
        results(i).PSO.length];

    times = [times;
        results(i).Astar.time ...
        results(i).RRT.time ...
        results(i).PSO.time];

    turns = [turns;
        results(i).Astar.turn ...
        results(i).RRT.turn ...
        results(i).PSO.turn];
end

figure; bar(lengths); title('Path Length'); legend('A*','RRT*','PSO');
figure; bar(turns);   title('Turning Radius'); legend('A*','RRT*','PSO');
figure; bar(times);   title('Time'); legend('A*','RRT*','PSO');