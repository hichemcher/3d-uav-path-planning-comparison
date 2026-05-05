function path = Astar3D(start, goal, maze, safety_margin)
    % Define the dimensions of the maze
    [numRows, numCols, numLayers] = size(maze);

    % Define the heuristic function as Euclidean distance
    heuristic = @(x, y) norm(x - y);

    % Define the cost function
    cost = @(x, y) 1;

    % Initialize the queue to store the nodes to be explored
    openSet = {start};

    % Initialize the map of g-scores (the cost to reach each node)
    gScores = Inf(numRows, numCols, numLayers);
    gScores(start(1), start(2), start(3)) = 0;

    % Initialize the map of f-scores (the estimated cost from start to goal)
    fScores = Inf(numRows, numCols, numLayers);
    fScores(start(1), start(2), start(3)) = heuristic(start, goal);

    % Initialize the map of came_from (the previous node in the shortest path)
    cameFrom = zeros(numRows, numCols, numLayers, 3);

    while ~isempty(openSet)
        % Get the node with the lowest f-score from the open set
        currentIdx = findLowestFScore(openSet, fScores);
        current = openSet{currentIdx};
        openSet(currentIdx) = [];

        % Check if the current node is the goal
        if isequal(current, goal)
            % Reconstruct the path
            path = reconstructPath(cameFrom, goal);
            return;
        end

        % Get the neighbors of the current node
        neighbors = getNeighbors3D(current, numRows, numCols, numLayers, maze);

        mazeSize = size(maze);
        for i = 1:size(neighbors, 1)
            neighbor = neighbors(i, :);
            newRow = neighbor(1);
            newCol = neighbor(2);
            newLayer = neighbor(3);

            margin_range_y = max(1, newCol - safety_margin):min(mazeSize(2), newCol + safety_margin);
            margin_range_x = max(1, newRow - safety_margin):min(mazeSize(1), newRow + safety_margin);
            margin_range_z = max(1, newLayer - safety_margin):min(mazeSize(3), newLayer + safety_margin);

            if all(all(all(maze(margin_range_x, margin_range_y, margin_range_z) == 0)))
                tentativeGScore = gScores(current(1), current(2), current(3)) + cost(current, neighbor);

                if tentativeGScore < gScores(neighbor(1), neighbor(2), neighbor(3))
                    % Update the path to this neighbor
                    cameFrom(neighbor(1), neighbor(2), neighbor(3), :) = current;

                    gScores(neighbor(1), neighbor(2), neighbor(3)) = tentativeGScore;
                    fScores(neighbor(1), neighbor(2), neighbor(3)) = tentativeGScore + heuristic(neighbor, goal);

                    % Add the neighbor to the open set if it is not already present
                    if ~isContain(openSet, neighbor)
                        openSet{end+1} = neighbor;
                    end
                end
            end
        end
    end

    % If we reach here, it means that no path to the goal exists
    path = [];
    disp('No path found');
end

function path = reconstructPath(cameFrom, goal)
    % Reconstruct the path from the cameFrom map
    path = {};
    current = goal;

    while ~isequal(current, [0, 0, 0])
        path{end+1} = current;
        current = squeeze(cameFrom(current(1), current(2), current(3), :))';
    end

    path = flipud(cell2mat(path'));
end

function neighbors = getNeighbors3D(node, numRows, numCols, numLayers, maze)
    % Get the valid neighbors of a node within the bounds of the maze
    neighbors = [];

    row = node(1);
    col = node(2);
    layer = node(3);

    % Define the possible direction changes (26-connected grid)
    [X, Y, Z] = ndgrid(-1:1, -1:1, -1:1);
    directions = [X(:), Y(:), Z(:)];
    directions(ismember(directions, [0 0 0], 'rows'), :) = [];  % Remove [0 0 0] move

    for dir = directions'
        newRow = row + dir(1);
        newCol = col + dir(2);
        newLayer = layer + dir(3);

        % Check if the new coordinates are within the maze bounds
        if newRow >= 1 && newRow <= numRows && newCol >= 1 && newCol <= numCols && newLayer >= 1 && newLayer <= numLayers
            if maze(newRow, newCol, newLayer) ~= 1
                neighbors = [neighbors; newRow, newCol, newLayer];
            end
        end
    end
end

function contains = isContain(cellArray, element)
    % Check if an element is present in a cell array
    contains = false;

    for i = 1:numel(cellArray)
        if isequal(cellArray{i}, element)
            contains = true;
            return;
        end
    end
end

function idx = findLowestFScore(openSet, fScores)
    % Find the index of the node with the lowest f-score in the open set
    minFScore = Inf;
    idx = 1;

    for i = 1:numel(openSet)
        node = openSet{i};
        fScore = fScores(node(1), node(2), node(3));

        if fScore < minFScore
            minFScore = fScore;
            idx = i;
        end
    end
end
