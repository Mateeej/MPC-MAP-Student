function [path] = astar(read_only_vars, public_vars)
%ASTAR A* path planning with adaptive clearance

grid    = read_only_vars.discrete_map.map;
dims    = read_only_vars.discrete_map.dims;
limits  = read_only_vars.discrete_map.limits;
goal_gc = read_only_vars.discrete_map.goal;

COLS = dims(1);
ROWS = dims(2);

xMin  = limits(1); yMin = limits(2);
xMax  = limits(3); yMax = limits(4);
xRange = xMax - xMin;
yRange = yMax - yMin;

% ---- Convert start pose to grid coords ----
pose = public_vars.estimated_pose;

start_col = round((pose(1) - xMin) / xRange * (COLS-1)) + 1;
start_row = round((pose(2) - yMin) / yRange * (ROWS-1)) + 1;
start_col = max(1, min(COLS, start_col));
start_row = max(1, min(ROWS, start_row));

goal_col = goal_gc(1);
goal_row = goal_gc(2);

% ---- Try A* with decreasing clearance ----
found = false;
grid_path = [];

for min_clearance = [2, 1, 0]
    cost_map = make_cost_map(grid, 8, min_clearance);

    % Debug vizualizacia
    figure(99); imagesc(cost_map); colorbar;
    title(sprintf('Cost map clearance=%d', min_clearance));
    drawnow; pause(0.5);

    if isinf(cost_map(start_row, start_col)) || isinf(cost_map(goal_row, goal_col))
        fprintf('Clearance %d: start or goal blocked\n', min_clearance);
        continue;
    end
    [found, grid_path] = run_astar(cost_map, ROWS, COLS, start_row, start_col, goal_row, goal_col);
    if found
        fprintf('Used clearance: %d\n', min_clearance);
        break;
    else
        fprintf('Clearance %d: no path found\n', min_clearance);
    end
end

if ~found || isempty(grid_path)
    path = public_vars.path;
    return;
end

% ---- Convert to world coords ----
path_x = (grid_path(:,2) - 1) / (COLS-1) * xRange + xMin;
path_y = (grid_path(:,1) - 1) / (ROWS-1) * yRange + yMin;
path   = [path_x, path_y];

% path = smooth_path(path);

end

% ---- Run A* on cost map ----
function [found, grid_path] = run_astar(cost_map, ROWS, COLS, start_row, start_col, goal_row, goal_col)

INF = 1e9;
g      = INF * ones(ROWS, COLS);
f      = INF * ones(ROWS, COLS);
parent = zeros(ROWS, COLS, 2);

g(start_row, start_col) = 0;
f(start_row, start_col) = heuristic(start_row, start_col, goal_row, goal_col);

open_list = [f(start_row, start_col), start_row, start_col];
closed    = false(ROWS, COLS);

neighbors = [-1,-1; -1,0; -1,1;
              0,-1;        0,1;
             1,-1;  1,0;  1,1];

found = false;

while ~isempty(open_list)
    [~, idx] = min(open_list(:,1));
    current  = open_list(idx, :);
    open_list(idx, :) = [];

    cr = current(2);
    cc = current(3);

    if closed(cr, cc); continue; end
    closed(cr, cc) = true;

    if cr == goal_row && cc == goal_col
        found = true;
        break;
    end

    for i = 1:8
        nr = cr + neighbors(i,1);
        nc = cc + neighbors(i,2);

        if nr < 1 || nr > ROWS || nc < 1 || nc > COLS; continue; end
        if isinf(cost_map(nr, nc)) || closed(nr, nc); continue; end

        if neighbors(i,1) ~= 0 && neighbors(i,2) ~= 0
            step = sqrt(2) * cost_map(nr, nc);
        else
            step = 1.0 * cost_map(nr, nc);
        end

        new_g = g(cr, cc) + step;
        if new_g < g(nr, nc)
            g(nr, nc) = new_g;
            f(nr, nc) = new_g + heuristic(nr, nc, goal_row, goal_col);
            parent(nr, nc, :) = [cr, cc];
            open_list = [open_list; f(nr,nc), nr, nc];
        end
    end
end

if ~found
    grid_path = [];
    return;
end

grid_path = [goal_row, goal_col];
r = goal_row; c = goal_col;
while ~(r == start_row && c == start_col)
    pr = parent(r, c, 1);
    pc = parent(r, c, 2);
    grid_path = [pr, pc; grid_path];
    r = pr; c = pc;
end



end

% ---- Heuristic ----
function h = heuristic(r, c, goal_r, goal_c)
    h = sqrt((r - goal_r)^2 + (c - goal_c)^2);
end

% ---- Cost map with adaptive clearance ----
function cost = make_cost_map(grid, max_dist, min_clearance)
    [rows, cols] = size(grid);
    border_grid = grid;
    border_grid(1,:) = 1; border_grid(end,:) = 1;
    border_grid(:,1) = 1; border_grid(:,end) = 1;

    cost = ones(rows, cols);
    cost(border_grid > 0) = Inf;

    [wall_rows, wall_cols] = find(border_grid > 0);

    for r = 1:rows
        for c = 1:cols
            if isinf(cost(r,c)); continue; end
            dists = sqrt((wall_rows - r).^2 + (wall_cols - c).^2);
            min_dist = min(dists);
            if min_dist < min_clearance
                cost(r,c) = Inf;
            elseif min_dist < max_dist
                cost(r,c) = 1 + (max_dist - min_dist) / max_dist * 500;
            end
        end
    end
end