function [path] = astar(read_only_vars, public_vars)
%ASTAR A* path planning on occupancy grid with clearance

grid    = read_only_vars.discrete_map.map;
dims    = read_only_vars.discrete_map.dims;   % [cols, rows] = [101, 76]
limits  = read_only_vars.discrete_map.limits; % [xMin yMin xMax yMax]
goal_gc = read_only_vars.discrete_map.goal;   % [col, row] in grid coords

COLS = dims(1);  % 101
ROWS = dims(2);  % 76

% ---- Task 2: Clearance — inflate obstacles ----
CLEARANCE_CELLS = 3;  % ~0.6m clearance (2 cells * 0.2m/cell)
inflated = manual_dilate(grid, CLEARANCE_CELLS);

% ---- Convert start pose to grid coords ----
pose    = public_vars.estimated_pose;
xMin    = limits(1); yMin = limits(2);
xMax    = limits(3); yMax = limits(4);
xRange  = xMax - xMin;
yRange  = yMax - yMin;

start_col = round((pose(1) - xMin) / xRange * (COLS-1)) + 1;
start_row = round((pose(2) - yMin) / yRange * (ROWS-1)) + 1;
start_col = max(1, min(COLS, start_col));
start_row = max(1, min(ROWS, start_row));

goal_col = goal_gc(1);
goal_row = goal_gc(2);

% ---- Check if start or goal is in obstacle ----
if inflated(start_row, start_col) || inflated(goal_row, goal_col)
    path = public_vars.path;
    return;
end

% ---- A* implementation ----
% Each node: [row, col]
% g = cost from start, h = heuristic to goal, f = g + h

INF = 1e9;
g   = INF * ones(ROWS, COLS);
f   = INF * ones(ROWS, COLS);
parent = zeros(ROWS, COLS, 2);  % parent [row, col]

g(start_row, start_col) = 0;
h_start = heuristic(start_row, start_col, goal_row, goal_col);
f(start_row, start_col) = h_start;

% Open list: [f, row, col]
open_list = [f(start_row, start_col), start_row, start_col];
closed    = false(ROWS, COLS);

% 8-connected neighbors
neighbors = [-1,-1; -1,0; -1,1;
              0,-1;        0,1;
              1,-1;  1,0;  1,1];
diag_cost = sqrt(2);
straight_cost = 1;

found = false;

while ~isempty(open_list)
    % Pick node with lowest f
    [~, idx] = min(open_list(:,1));
    current  = open_list(idx, :);
    open_list(idx, :) = [];

    cr = current(2);
    cc = current(3);

    if closed(cr, cc)
        continue;
    end
    closed(cr, cc) = true;

    % Goal reached
    if cr == goal_row && cc == goal_col
        found = true;
        break;
    end

    % Expand neighbors
    for i = 1:8
        nr = cr + neighbors(i,1);
        nc = cc + neighbors(i,2);

        if nr < 1 || nr > ROWS || nc < 1 || nc > COLS
            continue;
        end
        if inflated(nr, nc) || closed(nr, nc)
            continue;
        end

        % Step cost
        if neighbors(i,1) ~= 0 && neighbors(i,2) ~= 0
            step = diag_cost;
        else
            step = straight_cost;
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
    path = public_vars.path;
    return;
end

% ---- Reconstruct path in grid coords ----
grid_path = [goal_row, goal_col];
r = goal_row; c = goal_col;
while ~(r == start_row && c == start_col)
    pr = parent(r, c, 1);
    pc = parent(r, c, 2);
    grid_path = [pr, pc; grid_path];
    r = pr; c = pc;
end

% ---- Convert grid coords back to world coords ----
path_x = (grid_path(:,2) - 1) / (COLS-1) * xRange + xMin;
path_y = (grid_path(:,1) - 1) / (ROWS-1) * yRange + yMin;
path   = [path_x, path_y];

end

function h = heuristic(r, c, goal_r, goal_c)
    % Euclidean heuristic
    h = sqrt((r - goal_r)^2 + (c - goal_c)^2);
end

function out = manual_dilate(grid, r)
    out = grid;
    [rows, cols] = size(grid);
    for row = 1:rows
        for col = 1:cols
            if grid(row, col)
                for dr = -r:r
                    for dc = -r:r
                        if dr^2 + dc^2 <= r^2
                            nr = row+dr; nc = col+dc;
                            if nr>=1 && nr<=rows && nc>=1 && nc<=cols
                                out(nr, nc) = 1;
                            end
                        end
                    end
                end
            end
        end
    end
end