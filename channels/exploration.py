from collections import deque

def BFS_for_nearest_unknown_cell(low_res_grid, new_width, new_height, start_x, start_y):
    directions = [(0, 1, 'right'), (0, -1, 'left'), (1, 0, 'down'), (-1, 0, 'up')]
    start_idx = start_y * new_width + start_x
    queue = deque([(start_x, start_y)])
    visited = set()
    visited.add(start_idx)
    while queue:
        x, y = queue.popleft()
        for dx, dy, action in directions:
            nx, ny = x + dx, y + dy
            nidx = ny * new_width + nx
            if 0 <= nx < new_width and 0 <= ny < new_height:
                if low_res_grid[nidx] == -1: #found unknown
                    return (nx, ny)
                if low_res_grid[nidx] == 0 and nidx not in visited:
                    queue.append((nx, ny))
                    visited.add(nidx)
    return None
