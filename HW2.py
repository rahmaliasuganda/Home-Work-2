import heapq
import time

# Representasi grid kota
grid = [
    ['S', '.', '.', 'T', '.'],
    ['.', 'T', '.', 'T', '.'],
    ['.', '.', '.', '.', '.'],
    ['T', 'T', '.', 'T', '.'],
    ['.', '.', '.', '.', 'H']
]

rows = len(grid)
cols = len(grid[0])

# Arah gerakan: atas, bawah, kiri, kanan
directions = [(-1,0),(1,0),(0,-1),(0,1)]

# Cari posisi S dan H
def find_positions(grid):
    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == 'S':
                start = (r, c)
            elif grid[r][c] == 'H':
                goal = (r, c)
    return start, goal

# Heuristik Manhattan
def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# GBFS
def gbfs(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (manhattan(start, goal), start))
    came_from = {}
    visited = set()
    visited_nodes = 0

    while open_set:
        _, current = heapq.heappop(open_set)
        visited_nodes += 1

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path, visited_nodes

        visited.add(current)

        for d in directions:
            nr, nc = current[0] + d[0], current[1] + d[1]
            neighbor = (nr, nc)
            if 0 <= nr < rows and 0 <= nc < cols and neighbor not in visited:
                if grid[nr][nc] != 'T':
                    came_from[neighbor] = current
                    heapq.heappush(open_set, (manhattan(neighbor, goal), neighbor))
                    visited.add(neighbor)

    return None, visited_nodes

# A* 
def a_star(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    visited_nodes = 0

    while open_set:
        _, current = heapq.heappop(open_set)
        visited_nodes += 1

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path, visited_nodes

        for d in directions:
            nr, nc = current[0] + d[0], current[1] + d[1]
            neighbor = (nr, nc)
            if 0 <= nr < rows and 0 <= nc < cols:
                if grid[nr][nc] != 'T':
                    tentative_g = g_score.get(current, float('inf')) + 1
                    if tentative_g < g_score.get(neighbor, float('inf')):
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f = tentative_g + manhattan(neighbor, goal)
                        heapq.heappush(open_set, (f, neighbor))

    return None, visited_nodes

# Jalankan dan bandingkan
start, goal = find_positions(grid)

# GBFS
start_time = time.time()
gbfs_path, gbfs_nodes = gbfs(grid, start, goal)
gbfs_time = (time.time() - start_time) * 1000

# A*
start_time = time.time()
astar_path, astar_nodes = a_star(grid, start, goal)
astar_time = (time.time() - start_time) * 1000

# Visualisasi jalur pada grid
def print_grid(grid, path):
    display = [row[:] for row in grid]
    for r, c in path:
        if display[r][c] not in ('S', 'H'):
            display[r][c] = '*'
    for row in display:
        print(' '.join(row))

print("\n=== GBFS Path ===")
print_grid(grid, gbfs_path)

print("\n=== A* Path ===")
print_grid(grid, astar_path)

# Hasil perbandingan
print("\n=== COMPARISON BASED ON EXECUTION TIME (ms) ===")
print(f"{'Algoritma':<10} | {'Time (ms)':>10}")
print(f"{'-'*25}")
print(f"{'GBFS':<10} | {gbfs_time:>10.3f}")
print(f"{'A*':<10} | {astar_time:>10.3f}")

print("\n=== COMPARISON BASED ON NUMBER OF NODES VISITED  ===")
print(f"{'Algoritma':<10} | {'Node':>18}")
print(f"{'-'*32}")
print(f"{'GBFS':<10} | {gbfs_nodes:>18}")
print(f"{'A*':<10} | {astar_nodes:>18}")
