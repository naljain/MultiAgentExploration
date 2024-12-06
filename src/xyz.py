import heapq
from collections import deque

class Node:
    def __init__(self, x, y, g, h, parent=None):
        self.x = x
        self.y = y
        self.g = g  # Cost from start to current node
        self.h = h  # Heuristic cost from current node to goal
        self.f = g + h
        self.parent = parent

    def __lt__(self, other):
        return self.f < other.f

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def explore_a_star(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    open_list = []
    closed_set = set()
    start_node = Node(*start, 0, heuristic(start, goal))
    heapq.heappush(open_list, start_node)

    while open_list:
        current = heapq.heappop(open_list)
        if (current.x, current.y) == goal:
            path = []
            while current:
                path.append((current.x, current.y))
                current = current.parent
            return path[::-1]  # Return the path in correct order

        closed_set.add((current.x, current.y))

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = current.x + dx, current.y + dy
            if 0 <= nx < rows and 0 <= ny < cols and (nx, ny) not in closed_set:
                if grid[nx][ny] == 0:  # Only move to free cells
                    neighbor = Node(nx, ny, current.g + 1, heuristic((nx, ny), goal), current)
                    heapq.heappush(open_list, neighbor)

    return None  # Return None if no path is found

def multi_agent_exploration(grid, starts):
    rows, cols = len(grid), len(grid[0])
    agents = len(starts)
    known_map = [[-1 for _ in range(cols)] for _ in range(rows)]  # -1 means unexplored
    visited = [[False for _ in range(cols)] for _ in range(rows)]

    paths = [[] for _ in range(agents)]
    positions = starts[:]

    def is_fully_explored():
        return all(all(cell != -1 for cell in row) for row in known_map)

    def can_reach_unexplored():
        for x in range(rows):
            for y in range(cols):
                if known_map[x][y] == -1:  # Unexplored cell
                    for i in range(agents):
                        if explore_a_star(grid, positions[i], (x, y)):
                            return True
        return False

    step = 0
    while not is_fully_explored():
        if not can_reach_unexplored():
            print("Exploration halted: no reachable unexplored cells.")
            break

        for i in range(agents):
            x, y = positions[i]

            # Update the known map and mark visited
            known_map[x][y] = grid[x][y]
            visited[x][y] = True

            # Find the nearest unexplored cell
            queue = deque([(x, y, 0)])  # (x, y, distance)
            target = None
            visited_local = set()
            while queue:
                cx, cy, dist = queue.popleft()
                if (cx, cy) in visited_local:
                    continue
                visited_local.add((cx, cy))

                # Check if unexplored
                if known_map[cx][cy] == -1:
                    target = (cx, cy)
                    break

                # Add neighbors
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < rows and 0 <= ny < cols and (nx, ny) not in visited_local:
                        queue.append((nx, ny, dist + 1))

            if target:
                path = explore_a_star(grid, (x, y), target)
                if path:
                    paths[i].extend(path[1:])  # Skip current position
                    positions[i] = path[-1]

        step += 1
        if step > rows * cols * agents * 10:  # Break if exploration is too slow
            print("Exploration halted due to timeout.")
            break

    return paths, known_map



# Example Usage
if __name__ == "__main__":
    # 0 = Free space, 1 = Obstacle
    grid = [
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 1, 0, 0],
    ]
    starts = [(0, 0), (3, 0)]  # Starting positions of agents

    paths, final_map = multi_agent_exploration(grid, starts)
    print("\nExploration Paths:")
    for i, path in enumerate(paths):
        print(f"Agent {i}: {path}")
    print("\nFinal Known Map:")
    for row in final_map:
        print(row)
