from heapq import heappop, heappush
import math
import gtpyhop

# Define domain
the_domain = gtpyhop.Domain('pixel_domain')
gtpyhop.current_domain = the_domain

def is_collision(x, y, obstacles):
    """Check if the position (x, y) is inside any obstacle."""
    for obs in obstacles:
        if obs['x_min'] <= x <= obs['x_max'] and obs['y_min'] <= y <= obs['y_max']:
            return True
    return False

def move_right(s):
    x, y = s.agent_position
    if x + 1 <= s.world_size[1] and not is_collision(x + 1, y, s.obstacles):
        s.agent_position = (x + 1, y)
        return s
    return None

def move_left(s):
    x, y = s.agent_position
    if x - 1 >= s.world_size[0] and not is_collision(x - 1, y, s.obstacles):
        s.agent_position = (x - 1, y)
        return s
    return None

def move_up(s):
    x, y = s.agent_position
    if y + 1 <= s.world_size[3] and not is_collision(x, y + 1, s.obstacles):
        s.agent_position = (x, y + 1)
        return s
    return None

def move_down(s):
    x, y = s.agent_position
    if y - 1 >= s.world_size[2] and not is_collision(x, y - 1, s.obstacles):
        s.agent_position = (x, y - 1)
        return s
    return None

gtpyhop.declare_actions(move_up, move_down, move_left, move_right)


def heuristic(pos, goal):
    """Heuristic function: Euclidean distance to the goal."""
    return math.sqrt((pos[0] - goal[0])**2 + (pos[1] - goal[1])**2)

def neighbors(pos, world_size, obstacles):
    """Generate neighboring positions that are valid moves."""
    x, y = pos
    possible_moves = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
    valid_moves = []
    for nx, ny in possible_moves:
        if world_size[0] <= nx <= world_size[1] and world_size[2] <= ny <= world_size[3]:
            if not is_collision(nx, ny, obstacles):
                valid_moves.append((nx, ny))
    return valid_moves

def a_star_search(start, goal, world_size, obstacles):
    """Perform A* search from start to goal."""
    open_set = []
    heappush(open_set, (0 + heuristic(start, goal), 0, start, []))
    visited = set()
    
    while open_set:
        _, cost, current, path = heappop(open_set)
        
        if current in visited:
            continue
        visited.add(current)
        path = path + [current]
        
        if current == goal:
            return path  # Return the path found
        
        for neighbor in neighbors(current, world_size, obstacles):
            if neighbor not in visited:
                new_cost = cost + 1  # Assuming uniform cost for movement
                heappush(open_set, (new_cost + heuristic(neighbor, goal), new_cost, neighbor, path))
    
    return None  # No path found

def move_to_goal_with_a_star(s, goal_position):
    """Method using A* to find a path to the goal."""
    start = s.agent_position
    path = a_star_search(start, goal_position, s.world_size, s.obstacles)
    
    if path:
        subtasks = []
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            if x2 > x1:
                subtasks.append(('move_right',))
            elif x2 < x1:
                subtasks.append(('move_left',))
            elif y2 > y1:
                subtasks.append(('move_up',))
            elif y2 < y1:
                subtasks.append(('move_down',))
        return subtasks
    return None  # No path found

gtpyhop.declare_task_methods('reach_goal', move_to_goal_with_a_star)


pixel_world = gtpyhop.State('pixel_world', 
    agent_position=(0, 0), 
    world_size=(-1, 6, -1, 1),  # x_min, x_max, y_min, y_max
    obstacles=[
        {'x_min': 1, 'x_max': 1.25, 'y_min': 0, 'y_max': 1},
        {'x_min': 4.75, 'x_max': 5, 'y_min': -0.5, 'y_max': 0.5}
    ]
)
goal1a = gtpyhop.Multigoal('goal1a', agent_position=(5.5, 0))

gtpyhop.verbose = 3
plan = gtpyhop.find_plan(pixel_world, [('reach_goal', (5.5, 0))])
print(plan)
