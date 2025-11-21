# visualize_astar.py
import re
import sys


def parse_output(output_text):
    """解析A*搜索输出"""
    data = {
        'width': 0,
        'height': 0,
        'start': 0,
        'goal': 0,
        'obstacles': [],
        'node_data': {},
        'found': False,
        'total_edges': 0
    }

    lines = output_text.strip().split('\n')

    for line in lines:
        if line.startswith('GRID_INFO:'):
            match = re.match(r'GRID_INFO: (\d+) (\d+)', line)
            if match:
                data['width'] = int(match.group(1))
                data['height'] = int(match.group(2))

        elif line.startswith('START_NODE:'):
            data['start'] = int(line.split()[1])

        elif line.startswith('GOAL_NODE:'):
            data['goal'] = int(line.split()[1])

        elif line.startswith('OBSTACLES:'):
            obstacles = line.split()[1:]
            data['obstacles'] = [int(x) for x in obstacles]

        elif line.startswith('TOTAL_EDGES:'):
            data['total_edges'] = int(line.split()[1])

        elif line.startswith('FOUND:'):
            data['found'] = int(line.split()[1]) != 0

        elif line.startswith('NODE_DATA:'):
            parts = line.split()
            node_id = int(parts[1])
            cost = int(parts[2])
            prev = int(parts[3])
            data['node_data'][node_id] = {'cost': cost, 'prev': prev}

    return data


def reconstruct_path(data):
    """重构路径"""
    if not data['found']:
        return []

    path = []
    current = data['goal']

    # 检查目标节点是否可达
    if data['goal'] not in data['node_data'] or data['node_data'][data['goal']]['cost'] == 0:
        return []

    # 从目标回溯到起点
    while current != -1:
        path.append(current)
        if current == data['start']:
            break

        prev_node = data['node_data'][current]['prev']
        if prev_node == current or prev_node in path:
            break

        current = prev_node

    path.reverse()

    if path and path[0] != data['start']:
        return []

    return path


def visualize_astar(data):
    """可视化A*搜索结果"""
    width = data['width']
    height = data['height']
    path = reconstruct_path(data)

    print("\n" + "=" * 50)
    print("A* SEARCH RESULTS")
    print("=" * 50)
    print(f"Grid: {width}x{height}")
    print(f"Start: Node {data['start']}")
    print(f"Goal: Node {data['goal']}")
    print(f"Obstacles: {len(data['obstacles'])} nodes")
    print(f"Total graph edges: {data['total_edges']}")
    print(f"Path found: {data['found']}")
    print()

    # 网格显示
    print("Grid Visualization:")
    print("Legend: S=Start, G=Goal, X=Obstacle, *=Path, .=Free")
    print()

    grid = []
    for y in range(height):
        row = []
        for x in range(width):
            node_id = y * width + x
            if node_id == data['start']:
                row.append('S')
            elif node_id == data['goal']:
                row.append('G')
            elif node_id in data['obstacles']:
                row.append('X')
            elif node_id in path:
                row.append('*')
            else:
                row.append('.')
        grid.append(row)

    for row in grid:
        print(' '.join(row))
    print()

    # 显示路径信息
    if path:
        print(f"Optimal Path Found! Length: {len(path)} steps")
        print(f"Path: {' -> '.join(map(str, path))}")

        print("\nPath Analysis:")
        for i, node in enumerate(path):
            y = node // width
            x = node % width
            node_cost = data['node_data'][node]['cost']
            print(f"  Step {i:2d}: Node {node:2d} ({x},{y}) | Cost: {node_cost:2d}")

        if data['goal'] in data['node_data']:
            total_cost = data['node_data'][data['goal']]['cost']
            print(f"\n📈 Summary:")
            print(f"  Total Path Cost: {total_cost}")
            print(f"  Path Length: {len(path)} steps")
            print(f"  Cost per Step: {total_cost / len(path):.2f}")

    # 节点可达性分析
    print("\nReachability Analysis:")
    reachable_nodes = len([node for node in data['node_data'] if data['node_data'][node]['cost'] > 0])
    unreachable_nodes = TOTAL_NODES - len(data['obstacles']) - reachable_nodes
    print(f"  Reachable nodes: {reachable_nodes}")
    print(f"  Unreachable nodes: {unreachable_nodes}")
    print(f"  Obstacle nodes: {len(data['obstacles'])}")


def main():
    if len(sys.argv) > 1:
        with open(sys.argv[1], 'r') as f:
            output_text = f.read()
    else:
        print("Paste the A* output:")
        output_text = sys.stdin.read()

    data = parse_output(output_text)
    visualize_astar(data)


if __name__ == "__main__":
    main()