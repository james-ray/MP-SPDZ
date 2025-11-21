# visualize_potential_field.py
import re
import sys


def parse_output(output_text):
    """解析势场A*搜索输出"""
    data = {
        'width': 0,
        'height': 0,
        'start': 0,
        'goal': 0,
        'obstacles': [],
        'fire_zones': {},
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

        elif line.startswith('FIRE_REPULSION_ZONES:'):
            # 解析火警区域 "18:10 19:10 ..."
            zones_text = ' '.join(line.split()[1:])
            zone_matches = re.findall(r'(\d+):(\d+)', zones_text)
            for node_str, level_str in zone_matches:
                data['fire_zones'][int(node_str)] = int(level_str)

        elif line.startswith('TOTAL_EDGES:'):
            data['total_edges'] = int(line.split()[1])

        elif line.startswith('PATH_FOUND:'):
            data['found'] = int(line.split()[1]) != 0

        elif line.startswith('NODE_POTENTIAL:'):
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


def visualize_potential_field(data):
    """可视化势场A*搜索结果"""
    width = data['width']
    height = data['height']
    path = reconstruct_path(data)

    print("\n" + "=" * 60)
    print("POTENTIAL FIELD A* SEARCH RESULTS")
    print("=" * 60)
    print(f"Grid: {width}x{height}")
    print(f"Start: Node {data['start']} (Attraction Source)")
    print(f"Goal: Node {data['goal']} (Attraction Target)")
    print(f"Physical Obstacles: {data['obstacles']}")
    print(f"Fire Repulsion Zones: {len(data['fire_zones'])} areas")
    print(f"Total graph edges: {data['total_edges']}")
    print(f"Path found: {data['found']}")
    print()

    # 图1: 显示网格火警等级
    print("📊 GRID 1: Fire Risk Levels (0-10 scale)")
    print("Legend: S=Start, G=Goal, X=Obstacle, Number=FireRisk")
    print("        0=Safe, 1-3=Low, 4-7=Medium, 8-10=High")
    print()

    grid_risks = []
    for y in range(height):
        row = []
        for x in range(width):
            node_id = y * width + x
            if node_id == data['start']:
                row.append('  S ')
            elif node_id == data['goal']:
                row.append('  G ')
            elif node_id in data['obstacles']:
                row.append('  X ')
            elif node_id in data['fire_zones']:
                fire_level = data['fire_zones'][node_id]
                row.append(f'{fire_level:>3} ')
            else:
                row.append('  0 ')
        grid_risks.append(row)

    for row in grid_risks:
        print(''.join(row))
    print()

    # 图2: 显示路径和节点成本
    print("🛣️ GRID 2: Optimal Path with Node Costs")
    print("Legend: S=Start, G=Goal, X=Obstacle, [N]=PathCost")
    print()

    grid_path = []
    path_costs = {}

    if path:
        for i, node in enumerate(path):
            path_costs[node] = data['node_data'][node]['cost']

    for y in range(height):
        row = []
        for x in range(width):
            node_id = y * width + x
            if node_id == data['start']:
                row.append('  S ')
            elif node_id == data['goal']:
                row.append('  G ')
            elif node_id in data['obstacles']:
                row.append('  X ')
            elif node_id in path_costs:
                cost = path_costs[node_id]
                row.append(f'[{cost:>2}]')
            else:
                if node_id in data['fire_zones']:
                    fire_level = data['fire_zones'][node_id]
                    row.append(f'{fire_level:>3} ')
                else:
                    row.append('  . ')
        grid_path.append(row)

    for row in grid_path:
        print(''.join(row))
    print()

    # 图3: 简洁路径显示
    print("📍 GRID 3: Clean Path Visualization")
    print("Legend: S=Start, G=Goal, X=Obstacle, *=Path, Number=FireLevel")
    print()

    grid_clean = []
    for y in range(height):
        row = []
        for x in range(width):
            node_id = y * width + x
            if node_id == data['start']:
                row.append(' S ')
            elif node_id == data['goal']:
                row.append(' G ')
            elif node_id in data['obstacles']:
                row.append(' X ')
            elif node_id in path:
                row.append(' * ')
            elif node_id in data['fire_zones']:
                fire_level = data['fire_zones'][node_id]
                row.append(f'{fire_level:>2} ')
            else:
                row.append(' . ')
        grid_clean.append(row)

    for row in grid_clean:
        print(''.join(row))
    print()

    # 显示详细的路径分析
    if path:
        print(f"Optimal Path Found! Length: {len(path)} steps")
        print(f"Path: {' -> '.join(map(str, path))}")

        print("\nPath Risk Analysis:")
        total_risk = 0
        for i, node in enumerate(path):
            y = node // width
            x = node % width
            risk_level = data['fire_zones'].get(node, 0)
            total_risk += risk_level

            risk_desc = "🔥High" if risk_level >= 8 else "⚠️Medium" if risk_level >= 4 else "✅Low" if risk_level >= 1 else "🟢Safe"
            node_cost = data['node_data'][node]['cost']
            print(
                f"  Step {i:2d}: Node {node:2d} ({x},{y}) | Cost: {node_cost:2d} | Fire Risk: {risk_level} {risk_desc}")

        if data['goal'] in data['node_data']:
            total_cost = data['node_data'][data['goal']]['cost']
            print(f"\n📈 Summary:")
            print(f"  Total Path Cost: {total_cost}")
            print(f"  Total Fire Risk Exposure: {total_risk}")
            if total_risk > 0:
                risk_per_step = total_risk / len(path)
                print(f"  Average Risk per Step: {risk_per_step:.2f}")
            else:
                print(f"  🎉 Perfect Safety: Path avoids all fire risks!")

    # 势场分析
    print("\nPotential Field Analysis:")
    high_risk_nodes = [node for node, level in data['fire_zones'].items() if level >= 8]
    medium_risk_nodes = [node for node, level in data['fire_zones'].items() if 4 <= level < 8]
    low_risk_nodes = [node for node, level in data['fire_zones'].items() if 1 <= level < 4]
    safe_nodes = [node for node, level in data['fire_zones'].items() if level == 0]

    print(f"  🔥 High Risk Zones (8-10): {len(high_risk_nodes)} nodes")
    print(f"  ⚠️ Medium Risk Zones (4-7): {len(medium_risk_nodes)} nodes")
    print(f"  ✅ Low Risk Zones (1-3): {len(low_risk_nodes)} nodes")
    print(f"  🟢 Safe Zones (0): {len(safe_nodes)} nodes")

    if path:
        path_high_risk = len([node for node in path if data['fire_zones'].get(node, 0) >= 8])
        path_medium_risk = len([node for node in path if 4 <= data['fire_zones'].get(node, 0) < 8])
        path_low_risk = len([node for node in path if 1 <= data['fire_zones'].get(node, 0) < 4])
        path_safe = len([node for node in path if data['fire_zones'].get(node, 0) == 0])

        print(
            f"  🛣️ Path Risk Exposure: {path_high_risk} high, {path_medium_risk} medium, {path_low_risk} low, {path_safe} safe")


def main():
    if len(sys.argv) > 1:
        with open(sys.argv[1], 'r') as f:
            output_text = f.read()
    else:
        print("Paste the Potential Field A* output:")
        output_text = sys.stdin.read()

    data = parse_output(output_text)
    visualize_potential_field(data)


if __name__ == "__main__":
    main()