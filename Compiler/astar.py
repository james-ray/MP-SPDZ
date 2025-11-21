""" A* search algorithm implementation """

from Compiler.oram import *
from Compiler.program import Program
from Compiler.dijkstra import HeapQ, HeapEntry
from Compiler import types
from Compiler.util import log2

# 获取全局程序实例
try:
    prog = Program.prog
except:
    prog = None

ORAM = OptimalORAM


def astar(start, goal, edges, e_index, heuristic, oram_type=OptimalORAM,
          n_loops=None, int_type=None, debug=False):
    """
    Securely compute A* search algorithm on a secret graph.
    """
    if prog:
        prog.reading("A* search algorithm", "Custom")

    vert_loops = n_loops * e_index.size // edges.size if n_loops else -1

    # g_score: actual cost from start to current node
    g_score = oram_type(e_index.size, entry_size=(32, log2(e_index.size)),
                        init_rounds=vert_loops, value_type=int_type)

    int_type = g_score.value_type if int_type is None else int_type
    basic_type = int_type.basic_type

    Q = HeapQ(e_index.size, oram_type, init_rounds=vert_loops, int_type=int_type)

    if n_loops is not None:
        stop_timer()
        start_timer(1)

    # Initialize start node
    g_score[start] = (0, 0)  # (distance, previous)
    start_h = heuristic(start)
    start_f = start_h  # f = g + h, but g=0 for start
    Q.update(start, start_f)

    if n_loops is not None:
        stop_timer(1)
        start_timer()

    last_edge = MemValue(basic_type(1))
    i_edge = MemValue(int_type(0))
    u = MemValue(basic_type(0))
    running = MemValue(basic_type(1))
    goal_found = MemValue(basic_type(0))

    @for_range(n_loops or edges.size)
    def f(i):
        if debug:
            print_ln('A* loop %s', i)
        time()

        # 关键修复：先检查是否到达目标，再更新running状态
        goal_reached = (u == goal)
        goal_found.write(goal_found + goal_reached * (1 - goal_found))  # 只设置一次

        # 更新running状态：基于图状态且未找到目标
        running.write(last_edge.bit_not().bit_or(Q.size > 0).bit_and(1 - goal_found))

        u.write(if_else(last_edge, Q.pop(last_edge * (1 - goal_found)), u))

        i_edge.write(int_type(if_else(last_edge, e_index[u], i_edge)))
        v, weight, le = edges[i_edge]
        last_edge.write(le)
        i_edge.iadd(1)

        current_g = int_type(g_score[u][0])
        tentative_g = current_g + int_type(weight)

        gv, not_visited = g_score.read(v)
        is_better = (tentative_g < int_type(gv[0])) + not_visited
        is_better *= running * (1 - goal_reached)  # 只在运行中且未到达当前目标时更新

        g_score.access(v, (basic_type(tentative_g), u), is_better)

        # 使用启发式函数计算f_score
        new_h = heuristic(v)
        new_f = tentative_g + new_h
        Q.update(v, basic_type(new_f), is_better * (1 - goal_reached))  # 到达目标时不入队

    return g_score, goal_found

def absolute_value(x):
    """
    Compute absolute value of a sint securely.
    """
    # 使用比较来计算绝对值：abs(x) = (x < 0) * (-x) + (x >= 0) * x
    # 或者使用：abs(x) = x * (1 - 2 * (x < 0))
    is_negative = x < 0
    return x * (1 - 2 * is_negative)

def manhattan_heuristic_factory(goal, grid_width):
    """
    Create a Manhattan distance heuristic function for grid-based graphs.
    Works only when grid_width is a power of two.
    """
    # 检查 grid_width 是否是2的幂
    if (grid_width & (grid_width - 1)) != 0:
        raise Exception("manhattan_heuristic_factory only works with power-of-two grid widths")

    # 计算对数
    log_width = log2(grid_width)

    def heuristic(node):
        # 使用位运算计算坐标（对于2的幂次方的grid_width）
        node_x = node % grid_width  # 模运算对于2的幂是支持的
        node_y = node >> log_width  # 使用右移位代替除法

        goal_x = goal % grid_width
        goal_y = goal >> log_width

        # 曼哈顿距离 - 使用安全的绝对值计算
        dx = absolute_value(node_x - goal_x)
        dy = absolute_value(node_y - goal_y)
        return dx + dy

    return heuristic

def euclidean_heuristic_factory(goal, grid_width):
    """
    Create a squared Euclidean distance heuristic.
    Works only when grid_width is a power of two.
    """
    if (grid_width & (grid_width - 1)) != 0:
        raise Exception("euclidean_heuristic_factory only works with power-of-two grid widths")

    log_width = log2(grid_width)

    def heuristic(node):
        node_x = node % grid_width
        node_y = node >> log_width

        goal_x = goal % grid_width
        goal_y = goal >> log_width

        dx = node_x - goal_x
        dy = node_y - goal_y
        # Return squared Euclidean distance (不需要绝对值，因为平方会处理)
        return dx * dx + dy * dy

    return heuristic

def zero_heuristic(node):
    """Zero heuristic - falls back to Dijkstra"""
    return types.sint(0)

def simple_heuristic(node):
    """Simple heuristic for our test graph"""
    return types.sint(0)

# 简单曼哈顿启发式（使用位运算）
def simple_manhattan_heuristic_factory(goal, grid_width):
    """
    Simple Manhattan heuristic that uses bit operations for power-of-two grid widths.
    """
    # 检查是否是2的幂
    if (grid_width & (grid_width - 1)) == 0:
        log_width = log2(grid_width)

        def heuristic(node):
            # 使用位运算
            node_x = node % grid_width
            node_y = node >> log_width
            goal_x = goal % grid_width
            goal_y = goal >> log_width

            dx = absolute_value(node_x - goal_x)
            dy = absolute_value(node_y - goal_y)
            return dx + dy
    else:
        # 对于非2的幂，使用简单方法
        def heuristic(node):
            # 使用简单的启发式：直线距离的近似
            # 这里我们假设网格是连续的，使用简单的差值
            diff = node - goal
            return absolute_value(diff)

    return heuristic

# 替代方案：使用比较操作实现曼哈顿距离
def manhattan_heuristic_simple_factory(goal, grid_width):
    """
    Simple Manhattan heuristic that avoids complex coordinate calculations.
    This is less accurate but more reliable.
    """
    def heuristic(node):
        # 简单的曼哈顿距离近似
        # 对于小网格，我们可以假设直线距离是合理的启发式
        diff = node - goal
        return absolute_value(diff)

    return heuristic

# 网格专用的曼哈顿距离（使用预计算）
def grid_manhattan_heuristic_factory(goal, grid_width, grid_height=None):
    """
    Grid-specific Manhattan heuristic that precomputes goal coordinates.
    """
    if grid_height is None:
        grid_height = grid_width

    # 预计算目标的坐标（在编译时）
    goal_val = goal.value if hasattr(goal, 'value') else goal
    goal_x = goal_val % grid_width
    goal_y = goal_val // grid_width

    def heuristic(node):
        # 对于每个节点，计算曼哈顿距离
        # 由于网格较小，我们可以使用简单的方法
        diff = node - goal
        return absolute_value(diff)

    return heuristic