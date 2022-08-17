import numpy as np
import heapq as hq

WIDTH = 1024
HEIGHT = 1024


def __get_dist(x_0, y_0, x_1, y_1):
    return ((x_1 - x_0) ** 2 + (y_1 - y_0) ** 2) ** 0.5


def __get_neighbour(map, x, y):
    """
    (x,y) - cost
    """

    for i in range(-1, 2):
        for j in range(-1, 2):
            if i == 0 and j == 0:
                continue

            if i * j != 0:
                continue

            if x + i < 0 or y + j < 0 or x + i > WIDTH - 1 or y + j > HEIGHT - 1:
                continue

            el = map[x + i][y + j]
            cost = el
            yield (x + i, y + j), cost


def dijkstra(map, s_x, s_y, g_x, g_y):
    costs = {(i, j): np.inf for i in range(WIDTH) for j in range(HEIGHT)}
    costs_base = {(i, j): np.inf for i in range(WIDTH) for j in range(HEIGHT)}
    prev = {(i, j): None for i in range(WIDTH) for j in range(HEIGHT)}
    costs[(s_x, s_y)] = 0
    costs_base[(s_x, s_y)] = 0

    visited = set()

    heap = [(0.0, (s_x, s_y))]

    while heap:
        _, (x, y) = hq.heappop(heap)  # type: ignore
        c_base = costs_base[(x, y)]

        if c_base > 100:
            g_x, g_y = x, y
            break

        # print(f"popped with elements => {c,x,y}")

        if x == g_x and y == g_y:
            # print("arrived !!!")
            break

        if (x, y) in visited:
            continue

        visited.add((x, y))

        for (n_x, n_y), n_cost in __get_neighbour(map, x, y):
            # print(f"neighbour {n_x, n_y} cost {n_cost}")
            if (n_x, n_y) in visited:
                continue

            new_cost = c_base + n_cost + __get_dist(n_x, n_y, g_x, g_y)

            base_cost = c_base + n_cost

            if new_cost < costs[(n_x, n_y)]:

                costs[(n_x, n_y)] = new_cost
                costs_base[(n_x, n_y)] = base_cost
                prev[(n_x, n_y)] = (x, y)  # type:ignore

                hq.heappush(heap, (new_cost, (n_x, n_y)))
                # print(f"pushed {(new_cost, (n_x, n_y))}")

    map[x][y] = 500 # type: ignore

    path = []
    current = (g_x, g_y)

    while current != (s_x, s_y):
        path.insert(0, current)
        current = prev[current]

        if current is None:
            # print("not found")
            return None
    else:
        path.insert(0, (s_x, s_y))

    return path
