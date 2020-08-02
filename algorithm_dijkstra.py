class algorithm_dijkstra:
    def __init__(self, grahp, start):
        self.start = start
        self.grahp = grahp
        self.len_grahp = len(self.grahp)
        self.is_visited = [False] * self.len_grahp
        self.cost = [float('inf')] * self.len_grahp
        self.parent = [-1] * self.len_grahp
        self.path_start = start
        self.cost[start] = 0
        self.min_cost = 0

    def dijkstra(self):
        while self.min_cost < float('inf'):
            self.is_visited[self.start] = True
            for i, vertex in enumerate(self.grahp[self.start]):
                if vertex != 0 and not self.is_visited[i]:
                    if self.cost[i] > vertex + self.cost[self.start]:
                        self.cost[i] = vertex + self.cost[self.start]
                        self.parent[i] = self.start

            self.min_cost = float('inf')
            for i in range(self.len_grahp):
                if self.min_cost > self.cost[i] and not self.is_visited[i]:
                    self.min_cost = self.cost[i]
                    self.start = i

        for end_vertex in range(self.len_grahp):
            path = []
            if end_vertex != self.path_start:
                vertex = end_vertex
                if self.parent[vertex] != -1:
                    path.append(vertex)
                    while True:
                        path.append(self.parent[vertex])
                        if self.parent[vertex] == self.path_start:
                            break
                        vertex = self.parent[vertex]
                else:
                    path.append(f'Путь от вершины {_start} до вершины {vertex} не существует')
                print(f'Путь от вершины {_start} до вершины {end_vertex}: {path[::-1]}, вес = {self.cost[end_vertex]}')


_grahp = [
    [0, 0, 1, 1, 9, 0, 0, 0],
    [0, 0, 9, 4, 0, 0, 5, 0],
    [0, 9, 1, 1, 3, 0, 6, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 5, 0],
    [0, 0, 7, 0, 8, 1, 0, 0],
    [0, 0, 0, 0, 0, 1, 2, 0],
]
_start = int(input("От какой вершины идти: "))
#_start = 0
work_algorithm = algorithm_dijkstra(_grahp, _start)
work_algorithm.dijkstra()
