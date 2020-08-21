import random
import sys
import heapq
import math


class world:
    # initialise the world
    # L is the number of columns
    # H is the number of lines
    # P is the probability of having a wall in a given tile
    def __init__(self, L, H, P, C):
        self.L = L
        self.H = H

        # the world is represented by an array with one dimension
        self.w = [1 for i in range(L * H)]  # initialise every tile to empty (1)

        # add walls in the first and last columns
        for i in range(H):
            self.w[i * L] = 0
            self.w[i * L + L - 1] = 0

        # add walls in the first and last lines
        for j in range(L):
            self.w[j] = 0
            self.w[(H - 1) * L + j] = 0

        for i in range(H):
            for j in range(L):
                # add a wall in this tile with probability P and provided that it is neither
                # the starting tile nor the goal tile
                if random.random() < P and not (i == 1 and j == 1) and not (i == H - 2 and j == L - 2):
                    self.w[i * L + j] = 0
                elif i > 0 and i < H - 1 and j > self.L / 3 and j < 2 * L / 3:
                    self.w[i * L + j] = C

    # display the world
    def display(self):
        for i in range(self.H):
            for j in range(self.L):
                if self.w[i * self.L + j] == 1:
                    sys.stdout.write('.')
                elif self.w[i * self.L + j] == 0:
                    sys.stdout.write('W')
                elif self.w[i * self.L + j] == C:
                    sys.stdout.write(':')
            print('')

    # compute the successors of tile number i in world w
    def successors(self, i):
        if i < 0 or i >= self.L * self.H or self.w[i] == 0:
            # i is an incorrect tile number (outside the array or on a wall)
            return []
        else:
            # look in the four adjacent tiles and keep only those with no wall
            return list(filter(lambda x: self.w[x] > 0, [i - 1, i + 1, i - self.L, i + self.L]))

    # Depth-first search
    # starting from tile number s0, find a path to tile number t
    # return (r, path) where r is true if such a path exists, false otherwise
    # and path contains the path if it exists
    def dfs(self, s0, t):
        r = False
        path = []
        stack = []
        stack.append(s0)
        passed_list = set()
        passed_list.add(s0)
        parent = {s0: None}
        while len(stack) > 0:
            vertex = stack.pop()
            if vertex == t:
                r = True
                v = t
                path.append(t)
                while v != None:
                    v = parent[v]
                    path.append(v)
                path.reverse()
                path.pop(0)
            nodes = w.successors(vertex)
            for i in nodes:
                if i not in passed_list:
                    stack.append(i)
                    passed_list.add(i)
                    parent[i] = vertex
        return r, path

    def bfs(self, s0, t):
        r = False
        path = []
        queue = [s0]
        passed_list = set()
        passed_list.add(s0)
        parent = {s0: None}
        while len(queue) > 0:
            vertex = queue.pop(0)
            if (vertex) == t:
                r = True
                v = t
                path.append(t)
                while v != None:
                    v = parent[v]
                    path.append(v)
                path.reverse()
                path.pop(0)
            nodes = w.successors(vertex)
            for i in nodes:
                if i not in passed_list:
                    queue.append(i)
                    passed_list.add(i)
                    parent[i] = vertex
        return r, path

    def display_dfs(self):
        a, b = w.dfs(self.L + 1, self.L * (self.H - 1) - 2)
        for i in range(self.H):
            for j in range(self.L):
                if self.w[i * self.L + j] > 0:
                    if i * self.L + j in b:
                        sys.stdout.write('*')
                    elif self.w[i * self.L + j] == C:
                        sys.stdout.write(':')
                    else:
                        sys.stdout.write('.')
                elif self.w[i * self.L + j] == 0:
                    sys.stdout.write('W')
            print('')

    def display_bfs(self):
        a, b = w.bfs(self.L + 1, self.L * (self.H - 1) - 2)
        for i in range(self.H):
            for j in range(self.L):
                if self.w[i * self.L + j] > 0:
                    if i * self.L + j in b:
                        sys.stdout.write('*')
                    elif self.w[i * self.L + j] == C:
                        sys.stdout.write(':')
                    else:
                        sys.stdout.write('.')
                elif self.w[i * self.L + j] == 0:
                    sys.stdout.write('W')
            print('')

    def init_distance(self, s0):
        distance = {s0: 0}
        for i in range(self.H):
            for j in range(self.L):
                if self.w[i * self.L + j] > 0:
                    distance[i * self.L + j] = math.inf
        return distance

    def init_f(self, s0):
        f = {s0: self.L + self.H - 4}
        for i in range(self.H):
            for j in range(self.L):
                if self.w[i * self.L + j] > 0:
                    f[i * self.L + j] = math.inf
        return f

    def dijkstra(self, s0, t):
        r = False
        path = []
        pqueue = []
        heapq.heappush(pqueue, (0, s0))
        passed_list = set()
        passed_list.add(s0)
        parent = {s0: None}
        distance = w.init_distance(s0)
        final_dist = math.inf
        count_dijk = 0
        while len(pqueue) > 0:
            count_dijk = count_dijk + 1
            dist, vertex = heapq.heappop(pqueue)
            passed_list.add(vertex)
            if vertex == t:
                r = True
                v = t
                path.append(t)
                while v != None:
                    v = parent[v]
                    path.append(v)
                path.reverse()
                path.pop(0)
                final_dist = distance[t]
                break
            nodes = w.successors(vertex)
            for i in nodes:
                if i not in passed_list:
                    if dist + self.w[i] < distance[i]:
                        parent[i] = vertex
                        distance[i] = dist + self.w[i]
                        heapq.heappush(pqueue, (dist + self.w[i], i))
        print(count_dijk)
        return r, path, final_dist, passed_list

    def display_djik(self):
        a, b, c, d = w.dijkstra(self.L + 1, self.L * (self.H - 1) - 2)
        for i in range(self.H):
            for j in range(self.L):
                if self.w[i * self.L + j] > 0:
                    if i * self.L + j in b:
                        sys.stdout.write('*')
                    elif self.w[i * self.L + j] == C:
                        sys.stdout.write(':')
                    else:
                        sys.stdout.write('.')
                elif self.w[i * self.L + j] == 0:
                    sys.stdout.write('W')
            print('')

    def hn(self):
        h = [None for i in range(self.L * self.H)]
        for i in range(self.H):
            for j in range(self.L):
                h[i * self.L + j] = self.H - 1 - i + self.L - 1 - j
        return h

    def h_dijkstra(self, s0, t):
        r = False
        path = []
        pqueue = []
        heapq.heappush(pqueue, (L + H - 4, s0, 0))
        passed_list = set()
        passed_list.add(s0)
        parent = {s0: None}
        fn = w.init_f(s0)
        final_dist = math.inf
        h = w.hn()
        count_astar = 0
        while len(pqueue) > 0:
            count_astar = count_astar + 1
            f, vertex, dist = heapq.heappop(pqueue)
            passed_list.add(vertex)
            if vertex == t:
                r = True
                v = t
                path.append(t)
                while v != None:
                    v = parent[v]
                    path.append(v)
                path.reverse()
                path.pop(0)
                final_dist = fn[t]
                break
            nodes = w.successors(vertex)
            for i in nodes:
                if i not in passed_list:
                    if dist + self.w[i] + h[i] < fn[i]:
                        parent[i] = vertex
                        fn[i] = dist + self.w[i] + h[i]
                        heapq.heappush(pqueue, (dist + self.w[i] + h[i], i, dist + self.w[i]))
        print(count_astar)
        return r, path, final_dist, passed_list

    def display_hpassed(self):
        a, b, c, d = w.h_dijkstra(self.L + 1, self.L * (self.H - 1) - 2)
        for i in range(self.H):
            for j in range(self.L):
                if self.w[i * self.L + j] > 0:
                    if i * self.L + j in d:
                        sys.stdout.write('*')
                    elif self.w[i * self.L + j] == C:
                        sys.stdout.write(':')
                    else:
                        sys.stdout.write('.')
                elif self.w[i * self.L + j] == 0:
                    sys.stdout.write('W')
            print('')

    def display_passed(self):
        a, b, c, d = w.dijkstra(self.L + 1, self.L * (self.H - 1) - 2)
        for i in range(self.H):
            for j in range(self.L):
                if self.w[i * self.L + j] > 0:
                    if i * self.L + j in d:
                        sys.stdout.write('*')
                    elif self.w[i * self.L + j] == C:
                        sys.stdout.write(':')
                    else:
                        sys.stdout.write('.')
                elif self.w[i * self.L + j] == 0:
                    sys.stdout.write('W')
            print('')

    def display_hdjik(self):
        a, b, c, d = w.h_dijkstra(self.L + 1, self.L * (self.H - 1) - 2)
        for i in range(self.H):
            for j in range(self.L):
                if self.w[i * self.L + j] > 0:
                    if i * self.L + j in b:
                        sys.stdout.write('*')
                    elif self.w[i * self.L + j] == C:
                        sys.stdout.write(':')
                    else:
                        sys.stdout.write('.')
                elif self.w[i * self.L + j] == 0:
                    sys.stdout.write('W')
            print('')


# create a world

L = 20
H = 10
P = 0.2
C = 2
w = world(L, H, P, C)

# display it
w.display()

# print the tile numbers of the successors of the starting tile (1, 1)
# print(w.successors(w.L + 1))
# result_dfs = w.dfs(L + 1, L * (H - 1) - 2)
# result_bfs = w.bfs(L + 1, L * (H - 1) - 2)
# result_dijk = w.dijkstra(L + 1, L * (H - 1) - 2)

# print(result_dijk)
# w.display_djik()
# w.display_bfs()
# result_hdijk = w.h_dijkstra(L + 1, L * (H - 1) - 2)
# print(result_hdijk)
print('')
print('dijkstra path')
w.display_djik()
print('')
print('A* path')
w.display_hdjik()
print('')
print('dijkstra visited nodes')
w.display_passed()
print('')
print('A* visited nodes')
w.display_hpassed()
