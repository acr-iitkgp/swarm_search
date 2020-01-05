from heapq import heappush, heappop, heapify
import math


def findSet(parent, u):
    if parent[u] == u:
        return u
    parent[u] = findSet(parent, parent[u])
    return parent[u]


def unionSet(parent, rnk, u, v):
    u = findSet(parent, u)
    v = findSet(parent, v)
    if u != v:
        if rnk[u] < rnk[v]:
            temp = u
            u = v
            v = temp
        parent[v] = u
        if rnk[u] == rnk[v]:
            rnk[u] += 1


class Edge:

    def __init__(self, _u, _v, _w):
        self.u = _u
        self.v = _v
        self.w = _w

    def __lt__(self, E):
        return self.w < E.w


def AHP(edge, n):

    heapify(edge)

    parent = [i for i in range(n)]
    rnk = [0 for i in range(n)]
    path = []
    restore = []
    vis = [False] * n
    w = 0
    print(str(n) + "n ki value")
    i = 0
    while i < math.floor(n / 2):
        print(str(i) + "first")
        e = heappop(edge)
        if findSet(parent, e.u) != findSet(parent, e.v) and not vis[e.u] and not vis[e.v]:
            vis[e.u] = vis[e.v] = True
            unionSet(parent, rnk, e.u, e.v)
            path.append((e.u, e.v))
            # print(path)
            w += e.w
            i += 1
        else:
            restore.append(Edge(e.u, e.v, e.w))

    for e in restore:
        heappush(edge, Edge(e.u, e.v, e.w))

    vis = [False] * n

    i = 0
    while i < math.floor((n - 1) / 2):
        print(str(i) + "sec")
        e = heappop(edge)
        if findSet(parent, e.u) != findSet(parent, e.v) and not vis[e.u] and not vis[e.v]:
            vis[e.u] = vis[e.v] = True
            unionSet(parent, rnk, e.u, e.v)
            path.append((e.u, e.v))
            # print(path)
            w += e.w
            i += 1

    return path, w
