from __future__ import annotations

import heapq
import math
from collections import defaultdict, deque

MAX_LABELS_PER_NODE = 48
MAX_TOTAL_LABELS = 300_000
_COMPONENT_CACHE = {}


def haversine(a, b):
    lat1, lon1 = a; lat2, lon2 = b
    p1 = math.radians(lat1); p2 = math.radians(lat2)
    dp = math.radians(lat2-lat1); dl = math.radians(lon2-lon1)
    h = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return 6371000.0 * 2 * math.asin(math.sqrt(h))


def nearest_node(graph, point):
    key = id(graph)
    allowed = _COMPONENT_CACHE.get(key)
    if allowed is None:
        start = max(graph["adj"], key=lambda n: len(graph["adj"].get(n, [])), default=None)
        allowed = set()
        if start is not None:
            q = deque([start])
            allowed.add(start)
            while q:
                u = q.popleft()
                for edge in graph["adj"].get(u, []):
                    v = edge["to"]
                    if v not in allowed:
                        allowed.add(v)
                        q.append(v)
        _COMPONENT_CACHE[key] = allowed

    best = None
    best_d = float("inf")
    for node_id in allowed:
        node = graph["nodes"][node_id]
        d = haversine(point, (node[0], node[1]))
        if d < best_d:
            best_d = d
            best = node_id
    return best, best_d


def dominates(a, b):
    return a[0] <= b[0] and a[1] >= b[1] and (a[0] < b[0] or a[1] > b[1])


def insert_label(labels_by_node, node, dist, score, label_id):
    candidates = labels_by_node[node]

    for old in candidates:
        if dominates((old[0], old[1]), (dist, score)):
            return False

    candidates[:] = [old for old in candidates if not dominates((dist, score), (old[0], old[1]))]
    candidates.append((dist, score, label_id))

    if len(candidates) > MAX_LABELS_PER_NODE:
        candidates.sort(key=lambda x: (-x[1], x[0]))
        candidates[:] = sorted(candidates, key=lambda x: (-x[1], x[0]))[:MAX_LABELS_PER_NODE]
    return True


def shortest_distance(graph, start, goal):
    q = [(0.0, start)]
    best = {start: 0.0}
    while q:
        d, u = heapq.heappop(q)
        if d != best.get(u):
            continue
        if u == goal:
            return d
        for edge in graph["adj"].get(u, []):
            nd = d + edge["dist"]
            if nd < best.get(edge["to"], float("inf")):
                best[edge["to"]] = nd
                heapq.heappush(q, (nd, edge["to"]))
    return None


def reconstruct(labels, label_id):
    ids = []
    while label_id is not None:
        ids.append(labels[label_id][0])
        label_id = labels[label_id][4]
    return list(reversed(ids))


def find_route(graph, start_point, goal_point, detour_factor=1.35):
    start, snap_s = nearest_node(graph, start_point)
    goal, snap_g = nearest_node(graph, goal_point)
    if start is None or goal is None:
        raise ValueError("Граф улиц ещё не загружен")

    shortest = shortest_distance(graph, start, goal)
    if shortest is None:
        raise ValueError("Между выбранными точками нет пешеходного пути")

    max_distance = shortest * max(1.0, min(detour_factor, 2.5))

    # Distance is a constraint only. Objective: maximize accumulated quality score.
    labels = []  # node, score, distance, via_edge, parent_label
    labels_by_node = defaultdict(list)
    queue = []

    labels.append((start, 0.0, 0.0, None, None))
    insert_label(labels_by_node, start, 0.0, 0.0, 0)
    heapq.heappush(queue, (-0.0, 0.0, 0))

    best_goal = None
    expanded = 0

    while queue and len(labels) < MAX_TOTAL_LABELS:
        neg_score, dist, lid = heapq.heappop(queue)
        node, score, ldist, edge_used, parent = labels[lid]

        if abs(ldist - dist) > 1e-6 or abs(-neg_score - score) > 1e-6:
            continue

        expanded += 1
        if node == goal:
            if best_goal is None or score > labels[best_goal][1]:
                best_goal = lid
            continue

        for edge in graph["adj"].get(node, []):
            nd = ldist + edge["dist"]
            if nd > max_distance:
                continue
            ns = score + edge["score"]
            new_id = len(labels)

            if not insert_label(labels_by_node, edge["to"], nd, ns, new_id):
                continue

            labels.append((edge["to"], ns, nd, edge, lid))
            heapq.heappush(queue, (-ns, nd, new_id))

    if best_goal is None:
        raise ValueError("Не удалось найти качественный маршрут в заданном лимите объезда")

    node_ids = reconstruct(labels, best_goal)
    coordinates = [[graph["nodes"][nid][0], graph["nodes"][nid][1]] for nid in node_ids]

    total_score = labels[best_goal][1]
    total_distance = labels[best_goal][2]

    contributions = defaultdict(float)
    cur = best_goal
    while cur is not None:
        edge = labels[cur][3]
        if edge is not None:
            for key, value in edge["criteria"].items():
                contributions[key] += value
        cur = labels[cur][4]

    return {
        "start": {"lat": start_point[0], "lon": start_point[1], "snap_m": round(snap_s, 1)},
        "goal": {"lat": goal_point[0], "lon": goal_point[1], "snap_m": round(snap_g, 1)},
        "distance_m": round(total_distance, 1),
        "shortest_m": round(shortest, 1),
        "max_distance_m": round(max_distance, 1),
        "score": round(total_score, 2),
        "score_per_km": round(total_score / max(total_distance / 1000.0, 0.001), 2),
        "expanded_labels": expanded,
        "coordinates": coordinates,
        "criteria": {k: round(v, 2) for k, v in sorted(contributions.items(), key=lambda kv: -abs(kv[1])) if abs(v) > 0.001},
    }
