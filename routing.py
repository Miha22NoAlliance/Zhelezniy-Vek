from __future__ import annotations

import heapq
import math
from collections import defaultdict, deque

MAX_LABELS_PER_NODE = 64
MAX_TOTAL_LABELS = 500_000
_COMPONENT_CACHE = {}


def haversine(a, b):
    lat1, lon1 = a
    lat2, lon2 = b
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    h = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 6371000.0 * 2 * math.asin(math.sqrt(h))


def _components(graph):
    key = id(graph)
    cached = _COMPONENT_CACHE.get(key)
    if cached is not None:
        return cached

    adj = graph["adj"]
    comp_of = {}
    components = []
    for start in adj:
        if start in comp_of:
            continue
        cid = len(components)
        q = deque([start])
        comp_of[start] = cid
        nodes = []
        while q:
            u = q.popleft()
            nodes.append(u)
            for edge in adj.get(u, ()):
                v = edge["to"]
                if v not in comp_of:
                    comp_of[v] = cid
                    q.append(v)
        components.append(nodes)
    _COMPONENT_CACHE[key] = (comp_of, components)
    return comp_of, components


def nearest_candidates(graph, point, limit=16):
    values = []
    for node_id, node in graph["nodes"].items():
        values.append((haversine(point, (node[0], node[1])), node_id))
    values.sort(key=lambda x: x[0])
    return values[:limit]


def choose_endpoints(graph, start_point, goal_point):
    comp_of, _ = _components(graph)
    starts = nearest_candidates(graph, start_point)
    goals = nearest_candidates(graph, goal_point)

    best = None
    for sd, sn in starts:
        sc = comp_of.get(sn)
        if sc is None:
            continue
        for gd, gn in goals:
            if comp_of.get(gn) != sc:
                continue
            score = sd + gd
            if best is None or score < best[0]:
                best = (score, sn, sd, gn, gd)

    if best is None:
        raise ValueError("Старт и финиш находятся в разных изолированных участках карты")

    return best[1], best[3], best[2], best[4]


def shortest_path(graph, start, goal):
    q = [(0.0, start)]
    best = {start: 0.0}
    parent = {start: None}
    while q:
        d, u = heapq.heappop(q)
        if d != best.get(u):
            continue
        if u == goal:
            break
        for edge in graph["adj"].get(u, ()):
            v = edge["to"]
            nd = d + edge["dist"]
            if nd < best.get(v, float("inf")):
                best[v] = nd
                parent[v] = u
                heapq.heappush(q, (nd, v))
    if goal not in best:
        return None
    ids = []
    u = goal
    while u is not None:
        ids.append(u)
        u = parent[u]
    return list(reversed(ids)), best[goal]


def path_score(graph, node_ids):
    total = 0.0
    criteria = defaultdict(float)
    for a, b in zip(node_ids, node_ids[1:]):
        edge = next((e for e in graph["adj"].get(a, ()) if e["to"] == b), None)
        if edge is None:
            continue
        total += edge["score"]
        for k, v in edge["criteria"].items():
            criteria[k] += v
    return total, criteria


def _contains_ancestor(labels, label_id, node_id):
    cur = label_id
    while cur is not None:
        if labels[cur][0] == node_id:
            return True
        cur = labels[cur][4]
    return False


def _insert_label(labels, active, by_node, node, dist, score, lid):
    candidates = by_node[node]
    for old_id in candidates:
        if old_id not in active:
            continue
        old = labels[old_id]
        if old[2] <= dist and old[1] >= score and (old[2] < dist or old[1] > score):
            return False

    doomed = []
    for old_id in candidates:
        if old_id not in active:
            continue
        old = labels[old_id]
        if dist <= old[2] and score >= old[1] and (dist < old[2] or score > old[1]):
            doomed.append(old_id)

    for old_id in doomed:
        active.discard(old_id)
    by_node[node] = [x for x in candidates if x in active]
    by_node[node].append(lid)
    active.add(lid)

    if len(by_node[node]) > MAX_LABELS_PER_NODE:
        ranked = sorted(by_node[node], key=lambda x: (-labels[x][1], labels[x][2]))
        keep = set(ranked[:MAX_LABELS_PER_NODE])
        for old_id in by_node[node]:
            if old_id not in keep:
                active.discard(old_id)
        by_node[node] = ranked[:MAX_LABELS_PER_NODE]
    return True


def _weighted_path(graph, start, goal, quality_weight):
    q = [(0.0, 0.0, start)]
    best = {start: 0.0}
    distance = {start: 0.0}
    parent = {start: None}
    expanded = 0

    while q:
        cost, dist_so_far, u = heapq.heappop(q)
        if cost != best.get(u):
            continue

        expanded += 1
        if u == goal:
            break

        for edge in graph["adj"].get(u, ()):
            edge_dist = edge["dist"]
            edge_score = edge["score"]

            # Positive edge cost keeps Dijkstra valid and prevents cycles.
            # Higher quality_weight increasingly prefers edges with better
            # score per metre, while negative scores are naturally penalized.
            ratio = edge_score / max(edge_dist, 1.0)
            exponent = max(-20.0, min(20.0, -quality_weight * ratio))
            edge_cost = edge_dist * math.exp(exponent)

            new_dist = dist_so_far + edge_dist
            new_cost = cost + edge_cost
            v = edge["to"]

            if new_cost < best.get(v, float("inf")) - 1e-9:
                best[v] = new_cost
                distance[v] = new_dist
                parent[v] = u
                heapq.heappush(q, (new_cost, new_dist, v))

    if goal not in distance:
        return None

    node_ids = []
    u = goal
    while u is not None:
        node_ids.append(u)
        u = parent[u]
    node_ids.reverse()

    return node_ids, distance[goal], expanded


def _route_candidate(graph, start, goal, max_distance, quality_weight):
    result = _weighted_path(graph, start, goal, quality_weight)
    if result is None:
        return None

    node_ids, distance_m, expanded = result
    if distance_m > max_distance + 1e-6:
        return None

    score, criteria = path_score(graph, node_ids)
    return {
        "node_ids": node_ids,
        "distance_m": distance_m,
        "score": score,
        "criteria": criteria,
        "expanded": expanded,
        "quality_weight": quality_weight,
    }


def find_route(graph, start_point, goal_point, detour_factor=1.35):
    start, goal, snap_s, snap_g = choose_endpoints(graph, start_point, goal_point)
    shortest = shortest_path(graph, start, goal)
    if shortest is None:
        raise ValueError("Между выбранными точками нет пешеходного пути")

    shortest_ids, shortest_m = shortest
    detour_factor = max(1.0, min(float(detour_factor), 2.5))
    max_distance = shortest_m * detour_factor

    if detour_factor <= 1.00001:
        total_score, criteria = path_score(graph, shortest_ids)
        coordinates = [[graph["nodes"][nid][0], graph["nodes"][nid][1]] for nid in shortest_ids]
        return _result(
            start_point, goal_point, snap_s, snap_g,
            shortest_m, shortest_m, total_score, criteria,
            coordinates, len(shortest_ids), False, max_distance
        )

    # Search a small, logarithmically spaced family of quality-biased paths.
    # Every path is produced by Dijkstra with positive edge costs, so there
    # are no repeated points and runtime is far smaller than the old
    # multi-label search.
    candidates = []

    weights = [0.0]
    w = 0.25
    while w <= 64.0:
        weights.append(w)
        w *= 1.65

    detour_bias = max(0.0, detour_factor - 1.0)
    weights += [
        4.0 * detour_bias,
        8.0 * detour_bias,
        16.0 * detour_bias,
        32.0 * detour_bias,
    ]

    seen_weights = set()
    for quality_weight in sorted(weights):
        key = round(quality_weight, 6)
        if key in seen_weights:
            continue
        seen_weights.add(key)

        candidate = _route_candidate(
            graph, start, goal, max_distance, quality_weight
        )
        if candidate is not None:
            candidates.append(candidate)

    if not candidates:
        total_score, criteria = path_score(graph, shortest_ids)
        coordinates = [[graph["nodes"][nid][0], graph["nodes"][nid][1]] for nid in shortest_ids]
        return _result(
            start_point, goal_point, snap_s, snap_g,
            shortest_m, shortest_m, total_score, criteria,
            coordinates, 0, True, max_distance
        )

    # Pick the highest-scoring valid path. Prefer the shorter one only when
    # scores are effectively identical.
    best = max(
        candidates,
        key=lambda c: (c["score"], -c["distance_m"])
    )

    node_ids = best["node_ids"]
    coordinates = [[graph["nodes"][nid][0], graph["nodes"][nid][1]] for nid in node_ids]

    return _result(
        start_point, goal_point, snap_s, snap_g,
        best["distance_m"], shortest_m,
        best["score"], best["criteria"], coordinates,
        best["expanded"], False, max_distance
    )

def _result(start_point, goal_point, snap_s, snap_g, distance_m, shortest_m,
            score, criteria, coordinates, expanded, fallback, max_distance=None):
    if max_distance is None:
        max_distance = distance_m
    return {
        "start": {"lat": start_point[0], "lon": start_point[1], "snap_m": round(snap_s, 1)},
        "goal": {"lat": goal_point[0], "lon": goal_point[1], "snap_m": round(snap_g, 1)},
        "distance_m": round(distance_m, 1),
        "shortest_m": round(shortest_m, 1),
        "max_distance_m": round(max_distance, 1),
        "score": round(score, 2),
        "score_per_km": round(score / max(distance_m / 1000.0, 0.001), 2),
        "expanded_labels": expanded,
        "coordinates": coordinates,
        "criteria": {k: round(v, 2) for k, v in sorted(criteria.items(), key=lambda kv: -abs(kv[1])) if abs(v) > 0.001},
        "fallback": fallback,
        "repeated_points": len(coordinates) - len({(round(p[0], 7), round(p[1], 7)) for p in coordinates}),
    }
