from __future__ import annotations

import heapq
import math
from collections import defaultdict, deque

_COMPONENT_CACHE = {}

ROAD_FACTORS_SIMPLE = {
    "trunk": 0.90,
    "trunk_link": 0.94,
    "primary": 0.92,
    "primary_link": 0.95,
    "secondary": 0.95,
    "secondary_link": 0.98,
    "tertiary": 0.98,
    "tertiary_link": 1.00,
    "residential": 1.00,
    "living_street": 1.01,
    "pedestrian": 1.00,
    "cycleway": 1.08,
    "service": 1.18,
    "footway": 1.24,
    "path": 1.30,
    "track": 1.38,
    "steps": 1.62,
}

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


def _find_edge(graph, a, b):
    for edge in graph["adj"].get(a, ()):
        if edge["to"] == b:
            return edge
    return None


def path_score(graph, node_ids):
    total = 0.0
    criteria = defaultdict(float)
    ascent = 0.0
    descent = 0.0
    stairs = 0

    for a, b in zip(node_ids, node_ids[1:]):
        edge = _find_edge(graph, a, b)
        if edge is None:
            continue
        total += edge["score"]
        for k, v in edge["criteria"].items():
            criteria[k] += v

        delta = edge.get("elevation_delta_m")
        if delta is not None:
            if delta > 0:
                ascent += delta
            elif delta < 0:
                descent += -delta
        if edge.get("stairs"):
            stairs += 1

    return total, criteria, ascent, descent, stairs


def _simple_edge_cost(graph, u, edge, goal, quality_weight):
    dist = max(edge["dist"], 1.0)
    factor = ROAD_FACTORS_SIMPLE.get(edge.get("highway", ""), 1.06)

    # Упрощённый режим сильнее ценит непрерывные главные улицы и слабее
    # поощряет узкие местные проходы/дворы.
    before = haversine((graph["nodes"][u][0], graph["nodes"][u][1]), goal)
    v = edge["to"]
    after = haversine((graph["nodes"][v][0], graph["nodes"][v][1]), goal)
    progress = before - after
    progress_ratio = progress / dist

    if progress_ratio < 0:
        factor *= 1.0 + min(1.8, -progress_ratio * 1.35)
    elif progress_ratio < 0.35:
        factor *= 1.0 + (0.35 - progress_ratio) * 0.32

    delta = edge.get("elevation_delta_m")
    if delta is not None:
        uphill = max(0.0, delta)
        downhill = max(0.0, -delta)
        factor *= 1.0 + min(1.2, uphill / 28.0)
        factor *= 1.0 + min(0.35, downhill / 90.0)

    if edge.get("stairs"):
        factor *= 1.35

    ratio = edge.get("score", 0.0) / dist
    exponent = max(-4.0, min(4.0, -quality_weight * ratio))
    factor *= math.exp(exponent)
    return dist * factor


def _quality_edge_cost(edge, quality_weight):
    dist = max(edge["dist"], 1.0)
    factor = 1.0

    # В обычном режиме высота и лестницы влияют мягче, чем в упрощённом.
    delta = edge.get("elevation_delta_m")
    if delta is not None:
        uphill = max(0.0, delta)
        downhill = max(0.0, -delta)
        factor *= 1.0 + min(0.55, uphill / 45.0)
        factor *= 1.0 + min(0.12, downhill / 120.0)

    if edge.get("stairs"):
        factor *= 1.18

    ratio = edge.get("score", 0.0) / dist
    exponent = max(-5.0, min(5.0, -quality_weight * ratio))
    return dist * factor * math.exp(exponent)


def _weighted_path(graph, start, goal, quality_weight, mode):
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
            if mode == "simple":
                edge_cost = _simple_edge_cost(
                    graph, u, edge,
                    (graph["nodes"][goal][0], graph["nodes"][goal][1]),
                    quality_weight,
                )
            else:
                edge_cost = _quality_edge_cost(edge, quality_weight)

            v = edge["to"]
            nd = dist_so_far + edge["dist"]
            nc = cost + edge_cost
            if nc < best.get(v, float("inf")) - 1e-9:
                best[v] = nc
                distance[v] = nd
                parent[v] = u
                heapq.heappush(q, (nc, nd, v))

    if goal not in distance:
        return None

    node_ids = []
    u = goal
    while u is not None:
        node_ids.append(u)
        u = parent[u]
    node_ids.reverse()

    return node_ids, distance[goal], expanded


def _route_candidate(graph, start, goal, max_distance, quality_weight, mode):
    result = _weighted_path(graph, start, goal, quality_weight, mode)
    if result is None:
        return None

    node_ids, distance_m, expanded = result
    if distance_m > max_distance + 1e-6:
        return None

    score, criteria, ascent, descent, stairs = path_score(graph, node_ids)
    simple_cost = 0.0
    goal_point = (graph["nodes"][goal][0], graph["nodes"][goal][1])
    for a, b in zip(node_ids, node_ids[1:]):
        edge = _find_edge(graph, a, b)
        if edge is not None:
            simple_cost += _simple_edge_cost(graph, a, edge, goal_point, 0.0)

    return {
        "node_ids": node_ids,
        "distance_m": distance_m,
        "score": score,
        "criteria": criteria,
        "expanded": expanded,
        "ascent": ascent,
        "descent": descent,
        "stairs": stairs,
        "simple_cost": simple_cost,
    }


def find_route(graph, start_point, goal_point, detour_factor=1.35, mode="quality"):
    mode = "simple" if str(mode).lower() in {"simple", "simplified"} else "quality"

    start, goal, snap_s, snap_g = choose_endpoints(graph, start_point, goal_point)
    shortest = shortest_path(graph, start, goal)
    if shortest is None:
        raise ValueError("Между выбранными точками нет пешеходного пути")

    shortest_ids, shortest_m = shortest
    detour_factor = max(1.0, min(float(detour_factor), 1.8))
    max_distance = shortest_m * detour_factor

    if detour_factor <= 1.00001:
        total_score, criteria, ascent, descent, stairs = path_score(graph, shortest_ids)
        coordinates = [[graph["nodes"][nid][0], graph["nodes"][nid][1]] for nid in shortest_ids]
        return _result(
            start_point, goal_point, snap_s, snap_g,
            shortest_m, shortest_m, total_score, criteria,
            coordinates, len(shortest_ids), False, max_distance,
            mode, ascent, descent, stairs
        )

    if mode == "simple":
        search_weights = [0.0, 1.0, 2.5, 5.0, 9.0, 15.0, 24.0]
    else:
        # Более агрессивно исследуем хорошие альтернативы, но каждая серия
        # остаётся обычным Dijkstra и не взрывается сотнями тысяч меток.
        search_weights = [0.0, 4.0, 10.0, 20.0, 40.0, 80.0, 140.0, 220.0, 340.0]

    candidates = []
    seen = set()
    for quality_weight in search_weights:
        if quality_weight in seen:
            continue
        seen.add(quality_weight)
        candidate = _route_candidate(
            graph, start, goal, max_distance,
            quality_weight, mode
        )
        if candidate is not None:
            candidates.append(candidate)

    if not candidates:
        total_score, criteria, ascent, descent, stairs = path_score(graph, shortest_ids)
        coordinates = [[graph["nodes"][nid][0], graph["nodes"][nid][1]] for nid in shortest_ids]
        return _result(
            start_point, goal_point, snap_s, snap_g,
            shortest_m, shortest_m, total_score, criteria,
            coordinates, 0, True, max_distance,
            mode, ascent, descent, stairs
        )

    if mode == "simple":
        best = min(
            candidates,
            key=lambda c: (c["simple_cost"], -c["score"], c["distance_m"])
        )
    else:
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
        best["expanded"], False, max_distance,
        mode, best["ascent"], best["descent"], best["stairs"]
    )


def _result(start_point, goal_point, snap_s, snap_g, distance_m, shortest_m,
            score, criteria, coordinates, expanded, fallback,
            max_distance=None, mode="quality", ascent=0.0, descent=0.0,
            stairs=0):
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
        "criteria": {
            k: round(v, 2)
            for k, v in sorted(criteria.items(), key=lambda kv: -abs(kv[1]))
            if abs(v) > 0.001
        },
        "mode": mode,
        "ascent_m": round(ascent, 1),
        "descent_m": round(descent, 1),
        "stairs_count": stairs,
        "fallback": fallback,
        "repeated_points": len(coordinates) - len({
            (round(p[0], 7), round(p[1], 7)) for p in coordinates
        }),
    }
