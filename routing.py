from __future__ import annotations

import heapq
import math
from collections import defaultdict, deque

from simple_mode_v2 import (
    weighted_path as _simple_v2_weighted_path,
    path_cost as _simple_v2_path_cost,
    path_metrics as _simple_v2_path_metrics,
)

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


def shortest_path(graph, start, goal, progress_callback=None):
    """Exact shortest-distance path using bidirectional A*."""
    if start == goal:
        return [start], 0.0

    goal_point = (graph["nodes"][goal][0], graph["nodes"][goal][1])
    start_point = (graph["nodes"][start][0], graph["nodes"][start][1])

    forward_best = {start: 0.0}
    backward_best = {goal: 0.0}
    forward_parent = {start: None}
    backward_parent = {goal: None}

    sequence = 0
    forward_q = [(haversine(start_point, goal_point), 0.0, sequence, start)]
    backward_q = [(haversine(goal_point, start_point), 0.0, sequence, goal)]

    best_total = float("inf")
    meet = None
    expanded = 0
    progress_batch = []

    while forward_q and backward_q:
        while forward_q and forward_q[0][1] > forward_best.get(forward_q[0][3], float("inf")) + 1e-9:
            heapq.heappop(forward_q)
        while backward_q and backward_q[0][1] > backward_best.get(backward_q[0][3], float("inf")) + 1e-9:
            heapq.heappop(backward_q)
        if not forward_q or not backward_q:
            break

        if forward_q[0][0] + backward_q[0][0] >= best_total - 1e-9:
            break

        expand_forward = forward_q[0][0] <= backward_q[0][0]
        if expand_forward:
            _, distance, _, u = heapq.heappop(forward_q)
            if distance > forward_best.get(u, float("inf")) + 1e-9:
                continue

            expanded += 1
            if progress_callback is not None:
                progress_batch.append(u)
                if len(progress_batch) >= 64:
                    progress_callback("shortest", expanded, tuple(progress_batch), u, "forward")
                    progress_batch.clear()

            if u in backward_best:
                total = distance + backward_best[u]
                if total < best_total:
                    best_total, meet = total, u

            for edge in graph["adj"].get(u, ()):
                v = edge["to"]
                nd = distance + float(edge.get("dist", 0.0))
                if nd >= forward_best.get(v, float("inf")) - 1e-9:
                    continue
                forward_best[v] = nd
                forward_parent[v] = u
                sequence += 1
                point = (graph["nodes"][v][0], graph["nodes"][v][1])
                heapq.heappush(
                    forward_q,
                    (nd + haversine(point, goal_point), nd, sequence, v),
                )
        else:
            _, distance, _, u = heapq.heappop(backward_q)
            if distance > backward_best.get(u, float("inf")) + 1e-9:
                continue

            expanded += 1
            if progress_callback is not None:
                progress_batch.append(u)
                if len(progress_batch) >= 64:
                    progress_callback("shortest", expanded, tuple(progress_batch), u, "backward")
                    progress_batch.clear()

            if u in forward_best:
                total = distance + forward_best[u]
                if total < best_total:
                    best_total, meet = total, u

            for edge in graph["adj"].get(u, ()):
                v = edge["to"]
                nd = distance + float(edge.get("dist", 0.0))
                if nd >= backward_best.get(v, float("inf")) - 1e-9:
                    continue
                backward_best[v] = nd
                backward_parent[v] = u
                sequence += 1
                point = (graph["nodes"][v][0], graph["nodes"][v][1])
                heapq.heappush(
                    backward_q,
                    (nd + haversine(point, start_point), nd, sequence, v),
                )

    if progress_callback is not None and progress_batch:
        progress_callback("shortest", expanded, tuple(progress_batch), meet, "mixed")
    if meet is None:
        return None

    left = []
    u = meet
    while u is not None:
        left.append(u)
        u = forward_parent[u]
    left.reverse()

    right = []
    u = backward_parent.get(meet)
    while u is not None:
        right.append(u)
        u = backward_parent[u]

    return left + right, best_total


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


def _aggressive_quality_edge_cost(graph, u, edge, goal, quality_weight):
    """Агрессивный качественный поиск."""
    dist = max(edge["dist"], 1.0)
    factor = 1.0

    score_km = edge.get("score", 0.0) / max(dist / 1000.0, 0.001)
    exponent = max(-6.0, min(6.0, -quality_weight * score_km / 1000.0))
    factor *= math.exp(exponent)

    # Дополнительный явный приоритет местам, ради которых и нужен
    # качественный маршрут: парки, зелёные зоны, набережные и пешеходные улицы.
    criteria = edge.get("criteria", {})
    place_bonus = sum(criteria.get(k, 0.0) for k in (
        "park", "forest", "waterfront", "green_open", "garden",
        "natural_green", "pedestrian_street"
    ))
    if place_bonus > 0:
        factor *= math.exp(-min(1.4, place_bonus / 20.0))
    if edge.get("highway") == "pedestrian":
        factor *= 0.78

    before = haversine((graph["nodes"][u][0], graph["nodes"][u][1]), goal)
    v = edge["to"]
    after = haversine((graph["nodes"][v][0], graph["nodes"][v][1]), goal)
    progress_ratio = (before - after) / dist

    if progress_ratio < 0:
        factor *= math.exp(min(4.2, -progress_ratio * 3.8))
    elif progress_ratio < 0.30:
        factor *= math.exp((0.30 - progress_ratio) * 1.35)

    delta = edge.get("elevation_delta_m")
    if delta is not None:
        factor *= 1.0 + min(0.75, max(0.0, delta) / 34.0)
        factor *= 1.0 + min(0.16, max(0.0, -delta) / 95.0)

    if edge.get("stairs"):
        factor *= 1.30

    flags = graph.get("node_flags", {}).get(str(v), {})
    crossing_penalty = 0.0
    if flags.get("crossing"):
        crossing_penalty += 18.0 if flags.get("traffic_signals") else 30.0
    if flags.get("major_crossing"):
        crossing_penalty += 18.0

    return dist * factor + crossing_penalty


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
    exponent = max(-8.0, min(8.0, -quality_weight * ratio))
    return dist * factor * math.exp(exponent)


def _weighted_path(graph, start, goal, quality_weight, mode, max_distance=None,
                   progress_callback=None, progress_phase=None):
    if mode == "simple_v2":
        goal_point = (graph["nodes"][goal][0], graph["nodes"][goal][1])
        return _simple_v2_weighted_path(
            graph, start, goal, goal_point, quality_weight, max_distance,
            progress_callback=progress_callback,
            progress_phase=progress_phase or "candidate"
        )

    q = [(0.0, 0.0, start)]
    best = {start: 0.0}
    distance = {start: 0.0}
    parent = {start: None}
    expanded = 0
    progress_batch = []

    while q:
        cost, dist_so_far, u = heapq.heappop(q)
        if cost != best.get(u):
            continue

        expanded += 1
        if progress_callback is not None:
            progress_batch.append(u)
            if len(progress_batch) >= 64:
                progress_callback(progress_phase or "candidate", expanded,
                                  tuple(progress_batch), u, "forward")
                progress_batch.clear()
        if u == goal:
            break

        for edge in graph["adj"].get(u, ()):
            if mode == "simple":
                edge_cost = _simple_edge_cost(
                    graph, u, edge,
                    (graph["nodes"][goal][0], graph["nodes"][goal][1]),
                    quality_weight,
                )
            elif mode == "aggressive":
                edge_cost = _aggressive_quality_edge_cost(
                    graph, u, edge,
                    (graph["nodes"][goal][0], graph["nodes"][goal][1]),
                    quality_weight,
                )
            else:
                edge_cost = _quality_edge_cost(edge, quality_weight)

            v = edge["to"]
            nd = dist_so_far + edge["dist"]
            if max_distance is not None and nd > max_distance + 1e-6:
                continue
            nc = cost + edge_cost
            if nc < best.get(v, float("inf")) - 1e-9:
                best[v] = nc
                distance[v] = nd
                parent[v] = u
                heapq.heappush(q, (nc, nd, v))

    if progress_callback is not None and progress_batch:
        progress_callback(progress_phase or "candidate", expanded,
                          tuple(progress_batch), goal, "forward")
    if goal not in distance:
        return None

    node_ids = []
    u = goal
    while u is not None:
        node_ids.append(u)
        u = parent[u]
    node_ids.reverse()

    return node_ids, distance[goal], expanded


def _crossing_counts(graph, node_ids):
    crossings = 0
    signals = 0
    for nid in node_ids:
        flags = graph.get("node_flags", {}).get(str(nid), {})
        if flags.get("crossing"):
            crossings += 1
        if flags.get("traffic_signals"):
            signals += 1
    return crossings, signals


def _aggressive_utility(candidate, shortest_m):
    extra_km = max(0.0, (candidate["distance_m"] - shortest_m) / 1000.0)
    shortest_km = max(shortest_m / 1000.0, 0.001)
    density = candidate["score"] / max(candidate["distance_m"] / 1000.0, 0.001)
    normalized_quality = density * shortest_km
    return (
        normalized_quality
        - 6.5 * extra_km
        - 8.0 * extra_km * extra_km
        - 1.6 * candidate.get("crossings", 0)
    )


def _route_candidate(graph, start, goal, max_distance, quality_weight, mode,
                      progress_callback=None):
    phase = f"candidate · вес {quality_weight:g}"
    result = _weighted_path(
        graph, start, goal, quality_weight, mode, max_distance,
        progress_callback=progress_callback,
        progress_phase=phase
    )
    if result is None:
        return None

    node_ids, distance_m, expanded = result
    if distance_m > max_distance + 1e-6:
        return None

    score, criteria, ascent, descent, stairs = path_score(graph, node_ids)
    crossing_count, signal_count = _crossing_counts(graph, node_ids)

    goal_point = (graph["nodes"][goal][0], graph["nodes"][goal][1])
    if mode == "simple_v2":
        simple_cost = _simple_v2_path_cost(graph, node_ids, goal_point, quality_weight)
    else:
        simple_cost = 0.0
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
        "crossings": crossing_count,
        "signals": signal_count,
        "simple_cost": simple_cost,
        "simple_metrics": _simple_v2_path_metrics(graph, node_ids) if mode == "simple_v2" else {},
        "fallback": False,
    }


AUTO_DETOURS = tuple(round(x / 100.0, 2) for x in range(110, 181, 10))


def _search_weights(mode, automatic):
    if mode in {"simple", "simple_v2"}:
        if automatic:
            return [0.0, 5.0, 12.0, 24.0, 45.0, 68.0]
        return [0.0, 1.0, 2.5, 5.0, 9.0, 15.0, 24.0, 36.0, 52.0, 72.0]
    if mode == "aggressive":
        if automatic:
            return [0.0, 18.0, 40.0, 80.0, 160.0, 300.0, 500.0, 750.0]
        return [0.0, 4.0, 10.0, 20.0, 40.0, 80.0, 140.0, 220.0, 340.0, 520.0, 780.0, 1100.0]
    if automatic:
        return [0.0, 25.0, 75.0, 150.0, 300.0, 600.0]
    return [0.0, 4.0, 10.0, 20.0, 40.0, 80.0, 140.0, 220.0, 340.0, 520.0, 780.0, 1100.0]


def _solve_for_budget(graph, start, goal, shortest_ids, shortest_m,
                      max_distance, mode, automatic=False,
                      progress_callback=None):
    if max_distance <= shortest_m + 1e-6:
        score, criteria, ascent, descent, stairs = path_score(graph, shortest_ids)
        return {
            "node_ids": shortest_ids,
            "distance_m": shortest_m,
            "score": score,
            "criteria": criteria,
            "expanded": 0,
            "ascent": ascent,
            "descent": descent,
            "stairs": stairs,
            "crossings": _crossing_counts(graph, shortest_ids)[0],
            "signals": _crossing_counts(graph, shortest_ids)[1],
            "simple_cost": shortest_m,
            "simple_metrics": _simple_v2_path_metrics(graph, shortest_ids) if mode == "simple_v2" else {},
            "fallback": False,
        }

    candidates = []
    for quality_weight in _search_weights(mode, automatic):
        candidate = _route_candidate(
            graph, start, goal, max_distance,
            quality_weight, mode
        )
        if candidate is not None:
            candidates.append(candidate)

    if not candidates:
        score, criteria, ascent, descent, stairs = path_score(graph, shortest_ids)
        return {
            "node_ids": shortest_ids,
            "distance_m": shortest_m,
            "score": score,
            "criteria": criteria,
            "expanded": 0,
            "ascent": ascent,
            "descent": descent,
            "stairs": stairs,
            "crossings": _crossing_counts(graph, shortest_ids)[0],
            "signals": _crossing_counts(graph, shortest_ids)[1],
            "simple_cost": shortest_m,
            "simple_metrics": _simple_v2_path_metrics(graph, shortest_ids) if mode == "simple_v2" else {},
            "fallback": True,
        }

    # Автоматически перебираем несколько весов качества в пределах бюджета.
    if automatic:
        if mode in {"simple", "simple_v2"}:
            return min(candidates, key=lambda c: (c["simple_cost"], -c["score"], c["distance_m"]))
        if mode == "aggressive":
            return max(
                candidates,
                key=lambda c: (
                    _aggressive_utility(c, shortest_m),
                    c["score"],
                    -c["distance_m"],
                ),
            )
        return max(
            candidates,
            key=lambda c: (
                c["score"] / max(c["distance_m"] / 1000.0, 0.001),
                c["score"],
                -c["distance_m"],
            ),
        )

    if mode in {"simple", "simple_v2"}:
        return min(
            candidates,
            key=lambda c: (c["simple_cost"], -c["score"], c["distance_m"])
        )

    if mode == "aggressive":
        return max(
            candidates,
            key=lambda c: (
                _aggressive_utility(c, shortest_m),
                c["score"],
                -c["distance_m"],
            ),
        )

    return max(candidates, key=lambda c: (c["score"], -c["distance_m"]))


def find_route(graph, start_point, goal_point, detour_factor=1.35,
               mode="quality", automatic=False, progress_callback=None):    mode_value = str(mode).lower()
    if mode_value in {"simple_v2", "simple-v2", "simple2"}:
        mode = "simple_v2"
    elif mode_value in {"simple", "simplified"}:
        mode = "simple"
    elif mode_value in {"aggressive", "quality_aggressive"}:
        mode = "aggressive"
    else:
        mode = "quality"

    start, goal, snap_s, snap_g = choose_endpoints(graph, start_point, goal_point)
    if progress_callback is not None:
        progress_callback("snap", 0, (), start, "forward")
    shortest = shortest_path(graph, start, goal, progress_callback=progress_callback)
    if shortest is None:
        raise ValueError("Между выбранными точками нет пешеходного пути")

    shortest_ids, shortest_m = shortest

    if automatic:
        selected = None
        tested = []

        for factor in AUTO_DETOURS:
            max_distance = shortest_m * factor
            candidate = _solve_for_budget(
                graph, start, goal, shortest_ids, shortest_m,
                max_distance, mode, automatic=True,
                progress_callback=progress_callback
            )
            tested.append({
                "detour_pct": round(factor * 100),
                "distance_m": round(candidate["distance_m"], 1),
                "score": round(candidate["score"], 2),
                "score_per_km": round(
                    candidate["score"] / max(candidate["distance_m"] / 1000.0, 0.001), 2
                ),
                "fallback": bool(candidate["fallback"]),
            })

            if mode == "aggressive":
                current_value = _aggressive_utility(candidate, shortest_m)
            elif mode == "simple_v2":
                current_value = -candidate["simple_cost"]
            else:
                current_value = candidate["score"] / max(
                    candidate["distance_m"] / 1000.0, 0.001
                )
            if selected is None:
                selected = (current_value, candidate, factor)
            else:
                old_value = selected[0]
                if (current_value > old_value + 1e-9 or
                    (abs(current_value - old_value) <= 1e-9 and
                     candidate["score"] > selected[1]["score"])):
                    selected = (current_value, candidate, factor)

        if selected is None:
            raise ValueError("Не удалось подобрать автоматический маршрут")

        _, best, selected_factor = selected
        node_ids = best["node_ids"]
        coordinates = [
            [graph["nodes"][nid][0], graph["nodes"][nid][1]]
            for nid in node_ids
        ]
        return _result(
            start_point, goal_point, snap_s, snap_g,
            best["distance_m"], shortest_m,
            best["score"], best["criteria"], coordinates,
            best["expanded"], best["fallback"],
            shortest_m * selected_factor,
            mode, best["ascent"], best["descent"], best["stairs"],
            best.get("crossings", 0), best.get("signals", 0),
            selected_factor, True, tested, _segment_styles(graph, node_ids),
            best.get("simple_metrics", {})
        )

    detour_factor = max(1.0, min(float(detour_factor), 1.8))
    max_distance = shortest_m * detour_factor
    best = _solve_for_budget(
        graph, start, goal, shortest_ids, shortest_m,
        max_distance, mode, automatic=False,
        progress_callback=progress_callback
    )

    node_ids = best["node_ids"]
    coordinates = [
        [graph["nodes"][nid][0], graph["nodes"][nid][1]]
        for nid in node_ids
    ]

    return _result(
        start_point, goal_point, snap_s, snap_g,
        best["distance_m"], shortest_m,
        best["score"], best["criteria"], coordinates,
        best["expanded"], best["fallback"], max_distance,
        mode, best["ascent"], best["descent"], best["stairs"],
        best.get("crossings", 0), best.get("signals", 0),
        detour_factor, False, None, _segment_styles(graph, node_ids),
        best.get("simple_metrics", {})
    )

def _segment_styles(graph, node_ids):
    styles = []
    for a, b in zip(node_ids, node_ids[1:]):
        edge = _find_edge(graph, a, b)
        if edge is None:
            styles.append({
                "quality": 0.0,
                "kind": "neutral",
                "stairs": False,
                "grade_pct": None,
            })
            continue

        dist_km = max(edge["dist"] / 1000.0, 0.001)
        quality = edge.get("score", 0.0) / dist_km

        # Для визуальной оценки учитываем уклон и лестницы.
        delta = edge.get("elevation_delta_m")
        if delta is not None:
            grade = abs(delta) / max(edge["dist"], 1.0) * 100.0
            if delta > 0:
                quality -= min(8.0, grade * 0.9)
            elif delta < 0:
                quality -= min(2.0, grade * 0.18)
        else:
            grade = None

        if edge.get("stairs"):
            quality -= 8.0

        if quality >= 6.0:
            kind = "good"
        elif quality <= -4.0:
            kind = "bad"
        else:
            kind = "neutral"

        styles.append({
            "quality": round(quality, 2),
            "kind": kind,
            "stairs": bool(edge.get("stairs")),
            "grade_pct": round(grade, 2) if grade is not None else None,
        })
    return styles


def _result(start_point, goal_point, snap_s, snap_g, distance_m, shortest_m,
            score, criteria, coordinates, expanded, fallback,
            max_distance=None, mode="quality", ascent=0.0, descent=0.0,
            stairs=0, crossings=0, signals=0,
            selected_detour=1.0, automatic=False, tested_detours=None,
            segments=None, simple_metrics=None):
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
        "crossings_count": int(crossings),
        "signal_crossings_count": int(signals),
        "selected_detour_pct": round(selected_detour * 100),
        "automatic": automatic,
        "tested_detours": tested_detours,
        "segments": segments if segments is not None else [],
        "simple_metrics": simple_metrics if simple_metrics is not None else {},
        "fallback": fallback,
        "repeated_points": len(coordinates) - len({
            (round(p[0], 7), round(p[1], 7)) for p in coordinates
        }),
    }
