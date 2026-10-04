"""State-aware cost model for the Simple v2 pedestrian router.

The search state includes the previous node, allowing turn and street-class
continuity costs without changing the graph schema.
"""
from __future__ import annotations

import heapq
import math

ROAD_FACTORS = {
    "trunk": 0.91, "trunk_link": 0.96,
    "primary": 0.93, "primary_link": 0.97,
    "secondary": 0.96, "secondary_link": 0.99,
    "tertiary": 0.99, "tertiary_link": 1.01,
    "residential": 1.00, "living_street": 1.00,
    "pedestrian": 0.98, "cycleway": 1.08,
    "service": 1.20, "footway": 1.16,
    "path": 1.25, "track": 1.38, "steps": 1.65,
}
CONTINUOUS_CLASSES = {"trunk", "primary", "secondary", "tertiary", "residential", "living_street"}


def _haversine(a, b):
    lat1, lon1 = map(math.radians, a)
    lat2, lon2 = map(math.radians, b)
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 6371000.0 * 2 * math.asin(math.sqrt(h))


def _turn_penalty(graph, previous, current, following):
    if previous is None:
        return 0.0
    a = graph["nodes"][previous]
    b = graph["nodes"][current]
    c = graph["nodes"][following]
    lat = math.radians(b[0])
    scale_x = math.cos(lat)
    incoming = ((b[1] - a[1]) * scale_x, b[0] - a[0])
    outgoing = ((c[1] - b[1]) * scale_x, c[0] - b[0])
    ni = math.hypot(*incoming)
    no = math.hypot(*outgoing)
    if ni < 1e-12 or no < 1e-12:
        return 0.0
    cosine = max(-1.0, min(1.0, (incoming[0] * outgoing[0] + incoming[1] * outgoing[1]) / (ni * no)))
    angle = math.degrees(math.acos(cosine))
    if angle < 25:
        return 0.0
    if angle < 55:
        return 3.0
    if angle < 100:
        return 9.0
    if angle < 150:
        return 20.0
    return 38.0


def _edge_cost(graph, previous, current, edge, goal, quality_weight):
    target = edge["to"]
    dist = max(float(edge.get("dist", 1.0)), 1.0)
    highway = edge.get("highway", "")
    factor = ROAD_FACTORS.get(highway, 1.10)

    here = graph["nodes"][current]
    there = graph["nodes"][target]
    before = _haversine((here[0], here[1]), goal)
    after = _haversine((there[0], there[1]), goal)
    progress_ratio = (before - after) / dist
    if progress_ratio < -0.05:
        factor *= 1.0 + min(1.35, -progress_ratio * 0.95)
    elif progress_ratio < 0.25:
        factor *= 1.0 + (0.25 - progress_ratio) * 0.20

    if previous is not None:
        incoming = next((e for e in graph["adj"].get(previous, ()) if e["to"] == current), None)
        old_class = incoming.get("highway", "") if incoming else ""
        if old_class == highway and highway in CONTINUOUS_CLASSES:
            factor *= 0.965
        elif old_class in CONTINUOUS_CLASSES and highway in {"service", "footway", "path", "track"}:
            factor *= 1.10
        elif old_class in {"service", "footway", "path", "track"} and highway in CONTINUOUS_CLASSES:
            factor *= 0.985

    delta = edge.get("elevation_delta_m")
    if delta is not None:
        factor *= 1.0 + min(0.85, max(0.0, delta) / 32.0)
        factor *= 1.0 + min(0.20, max(0.0, -delta) / 100.0)
    if edge.get("stairs"):
        factor *= 1.42

    score_per_m = float(edge.get("score", 0.0)) / dist
    factor *= math.exp(max(-1.5, min(1.5, -quality_weight * score_per_m)))
    return dist * factor + _turn_penalty(graph, previous, current, target)


def weighted_path(graph, start, goal, goal_point, quality_weight, max_distance=None):
    """Return (node_ids, physical_distance_m, expanded_labels), or None."""
    initial = (None, start)
    queue = [(0.0, 0.0, initial)]
    best = {initial: 0.0}
    physical = {initial: 0.0}
    parent = {initial: None}
    final = None
    expanded = 0

    while queue:
        cost, distance, state = heapq.heappop(queue)
        if cost > best.get(state, float("inf")) + 1e-9:
            continue
        previous, current = state
        expanded += 1
        if current == goal:
            final = state
            break
        for edge in graph["adj"].get(current, ()):
            target = edge["to"]
            edge_dist = max(float(edge.get("dist", 1.0)), 0.0)
            next_distance = distance + edge_dist
            if max_distance is not None and next_distance > max_distance + 1e-6:
                continue
            next_state = (current, target)
            next_cost = cost + _edge_cost(graph, previous, current, edge, goal_point, quality_weight)
            if next_cost < best.get(next_state, float("inf")) - 1e-9:
                best[next_state] = next_cost
                physical[next_state] = next_distance
                parent[next_state] = state
                heapq.heappush(queue, (next_cost, next_distance, next_state))

    if final is None:
        return None
    states = []
    state = final
    while state is not None:
        states.append(state)
        state = parent[state]
    states.reverse()
    node_ids = [states[0][1]]
    node_ids.extend(s[1] for s in states[1:])
    return node_ids, physical[final], expanded


def path_cost(graph, node_ids, goal_point, quality_weight=0.0):
    total = 0.0
    previous = None
    for current, target in zip(node_ids, node_ids[1:]):
        edge = next((e for e in graph["adj"].get(current, ()) if e["to"] == target), None)
        if edge is not None:
            total += _edge_cost(graph, previous, current, edge, goal_point, quality_weight)
        previous, current = current, target
    return total
