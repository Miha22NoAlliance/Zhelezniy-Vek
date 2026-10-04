"""Simple v2 pedestrian routing model."""
from __future__ import annotations

import heapq
import math
from itertools import count

ROAD_FACTORS = {
    "trunk": 0.96, "trunk_link": 1.00,
    "primary": 0.95, "primary_link": 0.99,
    "secondary": 0.97, "secondary_link": 1.00,
    "tertiary": 0.99, "tertiary_link": 1.01,
    "residential": 1.00, "living_street": 1.00,
    "pedestrian": 0.98, "cycleway": 1.08,
    "service": 1.20, "footway": 1.14,
    "path": 1.24, "track": 1.38, "steps": 1.65,
}
CONTINUOUS_CLASSES = {
    "trunk", "primary", "secondary", "tertiary", "residential", "living_street"
}
LOW_LEVEL_CLASSES = {"service", "footway", "path", "track", "cycleway"}
HEURISTIC_SCALE = 0.15


def _haversine(a, b):
    lat1, lon1 = map(math.radians, a)
    lat2, lon2 = map(math.radians, b)
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 6371000.0 * 2 * math.asin(math.sqrt(h))


def _turn_angle(graph, previous, current, following):
    if previous is None:
        return 0.0
    a = graph["nodes"][previous]
    b = graph["nodes"][current]
    c = graph["nodes"][following]
    lat = math.radians(b[0])
    sx = math.cos(lat)
    incoming = ((b[1] - a[1]) * sx, b[0] - a[0])
    outgoing = ((c[1] - b[1]) * sx, c[0] - b[0])
    ni, no = math.hypot(*incoming), math.hypot(*outgoing)
    if ni < 1e-12 or no < 1e-12:
        return 0.0
    cosine = max(-1.0, min(1.0, (incoming[0] * outgoing[0] + incoming[1] * outgoing[1]) / (ni * no)))
    return math.degrees(math.acos(cosine))


def _incoming_edge(graph, previous, current):
    if previous is None:
        return None
    for edge in graph["adj"].get(previous, ()):
        if edge["to"] == current:
            return edge
    return None


def _street_identity(edge):
    if not edge:
        return None
    name = (edge.get("road_name") or "").strip()
    ref = (edge.get("road_ref") or "").strip()
    if name or ref:
        return (name, ref)
    return ("highway", edge.get("highway", ""))


def _crossing_penalty(graph, previous, current):
    flags = graph.get("node_flags", {}).get(str(current), {})
    if not flags.get("crossing"):
        return 0.0
    edge = _incoming_edge(graph, previous, current)
    highway = edge.get("highway", "") if edge else ""
    lanes = int(edge.get("lanes", 0) or 0) if edge else 0
    penalty = 4.0 if flags.get("traffic_signals") else 8.0
    penalty += {"primary": 7.0, "secondary": 5.0, "tertiary": 3.0}.get(highway, 0.0)
    penalty += min(8.0, max(0, lanes - 2) * 2.0)
    if flags.get("major_crossing"):
        penalty += 10.0
    return penalty


def _access_penalty(graph, current):
    flags = graph.get("node_flags", {}).get(str(current), {})
    access = str(flags.get("access", "")).lower()
    foot = str(flags.get("foot", "")).lower()
    barrier = str(flags.get("barrier", "")).lower()
    penalty = 0.0
    if foot in {"no", "private"}:
        penalty += 90.0
    elif access in {"no", "private"}:
        penalty += 70.0
    barrier_penalties = {
        "gate": 5.0, "lift_gate": 6.0, "swing_gate": 5.0,
        "wicket": 7.0, "kissing_gate": 9.0, "stile": 12.0,
        "turnstile": 14.0, "cycle_barrier": 8.0, "bollard": 4.0,
        "bus_trap": 35.0, "block": 25.0, "jersey_barrier": 25.0,
    }
    penalty += barrier_penalties.get(barrier, 0.0)
    return penalty


def _edge_cost(graph, previous, current, edge, goal, quality_weight):
    target = edge["to"]
    dist = max(float(edge.get("dist", 1.0)), 1.0)
    highway = edge.get("highway", "")
    factor = ROAD_FACTORS.get(highway, 1.10)

    here, there = graph["nodes"][current], graph["nodes"][target]
    before = _haversine((here[0], here[1]), goal)
    after = _haversine((there[0], there[1]), goal)
    progress_ratio = (before - after) / dist
    if progress_ratio < -0.05:
        factor *= 1.0 + min(1.20, -progress_ratio * 0.90)
    elif progress_ratio < 0.25:
        factor *= 1.0 + (0.25 - progress_ratio) * 0.20

    incoming = _incoming_edge(graph, previous, current)
    old_class = incoming.get("highway", "") if incoming else ""
    old_identity = _street_identity(incoming)
    new_identity = _street_identity(edge)

    if previous is not None:
        if incoming and incoming.get("way_id") == edge.get("way_id"):
            factor *= 0.945
        elif old_identity and new_identity and old_identity == new_identity:
            factor *= 0.960
        elif old_class == highway and highway in CONTINUOUS_CLASSES:
            factor *= 0.975
        elif old_class in CONTINUOUS_CLASSES and highway in LOW_LEVEL_CLASSES:
            factor *= 1.10
        elif old_class in LOW_LEVEL_CLASSES and highway in CONTINUOUS_CLASSES:
            factor *= 0.985

    if dist < 25.0 and highway in {"service", "path", "track"}:
        factor *= 1.07

    delta = edge.get("elevation_delta_m")
    if delta is not None:
        factor *= 1.0 + min(0.80, max(0.0, delta) / 34.0)
        factor *= 1.0 + min(0.18, max(0.0, -delta) / 105.0)
    if edge.get("stairs"):
        factor *= 1.42

    score_per_m = float(edge.get("score", 0.0)) / dist
    factor *= math.exp(max(-1.5, min(1.5, -quality_weight * score_per_m)))

    turn = _turn_angle(graph, previous, current, target)
    if turn < 25:
        turn_penalty = 0.0
    elif turn < 55:
        turn_penalty = 3.0
    elif turn < 100:
        turn_penalty = 9.0
    elif turn < 150:
        turn_penalty = 20.0
    else:
        turn_penalty = 38.0

    access_penalty = _access_penalty(graph, current)
    crossing_penalty = _crossing_penalty(graph, previous, current)
    return dist * factor + turn_penalty + access_penalty + crossing_penalty


def weighted_path(graph, start, goal, goal_point, quality_weight, max_distance=None):
    initial = (None, start)
    sequence = count()
    queue = [(0.0 + HEURISTIC_SCALE * _haversine(
        (graph["nodes"][start][0], graph["nodes"][start][1]), goal_point
    ), 0.0, next(sequence), initial)]
    best = {initial: 0.0}
    physical = {initial: 0.0}
    parent = {initial: None}
    final = None
    expanded = 0

    while queue:
        f_score, distance, _, state = heapq.heappop(queue)
        cost = best.get(state)
        if cost is None:
            continue
        if f_score > cost + HEURISTIC_SCALE * _haversine(
            (graph["nodes"][state[1]][0], graph["nodes"][state[1]][1]), goal_point
        ) + 1e-7:
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
            edge_cost = _edge_cost(graph, previous, current, edge, goal_point, quality_weight)
            next_cost = cost + edge_cost
            if next_cost < best.get(next_state, float("inf")) - 1e-9:
                best[next_state] = next_cost
                physical[next_state] = next_distance
                parent[next_state] = state
                heuristic = HEURISTIC_SCALE * _haversine(
                    (graph["nodes"][target][0], graph["nodes"][target][1]), goal_point
                )
                heapq.heappush(queue, (next_cost + heuristic, next_distance, next(sequence), next_state))

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


def path_metrics(graph, node_ids):
    distance = 0.0
    crossings = 0
    signals = 0
    major_crossings = 0
    turns = 0
    sharp_turns = 0
    u_turns = 0
    street_changes = 0
    ascent = 0.0
    descent = 0.0
    previous_edge = None

    for i, (current, target) in enumerate(zip(node_ids, node_ids[1:])):
        edge = next((e for e in graph["adj"].get(current, ()) if e["to"] == target), None)
        if edge is None:
            continue
        distance += float(edge.get("dist", 0.0))
        if previous_edge is not None:
            identity_a = _street_identity(previous_edge)
            identity_b = _street_identity(edge)
            if identity_a != identity_b:
                street_changes += 1
            angle = _turn_angle(graph, node_ids[i - 1], current, target)
            if angle >= 35:
                turns += 1
            if angle >= 100:
                sharp_turns += 1
            if angle >= 150:
                u_turns += 1

        flags = graph.get("node_flags", {}).get(str(target), {})
        if flags.get("crossing"):
            crossings += 1
        if flags.get("traffic_signals"):
            signals += 1
        if flags.get("major_crossing"):
            major_crossings += 1

        delta = edge.get("elevation_delta_m")
        if delta is not None:
            if delta > 0:
                ascent += delta
            elif delta < 0:
                descent += -delta
        previous_edge = edge

    directness = 1.0
    if len(node_ids) >= 2:
        a, b = graph["nodes"][node_ids[0]], graph["nodes"][node_ids[-1]]
        directness = _haversine((a[0], a[1]), (b[0], b[1])) / max(distance, 1.0)
        directness = max(0.0, min(1.0, directness))

    base_seconds = distance / 1.35
    eta_seconds = (
        base_seconds
        + turns * 2.5
        + sharp_turns * 7.0
        + u_turns * 14.0
        + crossings * 5.0
        + signals * 6.0
        + major_crossings * 8.0
        + max(0.0, ascent) * 1.6
        + max(0.0, -descent) * 0.4
    )

    return {
        "distance_m": round(distance, 1),
        "turns": turns,
        "sharp_turns": sharp_turns,
        "u_turns": u_turns,
        "street_changes": street_changes,
        "crossings": crossings,
        "signals": signals,
        "major_crossings": major_crossings,
        "directness": round(directness, 4),
        "eta_seconds": round(eta_seconds),
        "eta_minutes": round(eta_seconds / 60.0, 1),
        "ascent": round(ascent, 1),
        "descent": round(descent, 1),
    }
