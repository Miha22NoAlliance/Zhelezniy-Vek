from __future__ import annotations

import json
from pathlib import Path
from urllib.parse import urlencode
from urllib.request import Request, urlopen

from routing import haversine

LIPETSK_BBOX = (52.50, 39.40, 52.72, 39.78)
OVERPASS = "https://overpass-api.de/api/interpreter"
EXCLUDE = {"motorway", "motorway_link", "construction", "proposed", "raceway"}


def load_weights():
    p = Path(__file__).resolve().parent / "config" / "weights.json"
    return json.loads(p.read_text(encoding="utf-8"))["weights"]


def num(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def rate(tags, weights, context):
    c = {}
    highway = tags.get("highway", "")
    surface = tags.get("surface", "").lower()
    smooth = tags.get("smoothness", "").lower()
    sidewalk = tags.get("sidewalk", "").lower()
    access = tags.get("access", "").lower()
    foot = tags.get("foot", "").lower()
    lanes = num(tags.get("lanes"))
    maxspeed = num(str(tags.get("maxspeed", "")).split()[0])

    def add(name):
        c[name] = weights[name]

    natural = context.get("natural_score", 0)
    if natural >= 0.8:
        add("park"); add("natural_green")
    elif natural >= 0.5:
        add("green_open")
    if context.get("forest"): add("forest")
    if context.get("waterfront"): add("waterfront")
    if context.get("water_near"): add("water_near")
    if context.get("tree_line"): add("tree_line")
    if context.get("garden"): add("garden")

    if highway in {"footway", "path"}: add("very_quiet")
    if highway in {"residential", "living_street"}: add("quiet_residential")
    if highway == "pedestrian": add("pedestrian_street")
    if highway in {"footway", "path", "pedestrian", "cycleway", "track", "steps"}: add("no_motor_traffic")
    if highway not in {"primary", "secondary", "tertiary", "primary_link", "secondary_link", "tertiary_link"}: add("away_major_road")
    if highway in {"primary", "primary_link", "secondary", "secondary_link"}: add("major_road")
    if lanes >= 3 or maxspeed >= 60: add("heavy_traffic")
    if highway in {"trunk", "trunk_link"}: add("noisy_zone")
    if context.get("industrial"): add("industrial_zone")
    if highway in {"primary", "secondary", "tertiary"}: add("road_near")
    if context.get("railway_near"): add("railway_near")
    if highway in {"residential", "living_street"}: add("pleasant_density")

    if smooth in {"excellent", "very_good"} or surface in {"asphalt", "paved", "concrete"}: add("excellent_surface")
    elif smooth == "good": add("good_surface")
    if highway in {"footway", "path", "pedestrian"}: add("wide_path"); add("dedicated_path")
    if sidewalk in {"both", "left", "right", "yes", "separate"}: add("good_sidewalk")
    if context.get("rest_nearby"): add("rest_nearby")
    if context.get("service_nearby"): add("service_nearby")
    if highway in {"footway", "path", "pedestrian", "living_street"}: add("car_protected")

    if tags.get("lit", "").lower() == "yes": add("safe_crossing")
    if context.get("signal_crossing"): add("signal_crossing")
    if context.get("dangerous_crossing"): add("dangerous_crossing")
    if context.get("major_crossing"): add("major_crossing")
    if highway == "steps": add("stairs")

    if surface in {"ground", "dirt", "unpaved", "gravel", "sett", "sand"} or smooth in {"bad", "very_bad", "horrible", "impassable"}:
        add("bad_surface")
    if smooth in {"horrible", "very_horrible", "impassable"}:
        add("very_bad_surface")
    if highway in {"path", "track"} and sidewalk == "no": add("narrow_path")
    if highway in {"primary", "secondary", "tertiary", "residential", "service"} and sidewalk in {"", "no"}: add("no_sidewalk")

    incline = tags.get("incline", "").replace("%", "")
    inc = abs(num(incline))
    if inc >= 8: add("very_steep")
    elif inc >= 4: add("steep")

    if access in {"no", "private", "customers"} or foot in {"no", "private"}: add("restricted")
    return c


def bbox_area(items):
    for item in items:
        geom = item.get("geometry") or []
        if len(geom) < 3:
            continue
        lats = [p["lat"] for p in geom]
        lons = [p["lon"] for p in geom]
        item["bbox"] = [min(lats), min(lons), max(lats), max(lons)]


def point_in_poly(lat, lon, geom):
    inside = False
    j = len(geom) - 1
    for i in range(len(geom)):
        yi, xi = geom[i]["lat"], geom[i]["lon"]
        yj, xj = geom[j]["lat"], geom[j]["lon"]
        if ((yi > lat) != (yj > lat)) and (lon < (xj-xi) * (lat-yi) / ((yj-yi) or 1e-12) + xi):
            inside = not inside
        j = i
    return inside


def context_for(mid, areas):
    lat, lon = mid
    context = {}
    for area in areas:
        bb = area.get("bbox")
        if not bb or not (bb[0] - 0.002 <= lat <= bb[2] + 0.002 and bb[1] - 0.003 <= lon <= bb[3] + 0.003):
            continue

        tags = area["tags"]
        inside = point_in_poly(lat, lon, area["geometry"])
        kind = tags.get("leisure") or tags.get("landuse") or tags.get("natural") or tags.get("railway")

        if not inside:
            continue

        if kind in {"park", "recreation_ground"}:
            context["natural_score"] = max(context.get("natural_score", 0), 1.0)
        if kind in {"forest", "wood"}:
            context["forest"] = True
            context["natural_score"] = max(context.get("natural_score", 0), 0.85)
        if kind in {"grass", "meadow"}:
            context["natural_score"] = max(context.get("natural_score", 0), 0.65)
        if kind == "garden":
            context["garden"] = True
            context["natural_score"] = max(context.get("natural_score", 0), 0.7)
        if kind in {"water", "riverbank"}:
            context["water_near"] = True
            context["waterfront"] = True
        if kind == "industrial":
            context["industrial"] = True
        if tags.get("railway"):
            context["railway_near"] = True

    return context


def build_or_load_graph(path: Path):
    if path.exists():
        return json.loads(path.read_text(encoding="utf-8"))

    b = LIPETSK_BBOX
    query = f'''[out:json][timeout:120];
(
  way["highway"]["highway"!~"motorway|motorway_link|construction|proposed|raceway"]({b[0]},{b[1]},{b[2]},{b[3]});
  way["leisure"~"park|garden|recreation_ground"]({b[0]},{b[1]},{b[2]},{b[3]});
  way["landuse"~"forest|grass|meadow|industrial"]({b[0]},{b[1]},{b[2]},{b[3]});
  way["natural"~"wood|water"]({b[0]},{b[1]},{b[2]},{b[3]});
  way["railway"]({b[0]},{b[1]},{b[2]},{b[3]});
);
out body geom;'''

    form = urlencode({"data": query}).encode("utf-8")
    req = Request(
        OVERPASS,
        data=form,
        headers={
            "User-Agent": "WalkRoute-Demo/0.1",
            "Content-Type": "application/x-www-form-urlencoded; charset=utf-8",
        },
        method="POST",
    )
    with urlopen(req, timeout=180) as response:
        raw = json.loads(response.read().decode("utf-8"))

    weights = load_weights()
    roads, areas, nodes = [], [], {}

    for element in raw["elements"]:
        tags = element.get("tags", {})
        geom = element.get("geometry", [])
        if not geom:
            continue

        if "highway" in tags:
            coords = []
            for point in geom:
                node_id = f'{point["lat"]:.7f}:{point["lon"]:.7f}'
                nodes[node_id] = [point["lat"], point["lon"]]
                coords.append(node_id)
            roads.append((coords, tags))
        else:
            areas.append({"tags": tags, "geometry": geom})

    bbox_area(areas)
    adj = {node_id: [] for node_id in nodes}
    edge_count = 0

    for coords, tags in roads:
        oneway = tags.get("oneway") == "yes"
        for a, bnode in zip(coords, coords[1:]):
            pa = tuple(nodes[a]); pb = tuple(nodes[bnode])
            dist = haversine(pa, pb)
            if dist < 1:
                continue

            midpoint = ((pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2)
            criteria_rates = rate(tags, weights, context_for(midpoint, areas))
            contribution = {k: v * (dist / 1000.0) for k, v in criteria_rates.items()}
            score = sum(contribution.values())

            edge = {"to": bnode, "dist": dist, "score": score, "criteria": contribution}
            adj[a].append(edge)
            edge_count += 1

            if not oneway:
                adj[bnode].append({"to": a, "dist": dist, "score": score, "criteria": contribution})
                edge_count += 1

    graph = {
        "nodes": nodes,
        "adj": adj,
        "meta": {"bbox": LIPETSK_BBOX, "nodes": len(nodes), "edges": edge_count, "source": "OpenStreetMap via Overpass API"},
    }

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(graph, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
    return graph
