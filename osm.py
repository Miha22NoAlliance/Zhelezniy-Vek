from __future__ import annotations

import json
import math
import struct
import zlib
from pathlib import Path

from dem import open_dem
from routing import haversine

LIPETSK_BBOX = (52.5320, 39.4596, 52.6457, 39.7153)
PBF_FILENAME = "planet_39.4596,52.532_39.7153,52.6457.osm.pbf"
GRAPH_SCHEMA = 6
EXCLUDE = {"motorway", "motorway_link", "construction", "proposed", "raceway"}
GRID_LAT, GRID_LON = 0.01, 0.015


def load_weights():
    p = Path(__file__).resolve().parent / "config" / "weights.json"
    return json.loads(p.read_text(encoding="utf-8"))["weights"]


def num(v, default=0.0):
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def is_walkable(tags):
    h = tags.get("highway", "")
    if not h or h in EXCLUDE:
        return False
    access = tags.get("access", "").lower()
    foot = tags.get("foot", "").lower()
    if access in {"no", "private"} and foot not in {"yes", "designated"}:
        return False
    if foot in {"no", "private"}:
        return False
    if h in {"trunk", "trunk_link"} and foot not in {"yes", "designated"}:
        return False
    return True


def rate(tags, weights, context):
    out = {}
    h = tags.get("highway", "")
    surface = tags.get("surface", "").lower()
    smooth = tags.get("smoothness", "").lower()
    sidewalk = tags.get("sidewalk", "").lower()
    access = tags.get("access", "").lower()
    foot = tags.get("foot", "").lower()
    lanes = num(tags.get("lanes"))
    ms = str(tags.get("maxspeed", "")).strip()
    maxspeed = num(ms.split()[0] if ms else "")

    def add(k):
        out[k] = weights[k]

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

    if h in {"footway", "path"}: add("very_quiet")
    if h in {"residential", "living_street"}: add("quiet_residential")
    if h == "pedestrian": add("pedestrian_street")
    if h in {"footway","path","pedestrian","cycleway","track","steps"}: add("no_motor_traffic")
    if h not in {"primary","secondary","tertiary","primary_link","secondary_link","tertiary_link"}:
        add("away_major_road")
    if h in {"primary","primary_link","secondary","secondary_link"}: add("major_road")
    if lanes >= 3 or maxspeed >= 60: add("heavy_traffic")
    if h in {"trunk","trunk_link"}: add("noisy_zone")
    if context.get("industrial"): add("industrial_zone")
    if h in {"primary","secondary","tertiary"}: add("road_near")
    if context.get("railway_near"): add("railway_near")
    if h in {"residential","living_street"}: add("pleasant_density")

    if smooth in {"excellent","very_good"} or surface in {"asphalt","paved","concrete"}: add("excellent_surface")
    elif smooth == "good": add("good_surface")
    if h in {"footway","path","pedestrian"}: add("wide_path"); add("dedicated_path")
    if sidewalk in {"both","left","right","yes","separate"}: add("good_sidewalk")
    if context.get("rest_nearby"): add("rest_nearby")
    if context.get("service_nearby"): add("service_nearby")
    if h in {"footway","path","pedestrian","living_street"}: add("car_protected")

    if tags.get("lit", "").lower() == "yes": add("safe_crossing")
    if context.get("signal_crossing"): add("signal_crossing")
    if context.get("dangerous_crossing"): add("dangerous_crossing")
    if context.get("major_crossing"): add("major_crossing")
    if h == "steps": add("stairs")

    if surface in {"ground","dirt","unpaved","gravel","sett","sand"} or smooth in {"bad","very_bad","horrible","impassable"}:
        add("bad_surface")
    if smooth in {"horrible","very_horrible","impassable"}: add("very_bad_surface")
    if h in {"path","track"} and sidewalk == "no": add("narrow_path")
    if h in {"primary","secondary","tertiary","residential","service"} and sidewalk in {"","no"}:
        add("no_sidewalk")

    inc = abs(num(tags.get("incline", "").replace("%", "")))
    if inc >= 8: add("very_steep")
    elif inc >= 4: add("steep")
    if access in {"no","private","customers"} or foot in {"no","private"}: add("restricted")
    return out


def _varint(data, i):
    v = 0
    shift = 0
    while i < len(data):
        b = data[i]; i += 1
        v |= (b & 127) << shift
        if not b & 128: return v, i
        shift += 7
        if shift >= 70: raise ValueError("Слишком длинный protobuf varint")
    raise ValueError("Оборванный protobuf varint")


def _zz(v):
    return (v >> 1) ^ -(v & 1)


def _fields(data):
    i = 0
    while i < len(data):
        key, i = _varint(data, i)
        field, wire = key >> 3, key & 7
        if field <= 0: raise ValueError("Некорректное protobuf-поле")
        if wire == 0:
            value, i = _varint(data, i)
        elif wire == 1:
            end = i + 8
            if end > len(data): raise ValueError("Оборванное fixed64 protobuf-поле")
            value, i = data[i:end], end
        elif wire == 2:
            n, i = _varint(data, i)
            end = i + n
            if end > len(data): raise ValueError("Оборванное protobuf-поле")
            value, i = data[i:end], end
        elif wire == 5:
            end = i + 4
            if end > len(data): raise ValueError("Оборванное fixed32 protobuf-поле")
            value, i = data[i:end], end
        else:
            raise ValueError(f"Неподдерживаемый protobuf wire type: {wire}")
        yield field, wire, value


def _vals(data, number, signed=False):
    out = []
    for f, wire, v in _fields(data):
        if f != number: continue
        if wire == 2:
            i = 0
            while i < len(v):
                x, i = _varint(v, i)
                out.append(_zz(x) if signed else x)
        elif wire == 0:
            out.append(_zz(v) if signed else v)
    return out


def _one(data, number, default=0):
    for f, wire, v in _fields(data):
        if f == number and wire == 0: return int(v)
    return default


def _bytes(data, number):
    for f, wire, v in _fields(data):
        if f == number and wire == 2: return bytes(v)
    return None


def _strings(data):
    return [bytes(v).decode("utf-8", errors="replace") for f,w,v in _fields(data) if f == 1 and w == 2]


def _tags(keys, vals, strings):
    return {strings[k]: strings[v] for k,v in zip(keys,vals) if 0 <= k < len(strings) and 0 <= v < len(strings)}


def _node(data, strings, granularity, lat_off, lon_off):
    node_id = _one(data, 1)
    lat = 1e-9 * (lat_off + granularity * _zz(_one(data, 8)))
    lon = 1e-9 * (lon_off + granularity * _zz(_one(data, 9)))
    return node_id, lat, lon, _tags(_vals(data,2), _vals(data,3), strings)


def _dense(data, strings, granularity, lat_off, lon_off):
    raw_ids = _vals(data, 1, signed=True)
    raw_lats = _vals(data, 8, signed=True)
    raw_lons = _vals(data, 9, signed=True)
    raw_tags = _vals(data, 10, signed=False)

    ids = []
    acc = 0
    for value in raw_ids:
        acc += value
        ids.append(acc)

    lats = []
    acc = 0
    for value in raw_lats:
        acc += value
        lats.append(acc)

    lons = []
    acc = 0
    for value in raw_lons:
        acc += value
        lons.append(acc)

    # DenseNodes.keys_vals: пары string-table indexes,
    # завершение тегов каждого узла обозначается нулевым key.
    tags_by_node = []
    pos = 0
    for _ in ids:
        tags = {}
        while pos < len(raw_tags):
            key_index = raw_tags[pos]
            pos += 1
            if key_index == 0:
                break
            if pos >= len(raw_tags):
                break
            value_index = raw_tags[pos]
            pos += 1
            if 0 <= key_index < len(strings) and 0 <= value_index < len(strings):
                tags[strings[key_index]] = strings[value_index]
        tags_by_node.append(tags)

    result = []
    count = min(len(ids), len(lats), len(lons))
    for i in range(count):
        lat = 1e-9 * (lat_off + granularity * lats[i])
        lon = 1e-9 * (lon_off + granularity * lons[i])
        result.append((
            ids[i],
            lat,
            lon,
            tags_by_node[i] if i < len(tags_by_node) else {},
        ))
    return result


def _way(data, strings):
    wid = _one(data,1)
    refs = _vals(data,8,True); s=0
    for i,v in enumerate(refs): s += v; refs[i]=s
    return wid, refs, _tags(_vals(data,2),_vals(data,3),strings)


def _primitive(data):
    strings=_strings(_bytes(data,1) or b"")
    granularity=_one(data,17,100); lat_off=_one(data,18,0); lon_off=_one(data,19,0)
    nodes=[]; ways=[]
    for f,w,group in _fields(data):
        if f != 2 or w != 2: continue
        for sf,sw,value in _fields(group):
            if sw != 2: continue
            if sf == 1: nodes.append(_node(value,strings,granularity,lat_off,lon_off))
            elif sf == 2: nodes.extend(_dense(value,strings,granularity,lat_off,lon_off))
            elif sf == 3: ways.append(_way(value,strings))
    return nodes,ways


def _blob(data):
    raw=None; zdata=None
    for f,w,v in _fields(data):
        if f == 1 and w == 2: raw=bytes(v)
        elif f == 3 and w == 2: zdata=bytes(v)
    if raw is not None: return raw
    if zdata is not None: return zlib.decompress(zdata)
    raise ValueError("PBF Blob не содержит raw или zlib_data")


def parse_osm_pbf(path: Path):
    nodes={}; node_tags={}; ways=[]
    with path.open("rb") as fp:
        while True:
            head=fp.read(4)
            if not head: break
            if len(head)!=4: raise ValueError("Оборванный PBF BlobHeader")
            hs=struct.unpack(">I",head)[0]
            if not 0 < hs <= 64*1024: raise ValueError("Некорректный размер PBF BlobHeader")
            header=fp.read(hs)
            if len(header)!=hs: raise ValueError("Оборванный PBF BlobHeader")
            typ=None; size=None
            for f,w,v in _fields(header):
                if f==1 and w==2: typ=bytes(v).decode("utf-8",errors="replace")
                elif f==3 and w==0: size=int(v)
            if size is None or not 0 <= size <= 256*1024*1024:
                raise ValueError(f"Некорректный размер PBF Blob: {size}")
            blob=fp.read(size)
            if len(blob)!=size: raise ValueError("Оборванный PBF Blob")
            if typ!="OSMData": continue
            bn,bw=_primitive(_blob(blob))
            for nid,lat,lon,tags in bn:
                nodes[nid]=(lat,lon)
                if tags: node_tags[nid]=tags
            ways.extend(bw)
    return nodes,node_tags,ways


def _area_kind(tags):
    leisure,landuse,natural=tags.get("leisure"),tags.get("landuse"),tags.get("natural")
    if leisure in {"park","recreation_ground"}: return "park"
    if natural in {"wood","forest"} or landuse=="forest": return "forest"
    if natural in {"water","riverbank"} or tags.get("water"): return "water"
    if leisure=="garden": return "garden"
    if landuse in {"grass","meadow"} or natural in {"grassland","meadow"}: return "green"
    if landuse=="industrial": return "industrial"
    if tags.get("building"): return "building"
    return None


def _geometry(node_map, refs):
    if len(refs)<2: return None
    out=[]
    for ref in refs:
        p=node_map.get(ref)
        if p is None: return None
        out.append([p[0],p[1]])
    return out


def _bbox(areas):
    for a in areas:
        g=a["geometry"]
        a["bbox"]=[min(x[0] for x in g),min(x[1] for x in g),max(x[0] for x in g),max(x[1] for x in g)]


def _inside(lat,lon,geom):
    inside=False;j=len(geom)-1
    for i in range(len(geom)):
        yi,xi=geom[i];yj,xj=geom[j]
        if ((yi>lat)!=(yj>lat)) and lon<(xj-xi)*(lat-yi)/((yj-yi) or 1e-12)+xi: inside=not inside
        j=i
    return inside


def _area_index(areas):
    idx={}
    for n,a in enumerate(areas):
        b=a["bbox"]
        y0,y1=math.floor((b[0]-.002)/GRID_LAT),math.floor((b[2]+.002)/GRID_LAT)
        x0,x1=math.floor((b[1]-.003)/GRID_LON),math.floor((b[3]+.003)/GRID_LON)
        for y in range(int(y0),int(y1)+1):
            for x in range(int(x0),int(x1)+1): idx.setdefault((y,x),[]).append(n)
    return idx


def _context(mid,areas,idx):
    lat,lon=mid;out={};seen=set()
    for y in range(int(math.floor((lat-.002)/GRID_LAT)),int(math.floor((lat+.002)/GRID_LAT))+1):
        for x in range(int(math.floor((lon-.003)/GRID_LON)),int(math.floor((lon+.003)/GRID_LON))+1):
            seen.update(idx.get((y,x),()))
    for n in seen:
        a=areas[n];b=a["bbox"]
        if not(b[0]-.002<=lat<=b[2]+.002 and b[1]-.003<=lon<=b[3]+.003): continue
        if not _inside(lat,lon,a["geometry"]): continue
        k=a["kind"]
        if k=="park": out["natural_score"]=max(out.get("natural_score",0),1)
        elif k=="forest": out["forest"]=True;out["natural_score"]=max(out.get("natural_score",0),.85)
        elif k=="green": out["natural_score"]=max(out.get("natural_score",0),.65)
        elif k=="garden": out["garden"]=True;out["natural_score"]=max(out.get("natural_score",0),.7)
        elif k=="water": out["water_near"]=True;out["waterfront"]=True
        elif k=="industrial": out["industrial"]=True
    return out


def build_or_load_graph(graph_path: Path, map_path: Path, pbf_path: Path):
    if graph_path.exists() and map_path.exists():
        try:
            graph=json.loads(graph_path.read_text(encoding="utf-8"))
            map_data=json.loads(map_path.read_text(encoding="utf-8"))
            if graph.get("meta",{}).get("schema") == GRAPH_SCHEMA:
                return graph,map_data
        except (OSError,ValueError,json.JSONDecodeError):
            pass

    if not pbf_path.exists():
        raise FileNotFoundError(f"Не найден локальный OSM PBF: data/{PBF_FILENAME}")

    weights=load_weights()
    dem, dem_name = open_dem(pbf_path.parent)
    node_map,node_tags,ways=parse_osm_pbf(pbf_path)
    roads=[];area_ways=[];buildings=[]
    for wid,refs,tags in ways:
        if is_walkable(tags): roads.append((wid,refs,tags))
        k=_area_kind(tags)
        if k and k!="building": area_ways.append((wid,refs,k))
        if tags.get("building"): buildings.append((wid,refs,tags.get("building","yes")))

    used_nodes=set()
    for _,refs,_ in roads:
        used_nodes.update(refs)

    nodes={}
    for nid in used_nodes:
        if nid not in node_map:
            continue
        lat, lon = node_map[nid]
        ele = None
        raw_ele = node_tags.get(nid, {}).get("ele")
        if raw_ele is not None:
            ele = num(str(raw_ele).replace(",", "."))
        if ele is None and dem is not None:
            try:
                ele = dem.sample(lat, lon)
            except Exception:
                ele = None
        nodes[str(nid)] = [lat, lon, ele] if ele is not None else [lat, lon]
    map_roads=[]
    for _,refs,tags in roads:
        g=_geometry(node_map,refs)
        if g: map_roads.append({"class":tags.get("highway",""),"coords":g})

    areas=[]
    for _,refs,kind in area_ways:
        g=_geometry(node_map,refs)
        if g and len(g)>=3 and haversine(tuple(g[0]),tuple(g[-1]))<=5:
            areas.append({"kind":kind,"geometry":g})
    building_data=[]
    for _,refs,kind in buildings:
        g=_geometry(node_map,refs)
        if g and len(g)>=3 and haversine(tuple(g[0]),tuple(g[-1]))<=10:
            building_data.append({"kind":kind,"coords":g})
    _bbox(areas);aidx=_area_index(areas)

    map_points=[]
    for nid,tags in node_tags.items():
        if nid not in node_map: continue
        lat,lon=node_map[nid]
        if not(LIPETSK_BBOX[0]<=lat<=LIPETSK_BBOX[2] and LIPETSK_BBOX[1]<=lon<=LIPETSK_BBOX[3]): continue
        typ=None
        if tags.get("highway")=="crossing": typ="crossing"
        elif tags.get("highway")=="traffic_signals": typ="traffic_signals"
        elif tags.get("railway")=="level_crossing": typ="crossing"
        elif tags.get("highway")=="bus_stop" or tags.get("public_transport")=="platform": typ="bus_stop"
        elif tags.get("amenity") in {"school","hospital","pharmacy","fuel","cafe","restaurant","bank","parking"}: typ=tags["amenity"]
        elif tags.get("entrance"): typ="entrance"
        if typ: map_points.append({"type":typ,"lat":lat,"lon":lon})

    adj={nid:[] for nid in nodes}
    edge_count=0
    for _,refs,tags in roads:
        if len(refs)<2: continue
        for a,b in zip(refs,refs[1:]):
            sa,sb=str(a),str(b)
            if sa not in nodes or sb not in nodes: continue
            dist=haversine((nodes[sa][0],nodes[sa][1]),(nodes[sb][0],nodes[sb][1]))
            if dist<1: continue
            mid=((nodes[sa][0]+nodes[sb][0])/2,(nodes[sa][1]+nodes[sb][1])/2)
            crit=rate(tags,weights,_context(mid,areas,aidx))
            contrib={k:v*(dist/1000.0) for k,v in crit.items()}
            score=sum(contrib.values())
            elevation_a = nodes[sa][2] if len(nodes[sa]) >= 3 and nodes[sa][2] is not None else None
            elevation_b = nodes[sb][2] if len(nodes[sb]) >= 3 and nodes[sb][2] is not None else None
            elevation_delta = (elevation_b - elevation_a) if elevation_a is not None and elevation_b is not None else None

            if elevation_delta is not None:
                dem_slope = abs(elevation_delta) / max(dist, 1.0) * 100.0
                if dem_slope >= 8.0 and "very_steep" not in contrib:
                    contrib["very_steep"] = weights["very_steep"] * (dist / 1000.0)
                elif dem_slope >= 4.0 and "steep" not in contrib:
                    contrib["steep"] = weights["steep"] * (dist / 1000.0)

            incline = num(str(tags.get("incline", "")).replace("%", "").replace(",", "."))
            score=sum(contrib.values())
            edge_meta = {
                "elevation_grade_pct": round(
                    (elevation_delta / max(dist, 1.0)) * 100.0, 2
                ) if elevation_delta is not None else None,
                "highway": tags.get("highway", ""),
                "stairs": tags.get("highway") == "steps",
                "elevation_delta_m": round(elevation_delta, 2) if elevation_delta is not None else None,
                "incline_pct": round(incline, 2) if incline else None,
            }
            reverse_meta = {
                "highway": tags.get("highway", ""),
                "stairs": tags.get("highway") == "steps",
                "elevation_delta_m": round(-elevation_delta, 2) if elevation_delta is not None else None,
                "incline_pct": round(-incline, 2) if incline else None,
            }
            edge={"to":sb,"dist":dist,"score":score,"criteria":contrib,**edge_meta}
            reverse={"to":sa,"dist":dist,"score":score,"criteria":contrib,**reverse_meta}
            # Для пешеходного маршрута motor-vehicle oneway не является запретом
            # на обратное движение, поэтому граф строим двунаправленным.
            adj[sa].append(edge);adj[sb].append(reverse);edge_count+=2

    graph={"nodes":nodes,"adj":adj,"meta":{
        "schema":GRAPH_SCHEMA,"bbox":list(LIPETSK_BBOX),"nodes":len(nodes),"edges":edge_count,
        "source":"OpenStreetMap Protocolbuffer PBF (local file)","pbf":pbf_path.name,
        "elevation":"Copernicus DEM" if dem is not None else "OSM ele tags only",
        "dem":dem_name if dem is not None else None,
        "offline":True}}
    map_data={"bbox":list(LIPETSK_BBOX),"source":pbf_path.name,"roads":map_roads,
              "areas":[{"kind":a["kind"],"coords":a["geometry"]} for a in areas],
              "buildings":building_data,"points":map_points}
    graph_path.parent.mkdir(parents=True,exist_ok=True)
    graph_path.write_text(json.dumps(graph,ensure_ascii=False,separators=(",",":")),encoding="utf-8")
    map_path.write_text(json.dumps(map_data,ensure_ascii=False,separators=(",",":")),encoding="utf-8")
    return graph,map_data
