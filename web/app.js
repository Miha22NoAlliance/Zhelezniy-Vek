const canvas = document.getElementById('map');
const ctx = canvas.getContext('2d');

let start=null, goal=null, routeCoordinates=null, graphMeta=null, mapData=null;
let view={lat:52.58885,lon:39.58745,pixelsPerMeter:.8};
let dragging=false,moved=false,downX=0,downY=0,panStart=null;
const $=id=>document.getElementById(id);

const labels={
park:'Парк / сквер',forest:'Лес / природная зона',waterfront:'Набережная / берег',water_near:'Рядом с водой',
green_open:'Луга / зелёная зона',tree_line:'Деревья',garden:'Сад',natural_green:'Естественная зелень',
very_quiet:'Очень тихая зона',quiet_residential:'Тихая жилая улица',pedestrian_street:'Пешеходная улица',
no_motor_traffic:'Нет автотрафика',away_major_road:'Удалённость от крупной дороги',pleasant_density:'Приятная застройка',
major_road:'Крупная магистраль',heavy_traffic:'Интенсивный трафик',industrial_zone:'Промышленная зона',
noisy_zone:'Шумная зона',road_near:'Автодорога рядом',railway_near:'Железная дорога рядом',
excellent_surface:'Отличное покрытие',good_surface:'Хорошее покрытие',wide_path:'Широкая дорожка',
dedicated_path:'Пешеходная дорожка',good_sidewalk:'Хороший тротуар',rest_nearby:'Места отдыха',
service_nearby:'Сервис рядом',car_protected:'Защищённость от машин',bad_surface:'Плохое покрытие',
very_bad_surface:'Очень плохое покрытие',narrow_path:'Узкая дорожка',no_sidewalk:'Нет тротуара',
safe_crossing:'Безопасный переход',signal_crossing:'Светофорный переход',dangerous_crossing:'Опасный переход',
major_crossing:'Пересечение крупной дороги',stairs:'Лестницы',steep:'Сильный подъём',
very_steep:'Очень сильный подъём',restricted:'Ограниченный доступ'};

const roadWidth={trunk:3.6,primary:3.1,secondary:2.6,tertiary:2.2,residential:1.55,living_street:1.6,
service:1.05,pedestrian:1.6,footway:1.25,path:1.1,cycleway:1.2,track:1,steps:1};
const roadStroke={primary:'#b9bdc3',secondary:'#a4a8ae',tertiary:'#92979e',residential:'#777c84',
living_street:'#7d8288',service:'#656a71',pedestrian:'#aeb2b8',footway:'#777c84',path:'#666b72',
cycleway:'#777c84',track:'#5b6066',steps:'#70757c'};

function resize(){
  const dpr=window.devicePixelRatio||1;
  canvas.width=Math.max(1,Math.floor(canvas.clientWidth*dpr));
  canvas.height=Math.max(1,Math.floor(canvas.clientHeight*dpr));
  ctx.setTransform(dpr,0,0,dpr,0,0);draw();
}
function metrics(){
  return{w:canvas.clientWidth,h:canvas.clientHeight,
    metersPerLon:111320*Math.cos(view.lat*Math.PI/180),metersPerLat:111320};
}
function project(lat,lon){
  const m=metrics();
  return{x:m.w/2+(lon-view.lon)*m.metersPerLon*view.pixelsPerMeter,
    y:m.h/2-(lat-view.lat)*m.metersPerLat*view.pixelsPerMeter};
}
function unproject(x,y){
  const m=metrics();
  return[view.lat-(y-m.h/2)/(m.metersPerLat*view.pixelsPerMeter),
    view.lon+(x-m.w/2)/(m.metersPerLon*view.pixelsPerMeter)];
}
function fitBounds(bbox,padding=28){
  const [south,west,north,east]=bbox,midLat=(south+north)/2,midLon=(west+east)/2;
  const w=canvas.clientWidth,h=canvas.clientHeight;
  const widthM=Math.max(1,(east-west)*111320*Math.cos(midLat*Math.PI/180));
  const heightM=Math.max(1,(north-south)*111320);
  view.lat=midLat;view.lon=midLon;
  view.pixelsPerMeter=Math.max(.03,Math.min(10,Math.min((w-padding*2)/widthM,(h-padding*2)/heightM)));
  draw();
}
function drawPolygon(coords){
  if(!coords?.length)return;
  let p=project(coords[0][0],coords[0][1]);ctx.beginPath();ctx.moveTo(p.x,p.y);
  for(let i=1;i<coords.length;i++){p=project(coords[i][0],coords[i][1]);ctx.lineTo(p.x,p.y);}
  ctx.closePath();ctx.fill();ctx.stroke();
}
function drawRoad(coords,cls){
  if(!coords?.length)return;
  let p=project(coords[0][0],coords[0][1]);ctx.beginPath();ctx.moveTo(p.x,p.y);
  for(let i=1;i<coords.length;i++){p=project(coords[i][0],coords[i][1]);ctx.lineTo(p.x,p.y);}
  ctx.strokeStyle=roadStroke[cls]||'#696e75';ctx.lineWidth=roadWidth[cls]||1;
  ctx.lineCap='round';ctx.lineJoin='round';ctx.stroke();
}
function drawPointFeature(item){
  const p=project(item.lat,item.lon);
  const type=item.type;
  if(p.x<-30||p.y<-30||p.x>canvas.clientWidth+30||p.y>canvas.clientHeight+30)return;

  ctx.save();
  if(type==='crossing'){
    ctx.fillStyle='#dce3ea';ctx.strokeStyle='#1b1f24';ctx.lineWidth=1;
    ctx.fillRect(p.x-6,p.y-6,12,12);ctx.strokeRect(p.x-6,p.y-6,12,12);
    ctx.strokeStyle='#555d66';ctx.lineWidth=2;
    for(let i=-4;i<=4;i+=4){ctx.beginPath();ctx.moveTo(p.x+i-2,p.y-5);ctx.lineTo(p.x+i+2,p.y+5);ctx.stroke();}
  }else if(type==='traffic_signals'){
    ctx.fillStyle='#202328';ctx.strokeStyle='#111';ctx.lineWidth=1.5;
    ctx.beginPath();ctx.roundRect(p.x-4,p.y-8,8,16,3);ctx.fill();ctx.stroke();
    ctx.fillStyle='#d55';ctx.beginPath();ctx.arc(p.x,p.y-4,1.8,0,Math.PI*2);ctx.fill();
    ctx.fillStyle='#d8c34b';ctx.beginPath();ctx.arc(p.x,p.y,1.8,0,Math.PI*2);ctx.fill();
    ctx.fillStyle='#5cb36b';ctx.beginPath();ctx.arc(p.x,p.y+4,1.8,0,Math.PI*2);ctx.fill();
  }else{
    const styles={
      bus_stop:['#4e84c4','B'],school:['#c58a42','S'],hospital:['#b95b62','H'],
      pharmacy:['#65a56e','+'],fuel:['#b56f49','F'],cafe:['#a57e55','C'],
      restaurant:['#b96f56','R'],bank:['#8a7eb5','₽'],parking:['#6d8ea8','P'],entrance:['#8c8f94','E']
    };
    const st=styles[type]||['#888','•'];
    ctx.fillStyle=st[0];ctx.strokeStyle='#16191d';ctx.lineWidth=1.5;
    ctx.beginPath();ctx.arc(p.x,p.y,6,0,Math.PI*2);ctx.fill();ctx.stroke();
    ctx.fillStyle='#fff';ctx.font='bold 8px Segoe UI,Arial,sans-serif';
    ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(st[1],p.x,p.y+.4);
  }
  ctx.restore();
}

function drawMarker(lat,lon,fill,letter){
  const p=project(lat,lon);ctx.beginPath();ctx.arc(p.x,p.y,8,0,Math.PI*2);
  ctx.fillStyle=fill;ctx.fill();ctx.lineWidth=2;ctx.strokeStyle='#15171a';ctx.stroke();
  ctx.fillStyle='#111';ctx.font='bold 9px Segoe UI,Arial,sans-serif';
  ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(letter,p.x,p.y+.5);
  ctx.textAlign='left';ctx.textBaseline='alphabetic';
}
function draw(){
  const w=canvas.clientWidth,h=canvas.clientHeight;
  ctx.clearRect(0,0,w,h);ctx.fillStyle='#111418';ctx.fillRect(0,0,w,h);
  if(!mapData){ctx.fillStyle='#8d939b';ctx.font='14px Segoe UI,Arial,sans-serif';
    ctx.fillText('Загрузка локальной карты…',24,30);return;}
  for(const area of mapData.areas||[]){
    const s={water:['#1b3442','#2e5366'],forest:['#1c2c24','#304738'],park:['#1f3026','#415645'],
      green:['#202d23','#394b3d'],garden:['#263329','#435848'],industrial:['#2b2925','#504b44']}[area.kind]||['#202020','#333'];
    ctx.fillStyle=s[0];ctx.strokeStyle=s[1];ctx.lineWidth=.8;drawPolygon(area.coords);
  }

  if(view.pixelsPerMeter > .08){
    ctx.fillStyle='#25272a';ctx.strokeStyle='#3b3e43';ctx.lineWidth=.45;
    for(const building of mapData.buildings||[])drawPolygon(building.coords);
  }

  const order={path:1,track:1,footway:1,cycleway:1,steps:1,service:2,residential:3,living_street:3,
    pedestrian:3,tertiary:4,secondary:5,primary:6,trunk:7};
  for(const road of [...(mapData.roads||[])].sort((a,b)=>(order[a.class]||0)-(order[b.class]||0)))drawRoad(road.coords,road.class);
  if(routeCoordinates?.length){
    let p=project(routeCoordinates[0][0],routeCoordinates[0][1]);ctx.beginPath();ctx.moveTo(p.x,p.y);
    for(let i=1;i<routeCoordinates.length;i++){p=project(routeCoordinates[i][0],routeCoordinates[i][1]);ctx.lineTo(p.x,p.y);}
    ctx.strokeStyle='#17191d';ctx.lineWidth=8;ctx.lineCap='round';ctx.lineJoin='round';ctx.stroke();
    ctx.strokeStyle='#ffad1f';ctx.lineWidth=5;ctx.stroke();
  }
  if(view.pixelsPerMeter > .22){
    for(const item of mapData.points||[])drawPointFeature(item);
  }

  if(start)drawMarker(start[0],start[1],'#f5f5f5','С');
  if(goal)drawMarker(goal[0],goal[1],'#9da3ab','Ф');
  ctx.fillStyle='rgba(20,22,25,.85)';ctx.fillRect(12,12,190,30);
  ctx.fillStyle='#aaaeb4';ctx.font='12px Segoe UI,Arial,sans-serif';ctx.fillText('Локальная OSM-карта · offline',22,31);
}
function setPoint(lat,lon){
  if(!start)start=[lat,lon];
  else if(!goal)goal=[lat,lon];
  else{start=[lat,lon];goal=null;routeCoordinates=null;$('result').classList.add('hidden');}
  $('build').disabled=!(start&&goal)||!mapData;
  $('status').textContent=start&&goal?'Готово к построению маршрута':'Теперь выберите финиш';draw();
}
function clearAll(){
  start=goal=null;routeCoordinates=null;$('result').classList.add('hidden');$('build').disabled=true;
  $('status').textContent=mapData?'Выберите старт и финиш на карте':'Загрузка локальной карты…';draw();
}
function pointerLocal(e){const r=canvas.getBoundingClientRect();return[e.clientX-r.left,e.clientY-r.top];}

canvas.addEventListener('pointerdown',e=>{
  dragging=true;moved=false;[downX,downY]=pointerLocal(e);panStart={lat:view.lat,lon:view.lon};
  canvas.setPointerCapture(e.pointerId);
});
canvas.addEventListener('pointermove',e=>{
  if(!dragging)return;const[x,y]=pointerLocal(e);
  if(Math.hypot(x-downX,y-downY)>4)moved=true;if(!moved)return;
  const m=metrics();view.lat=panStart.lat+(y-downY)/(m.metersPerLat*view.pixelsPerMeter);
  view.lon=panStart.lon-(x-downX)/(m.metersPerLon*view.pixelsPerMeter);draw();
});
canvas.addEventListener('pointerup',e=>{
  if(!dragging)return;dragging=false;const[x,y]=pointerLocal(e);
  if(!moved&&mapData){const[lat,lon]=unproject(x,y),b=mapData.bbox;
    if(lat>=b[0]&&lat<=b[2]&&lon>=b[1]&&lon<=b[3])setPoint(lat,lon);}
});
canvas.addEventListener('wheel',e=>{
  e.preventDefault();if(!mapData)return;const[x,y]=pointerLocal(e),before=unproject(x,y);
  view.pixelsPerMeter=Math.max(.03,Math.min(12,view.pixelsPerMeter*Math.exp(-e.deltaY*.0015)));
  const after=unproject(x,y);view.lat+=before[0]-after[0];view.lon+=before[1]-after[1];draw();
},{passive:false});

$('detour').addEventListener('input',e=>$('detourValue').textContent=e.target.value+'%');
$('routeMode').addEventListener('change',e=>{
  const simple=e.target.value==='simple';
  $('status').textContent=simple
    ? 'Упрощённый режим: прямее, крупнее улицы, меньше лестниц и подъёмов'
    : 'Качественный режим: ищет более приятный путь';
});
$('clear').onclick=clearAll;$('build').onclick=buildRoute;window.addEventListener('resize',resize);

async function buildRoute(){
  if(!start||!goal||!mapData)return;
  $('build').disabled=true;$('status').textContent='Ищу лучший маршрут по локальному графу…';
  try{
    const params=new URLSearchParams({
      slat:start[0],slon:start[1],glat:goal[0],glon:goal[1],
      detour:$('detour').value/100,mode:$('routeMode').value
    });
    const r=await fetch('/api/route?'+params,{cache:'no-store'}),data=await r.json();
    if(!r.ok)throw new Error(data.error||'Маршрут не найден');
    routeCoordinates=data.coordinates;draw();
    const lats=data.coordinates.map(p=>p[0]),lons=data.coordinates.map(p=>p[1]);
    fitBounds([Math.min(...lats),Math.min(...lons),Math.max(...lats),Math.max(...lons)],70);
    $('distance').textContent=(data.distance_m/1000).toFixed(2)+' км';
    $('score').textContent=data.score.toFixed(1);$('scoreKm').textContent=data.score_per_km.toFixed(1);
    $('limit').textContent=(data.max_distance_m/1000).toFixed(2)+' км';
    $('elevation').textContent=data.ascent_m.toFixed(0)+' м / '+data.descent_m.toFixed(0)+' м';
    $('stairs').textContent=data.stairs_count.toLocaleString('ru-RU');
    $('criteria').innerHTML=Object.entries(data.criteria||{}).map(([key,value])=>
      '<div class="crit"><span class="name">'+(labels[key]||key)+'</span><span class="'+(value<0?'minus':'plus')+'">'+
      (value>=0?'+':'')+value.toFixed(1)+'</span></div>').join('');
    $('result').classList.remove('hidden');
    $('status').textContent=(data.mode==='simple'?'Упрощённый маршрут':'Качественный маршрут')+
      ' построен · обработано узлов: '+data.expanded_labels.toLocaleString('ru-RU');
  }catch(e){$('status').textContent='Ошибка: '+e.message;}
  finally{$('build').disabled=!(start&&goal&&mapData);}
}

async function init(){
  resize();$('build').disabled=true;$('status').textContent='Загружаю локальный OSM PBF…';
  try{
    const r=await fetch('/api/map',{cache:'no-store'}),data=await r.json();
    if(!r.ok)throw new Error(data.error||'Не удалось загрузить локальные данные');
    mapData=data;fitBounds(mapData.bbox,28);
    const mr=await fetch('/api/graph',{cache:'no-store'});graphMeta=await mr.json();
    $('status').textContent='Локальная карта готова · дорог: '+mapData.roads.length.toLocaleString('ru-RU')+' · зданий: '+(mapData.buildings||[]).length.toLocaleString('ru-RU');
  }catch(e){$('status').textContent='Ошибка: '+e.message;}
}
init();
