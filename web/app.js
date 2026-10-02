const map = L.map('map').setView([52.6108, 39.594], 12);
L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
  maxZoom: 19,
  attribution: '&copy; OpenStreetMap contributors'
}).addTo(map);

let start = null, goal = null, startMarker = null, goalMarker = null, routeLine = null, graphMeta = null;
const $ = id => document.getElementById(id);

const labels = {
  park:'Парк / сквер', forest:'Лес / природная зона', waterfront:'Набережная / берег', water_near:'Рядом с водой',
  green_open:'Луга / зелёная зона', tree_line:'Деревья', garden:'Сад', natural_green:'Естественная зелень',
  very_quiet:'Очень тихая зона', quiet_residential:'Тихая жилая улица', pedestrian_street:'Пешеходная улица', no_motor_traffic:'Нет автотрафика',
  away_major_road:'Удалённость от крупной дороги', pleasant_density:'Приятная застройка', major_road:'Крупная магистраль', heavy_traffic:'Интенсивный трафик',
  industrial_zone:'Промышленная зона', noisy_zone:'Шумная зона', road_near:'Автодорога рядом', railway_near:'Железная дорога рядом',
  excellent_surface:'Отличное покрытие', good_surface:'Хорошее покрытие', wide_path:'Широкая дорожка', dedicated_path:'Пешеходная дорожка',
  good_sidewalk:'Хороший тротуар', rest_nearby:'Места отдыха', service_nearby:'Сервис рядом', car_protected:'Защищённость от машин',
  bad_surface:'Плохое покрытие', very_bad_surface:'Очень плохое покрытие', narrow_path:'Узкая дорожка', no_sidewalk:'Нет тротуара',
  safe_crossing:'Безопасный переход', signal_crossing:'Светофорный переход', dangerous_crossing:'Опасный переход', major_crossing:'Пересечение крупной дороги',
  stairs:'Лестницы', steep:'Сильный подъём', very_steep:'Очень сильный подъём', restricted:'Ограниченный доступ'
};

$('detour').addEventListener('input', e => $('detourValue').textContent = e.target.value + '%');

$('clear').onclick = () => {
  start = goal = null;
  startMarker?.remove(); goalMarker?.remove(); routeLine?.remove();
  startMarker = goalMarker = routeLine = null;
  $('result').classList.add('hidden');
  $('status').textContent = 'Выберите старт и финиш на карте';
};

$('build').onclick = buildRoute;

map.on('click', e => {
  if (!start) {
    start = [e.latlng.lat, e.latlng.lng];
    startMarker = L.marker(start).addTo(map).bindTooltip('Старт', {permanent:true, className:'marker-label', offset:[0,-18]});
  } else if (!goal) {
    goal = [e.latlng.lat, e.latlng.lng];
    goalMarker = L.marker(goal).addTo(map).bindTooltip('Финиш', {permanent:true, className:'marker-label', offset:[0,-18]});
  } else {
    start = [e.latlng.lat, e.latlng.lng];
    goal = null;
    startMarker?.remove(); goalMarker?.remove(); routeLine?.remove();
    startMarker = L.marker(start).addTo(map).bindTooltip('Старт', {permanent:true, className:'marker-label', offset:[0,-18]});
    goalMarker = routeLine = null;
  }
  $('build').disabled = !(start && goal && graphMeta);
  $('status').textContent = start && goal ? 'Готово к построению маршрута' : 'Теперь выберите финиш';
});

async function init() {
  try {
    const r = await fetch('/api/graph');
    const data = await r.json();
    if (!r.ok) throw new Error(data.error || 'Ошибка загрузки');
    graphMeta = data;
    $('status').textContent = `Граф загружен: ${data.nodes.toLocaleString('ru-RU')} узлов, ${data.edges.toLocaleString('ru-RU')} рёбер`;
    $('build').disabled = false;
  } catch (e) {
    $('status').textContent = 'Ошибка: ' + e.message;
  }
}

async function buildRoute() {
  if (!start || !goal) return;
  $('build').disabled = true;
  $('status').textContent = 'Ищу лучший маршрут по модели прогулки…';

  try {
    const params = new URLSearchParams({
      slat:start[0], slon:start[1], glat:goal[0], glon:goal[1],
      detour:$('detour').value / 100
    });
    const r = await fetch('/api/route?' + params.toString());
    const data = await r.json();
    if (!r.ok) throw new Error(data.error || 'Маршрут не найден');

    routeLine?.remove();
    routeLine = L.polyline(data.coordinates, {weight:6, opacity:.9}).addTo(map);
    map.fitBounds(routeLine.getBounds(), {padding:[60,60]});

    $('distance').textContent = (data.distance_m / 1000).toFixed(2) + ' км';
    $('score').textContent = data.score.toFixed(1);
    $('scoreKm').textContent = data.score_per_km.toFixed(1);
    $('limit').textContent = (data.max_distance_m / 1000).toFixed(2) + ' км';

    $('criteria').innerHTML = Object.entries(data.criteria).map(([key, value]) =>
      `<div class="crit"><span class="name">${labels[key] || key}</span><span class="${value < 0 ? 'minus':'plus'}">${value >= 0 ? '+' : ''}${value.toFixed(1)}</span></div>`
    ).join('');

    $('result').classList.remove('hidden');
    $('status').textContent = `Маршрут построен · обработано меток: ${data.expanded_labels.toLocaleString('ru-RU')}`;
  } catch (e) {
    $('status').textContent = 'Ошибка: ' + e.message;
  } finally {
    $('build').disabled = false;
  }
}

init();
