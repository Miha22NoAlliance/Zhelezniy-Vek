# WalkRoute Demo

Компьютерная демка собственного алгоритма пешеходной маршрутизации. Тестовая территория — Липецк.

## Данные карты

Программа работает с локальной выгрузкой OpenStreetMap в формате Protocolbuffer (PBF).

Нужный файл:

`data/planet_39.4596,52.532_39.7153,52.6457.osm.pbf`

Параметры выгрузки:

- LIPETSK
- 218 км²
- `39.4596,52.5320 x 39.7153,52.6457`
- Protocolbuffer (PBF)

**PBF нужно один раз положить в папку `data` внутри репозитория.**

После этого приложение не скачивает данные карты из интернета.

При первом запуске из PBF локально строятся:

`data/lipetsk_graph.json` — граф для маршрутизации  
`data/lipetsk_map.json` — данные для отрисовки карты

Эти два JSON являются производными файлами и не нужны в GitHub: они автоматически создаются на компьютере пользователя.

## Запуск

Нужен Python 3.10+.

Запустите `run.bat`.

Сервер запускается на:

`http://127.0.0.1:8765`

Первое обращение к карте читает локальный PBF и строит граф. Последующие запуски используют готовые локальные JSON.

Веб-интерфейс не использует Leaflet CDN, внешние JS/CSS или онлайн-тайлы. Карта рисуется локально на Canvas.

На локальном слое отображаются здания, пешеходные переходы, светофоры и базовые POI, если они присутствуют в PBF.

## Главное

Демка показывает **только наш маршрут**. Сравнение с Яндекс.Картами и Google Maps делается вручную в отдельных сервисах.

Алгоритм не минимизирует длину как критерий качества. Для каждого участка графа считаются баллы по 40 критериям в баллах/км. Длина участка только масштабирует его вклад. Для защиты от бесконечного удлинения маршрута длина используется отдельно как жёсткое ограничение максимального объезда.

Поиск — многокритериальный label-setting: для узлов сохраняются недоминируемые состояния «длина + накопленный score». Среди состояний, укладывающихся в лимит объезда, выбирается маршрут с максимальным score.

## Настройка

Все 40 весов находятся в `config/weights.json`. Меняя их, можно экспериментировать с поведением алгоритма без переписывания маршрутизатора.

Порог максимального объезда меняется в интерфейсе от 100% до 180% от кратчайшего доступного пешеходного пути.

## Источник данных

OpenStreetMap contributors. Данные распространяются на условиях ODbL.

Программа читает предоставленный OSM PBF локально; сетевой запрос к Overpass для работы демки больше не нужен.


## Маршрутизация

Доступны режимы «Качественный маршрут» и «Упрощённый маршрут — прямее». Автоподбор объезда может прогнать бюджеты 110%, 120%, …, 180% и выбрать фактический маршрут по максимальному показателю «Баллы/км».

Для учёта рельефа положите Copernicus DEM GeoTIFF в `data/`. Поддерживается файл `Copernicus_DSM_COG_10_N52_00_E039_00_DEM.tif`; также сервер автоматически подхватывает первый `.tif/.tiff` в `data/`. Интернет во время работы не требуется.


## Simple mode v2

Simple v2 is a separate city-oriented routing mode. Its cost model considers route continuity, road hierarchy, turn severity, directness, crossings, traffic signals, major crossings, OSM access/barrier tags, elevation and stairs. The graph stores OSM way/name/ref metadata so a route can prefer staying on the same street instead of taking a short service/path detour.

The shortest-distance baseline uses A* with the straight-line haversine distance as an admissible heuristic. Simple v2 uses a state-aware A* search whose state contains the previous and current graph nodes, allowing turn penalties without changing the public routing API.

The result includes a heuristic walking-time estimate based on route length plus penalties for turns, crossings, signals, major crossings, stairs and ascent. It is an estimate, not live navigation ETA.

Changing the graph schema forces local derived JSON to be rebuilt from the local PBF. Use /api/reload or remove data/lipetsk_graph.json and data/lipetsk_map.json.
