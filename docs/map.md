# Map

Five dashboard pages share one canvas map:
[`map.js`](../crates/sim-dashboard/assets/map.js). It draws the A5 occupancy
grid, the drones and what they see, geofences, stations, regions of interest
and the couriers' delivery range, optionally over map tiles. The server does
the expensive part (aggregating cells and building their outlines); the
browser projects, draws and turns clicks into cells. Each page adds a tool of
its own on top: the cell inspector on State, drawing on Geofences and Stations
& regions, wind on Weather, the heatmap on Conflicts.

## Where it appears

| Page | Map controls | A click does |
|---|---|---|
| State | Display resolution, Basemap, Live, Fit, Layers | Inspects the cell under the pointer |
| Geofences | Basemap, Live, Fit, Layers, drawing tools | Picks a fence, or draws one |
| Stations & regions | Basemap, Live, Fit, Layers, placing tools | Picks a station or region, or places one |
| Weather | Height, Now, Hour, Basemap, Live, Fit, Layers | Queries the wind there |
| Conflicts | Cells, Shade by, Basemap, Live, Fit, Layers | Picks a conflict cell |

Only State has a display resolution slider. The other pages fetch the grid at
the scenario's `grid.view_resolution`.

## Layers

Each layer has a checkbox under the map.

| Layer | Key | Shows | Source |
|---|---|---|---|
| Drones & aircraft | `drones` | Drones, intruders and manned aircraft | `/api/v1/state/entities?kind=drone,intruder,aircraft` |
| Hubs and landmarks | `pois` | The model's points of interest: depots and named places | `/api/v1/state/entities?kind=poi` |
| Delivery addresses | `addresses` | Every address in the pack's address book, coloured by the hub that serves it. Off until ticked | `/api/v1/state/addresses` |
| Orders and drop-offs | `deliveries` | Open orders' addresses (a ring while waiting, filled once a drone is on its way) and recent drop-offs | `/api/v1/state/overlays` (`orders`), `/api/v1/reports` |
| People and vehicles | `people` | Where people and vehicles on the ground really are | `/api/v1/state/entities?kind=person,vehicle` |
| Detections | `detections` | What the drones' cameras have seen lately | `/api/v1/reports` |
| Geofences | `fences` | Open geofences | `/api/v1/geofences` |
| Stations | `stations` | Launch and recovery stations | `/api/v1/state/overlays` (`stations`) |
| Regions of interest | `rois` | Areas for the drones to survey | `/api/v1/state/overlays` (`rois`) |
| Delivery range | `range` | Where couriers can deliver to and get back from, by way of stations | `/api/v1/state/overlays` (`range`) |
| A5 occupancy grid | `grid` | Agents per cell at the display resolution | `/api/v1/state/map` |

Weather and Conflicts add a layer of their own (Wind, Conflict heatmap).

**The choice is remembered.** Each change is stored in the browser
(`localStorage`, key `krabshell.layers`) and applies on every map page. A page
may start a layer off for itself without changing what the others remember:
Weather and Conflicts start with the occupancy grid off
(`data-layers-off="grid"` on the canvas). Tick it by hand there and that page
remembers it like any other. With storage blocked, every layer starts on and
changes apply until the page is left, with delivery addresses off.

A layer that is off is not fetched either, so turning off people and vehicles
on a large world saves bandwidth as well as clutter. Delivery addresses start
off because a city's book is large (Kaunas has 34 502, about 1.5 MB): the map
fetches it once per scenario when the layer is ticked, and after that only
asks whether the scenario has changed. A pack without an address book leaves
the layer empty. A model that publishes no
stations, orders or regions leaves those layers empty.

### Drawing order

From the bottom: basemap, occupancy cells, the inspected cell's outline,
delivery range, geofences, regions of interest, stations, delivery
addresses, people and vehicles, orders and drop-offs, detections, hubs and landmarks, drones, then
the page's own tool (a fence being drawn, the wind, the heatmap).

## Symbology

Drones, intruders, manned aircraft, sightings and stations are NATO joint military symbols
(APP-6 / MIL-STD-2525), drawn by
[milsymbol](https://github.com/spatialillusions/milsymbol) 3.0.4 (MIT). The
library is compiled into the binary and served at
`/assets/milsymbol-3.0.4.js`, so the dashboard needs no internet for symbols.

Drones, intruders and manned aircraft use the 2525 letter SIDCs of the CoT
types krabshell sends TAK for them (a manned aircraft is `SNAPCF`, CoT
`a-n-A-C-F`, neutral civilian fixed-wing), so the dashboard and TAK show the
same symbol. The rest
use APP-6 D / MIL-STD-2525D 20-digit SIDCs, which have the civilian
individuals and vehicles the letter codes lack.

| What | Symbol | SIDC |
|---|---|---|
| Delivery drone | Friendly air, military rotary-wing UAV (CoT `a-f-A-M-H-Q`) | `SFAPMHQ--------` |
| Intruder, and a camera sighting of one | Unknown air, military rotary-wing UAV (CoT `a-u-A-M-H-Q`) | `SUAPMHQ--------` |
| Sighting of a person | Unknown land civilian unit, individual | `10011100001103000000` |
| Sighting of a car | Unknown land equipment, civilian automobile | `10011500001601000000` |
| Sighting of a pickup | Unknown, open-bed truck | `10011500001602000000` |
| Sighting of a bus | Unknown, multiple passenger vehicle | `10011500001603000000` |
| Sighting of a van | Unknown, utility vehicle | `10011500001604000000` |
| Sighting of a lorry | Unknown, tractor-trailer truck with box | `10011500001606000000` |
| Launch and recovery station | Friendly ground installation, air base | `SFGPIBA---H----` |

A vehicle sighting is drawn by the `vehicle_type` it carries; an unknown type
is drawn as a car. Civilians keep the unknown identity's yellow rather than
APP-6's optional civilian purple, since who they are is not known.

Symbols stay upright, as the standard has them. A moving drone (over
0.3 m/s) gets APP-6's direction-of-movement arrow along its heading. Symbols
grow in three steps as you zoom in: 17, 21 and 26 pixels. Sightings are drawn
a little smaller than drones. Every symbol and arrow has a pale halo so it
reads over a dark theme or a basemap.

The rest of the map is not military symbology:

| What | Drawn as |
|---|---|
| Region of interest | A dashed circle labelled `NAI` and its name, as an APP-6 named area of interest. Heavier and blue, with "N drones tasked to survey", while drones are on their way; red when a crash put it there. |
| Hub or landmark | A small diamond, named once zoomed in to street scale. |
| Order waiting | A green ring. |
| Order on its way | A filled green circle. |
| Parcel just delivered | A green tick, fading over two minutes. |
| Person, vehicle (the truth) | A faint dot (person) or square (vehicle). |
| Geofence | Red hatching and outline, labelled with its number and altitude band once big enough on screen (`#4 · 60 m–UNL`, `#4 · SFC–120 m`; SFC is the surface, UNL unlimited). A fence with a raised floor is lighter and dashed: drones may pass beneath. One picked out in the list is outlined in blue. |
| Delivery range | A light green fill with a dashed green outline round the union of every hub's reach. |
| Occupancy cell | Shaded from blue through green, yellow and orange to red by agent count, on a log scale so one crowded cell does not flatten the rest. The scale is at the bottom right. |

Sightings fade over 30 seconds. When the map opens it only learns where the
report stream is, so it does not replay old sightings at once.

Labels are kept off a crowded map. Everything in the air (drones moving or
hovering, intruders, manned aircraft) is always named; a drone's label says
what it is doing (`MEGA-3 · delivering`) or, failing that, its flight mode.
Labels of anything in the air end with its flight level: altitude above mean
sea level in thousands of feet, rounded down, so `RELAY-4 · flying · FL3` is
between 3 000 and 3 999 ft, FL1 is 1 000 ft and FL23 23 100 ft. Parked drones
are named once zoomed in to street scale.

A manned aircraft is drawn with a ring of 1 000 ft (304.8 m) radius round it.
The ring is grey and dashed, and turns solid red, filled, while a drone or
intruder is inside it and within 150 m above or below: a loss of separation
you can see. The ring's size and the 150 m are fixed in the map; the conflict
service's own minima are the pack's `manned_lateral_m` and `manned_vertical_m`
(300 m and 150 m by default), see [Conflicts](conflicts.md). Stations are named, with slots in use (`Roof 2 · 1/3 slots`),
once zoomed in a little; regions and fence numbers likewise.

If milsymbol fails to load, the map falls back to plain shapes: arrows for
drones (blue) and intruders (red), squares for stations, dots for sightings.
The map key beside each map draws the same symbols with the same code, so the
key always matches the map.

## Basemaps

**Basemap** lays XYZ map tiles under the cells. It is off ("None") by
default, so an air-gapped console never waits on tiles.

| Name | Provider | Max zoom |
|---|---|---|
| OpenStreetMap | `tile.openstreetmap.org` | 19 |
| Light gray (Esri) | ArcGIS World Light Gray Base | 16 |
| Dark gray (Esri) | ArcGIS World Dark Gray Base | 16 |
| Satellite (Esri) | ArcGIS World Imagery | 19 |

Tiles are fetched by the operator's browser straight from the provider, not
by the server. They need internet access from the browser and fall under each
provider's usage policy. Only keyless providers are listed. The provider's
attribution is shown in the corner of the map; if no tile loads, it adds that
the browser needs internet access. While a tile loads, the nearest loaded
ancestor is stretched over its square, so zooming does not flash blank. The
choice is remembered in the browser (`krabshell.basemap`) across pages.

The list is `BASEMAPS` in
[`pages.rs`](../crates/sim-axum/src/routes/pages.rs). A new entry needs an
`{z}`/`{x}`/`{y}` URL template, a maximum zoom and its attribution.

The map uses Web Mercator, as every XYZ tile server does, so cells and tiles
line up. It does not wrap the antimeridian and clips at 85° north and south.

## Interaction

| Action | Effect |
|---|---|
| Drag | Pan |
| Scroll | Zoom about the pointer |
| Click | The page's action (see [Where it appears](#where-it-appears)) |
| Hover (State) | Outlines the cell and shows its id and agent count |
| **Fit** | Zoom to the occupied cells |
| **Live** | Unticked, the map stops fetching cells, drones and overlays and holds what it shows; geofences keep coming so the fence list stays true |

The map fits itself to the occupied cells the first time it loads and then
keeps your view. Changing the display resolution keeps the view too, and
re-inspects the cell under the point you last clicked at the new resolution.

### Polling and motion

| What | How often |
|---|---|
| Occupancy cells (`/api/v1/state/map`) | Once a second |
| Geofences | Once a second, even when Live is off |
| People, vehicles, orders, stations, regions, range | Once a second |
| Drones, hubs, reports | Four times a second |

Between polls each drone is drawn where its last reported speed and heading
put it at the current simulated time, and the map redraws at about 30 frames
a second while anything moves. Each poll replaces the prediction with the
truth, so a drone is never more than a quarter of a second of prediction off.
At high speed-ups the prediction stops 20 simulated seconds past a sample
rather than dead-reckon through a turn. The simulated clock comes from the
`x-sim-tick`, `x-sim-mode` and `x-sim-tick-seconds` headers on the entities
response.

### The cell inspector (State)

A click inspects the polygon under the pointer at the display resolution. A
click on empty ground asks `/api/v1/cells/lookup` for the cell there, so an
empty cell can be inspected too. The inspected cell is outlined with a dashed
blue line. Its contents appear below the map and stay current over the
WebSocket; see [dashboard.md](dashboard.md#state).

### Drawing tools (Geofences, Stations & regions)

These pages hand the map a tool through `window.krabshellMap`. In **Select**,
a click picks out the fence, station or region under it in its list (and a
row in the list zooms to it). The drawing tools take over clicks while active:

| Tool | Page | How |
|---|---|---|
| Draw circle | Geofences | Click the centre, then the edge, or type the radius. |
| Draw polygon | Geofences | Click each corner. Click the first corner again, press Enter, or press **Open fence** to finish. Backspace or Delete removes the last corner. |
| Add station | Stations & regions | Click where it goes, then name it and give its slots. |
| Add region | Stations & regions | Click the centre, then the edge, or type the radius, then name it. |

Esc cancels a draft. The draft is drawn in the accent colour until it is
submitted. What the forms send is in [geofences.md](geofences.md) and
[stations.md](stations.md).

## The A5 grid overlay

The occupancy layer is the A5 grid itself. A5 cells are pentagons; the
server aggregates occupancy, which lives at the scenario's resolution, to
parent cells at the display resolution and returns each with its boundary
ring (`GET /api/v1/state/map?res=N`). That keeps a large world to a few
hundred polygons in the browser. Lower the display resolution for a city-wide
view; raise it to see single cells. The slider runs from 0 to the occupancy
resolution.

Hit testing uses the same projected polygons the map draws, so a click lands
in exactly the cell you see. If the published view was capped by
`run.max_published_cells`, the status line under the map says some cells are
missing; use a coarser display resolution or raise the cap.

The status line also gives the tick, the number of cells drawn, the
resolution and how many agents the busiest cell holds. Weather and Conflicts
draw their own cells (weather cells, conflict cells) as page layers in the
same way; see [weather.md](weather.md) and [conflicts.md](conflicts.md).
