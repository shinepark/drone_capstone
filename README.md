# Dual Drone Flight Planner

An interactive, browser-based tool for generating optimized dual-drone survey flight paths and exporting them as `.waypoints` files for **Mission Planner** (ArduPilot / QGC WPL 110 format).

**→ [Live Demo](https://shinepark.github.io/drone_capstone)**

---

## Features

- **No installation required** — runs entirely in your browser, no data sent to any server
- **Dual-drone coverage** — automatically splits your field into two equal zones and generates independent lawnmower sweep paths for each drone
- **Live field preview** — see your polygon and the drone split line update as you enter coordinates
- **Configurable parameters** — altitude, flight time, sweep spacing, cruise speed, and battery reserve
- **Direct download** — exports `.waypoints` files ready to load into Mission Planner

---

## How to Use

### 1. Enter your field boundary

Input the GPS coordinates (latitude / longitude) of each corner of your field. You need at least **3 points** to define a polygon. The field preview updates live as you type.

> **Tip:** You can get coordinates from Google Maps by right-clicking any point on the map.

### 2. Set flight parameters

| Parameter | Description | Default |
|---|---|---|
| **Flight Altitude** | Height above ground in metres | 50 m |
| **Max Flight Time** | Maximum battery flight time in seconds | 900 s (15 min) |
| **Sweep Line Spacing** | Distance between parallel sweep passes | 5.5 m |
| **Cruise Speed** | Drone forward speed in m/s | 20 m/s |
| **Battery Reserve** | Safety margin kept in reserve (%) | 20% |

### 3. Generate & download

Click **▶ Generate Mission Files**. The planner will:

1. Split the field polygon vertically into two equal halves
2. Generate a lawnmower sweep pattern for each half
3. Show estimated waypoint counts and flight times
4. Provide a **Download .waypoints** button for each drone

### 4. Load into Mission Planner

1. Open Mission Planner
2. Go to **Flight Plan** tab
3. Click **Load WP File** and select the downloaded `.waypoints` file
4. Review the mission on the map, then upload to your flight controller

---

## Waypoint File Format

Files use the standard **QGC WPL 110** format compatible with ArduPilot and Mission Planner:

```
QGC WPL 110
0   1   0   16  0   0   0   0   <home_lat>  <home_lon>  <alt>   1
1   0   3   16  0   0   0   0   <lat>       <lon>       <alt>   1
...
N   0   3   20  0   0   0   0   0           0           0       1   ← RTL
```

Each mission ends with a **Return to Launch (RTL)** command.

---

## Deployment (GitHub Pages)

1. **Fork or clone** this repository
2. Make sure `index.html` is in the root of the `main` branch
3. Go to your repo → **Settings** → **Pages**
4. Set source to `Deploy from a branch` → `main` → `/ (root)`
5. Click **Save** — your site will be live at:
   ```
   https://<your-username>.github.io/<repo-name>
   ```

---

## Local Development

No build tools or dependencies needed. Just open `index.html` directly in any modern browser:

```bash
git clone https://github.com/yourusername/your-repo-name.git
cd your-repo-name
open index.html        # macOS
# or
start index.html       # Windows
# or
xdg-open index.html    # Linux
```

---

## Python Script (Advanced)

The [`flight_planner.py`](./flight_planner.py) script provides the full backend implementation including:

- Multi-battery **resume capability** — continues from the last waypoint after battery swap
- **State persistence** — saves and loads flight progress to JSON
- Batch `.waypoints` export for multi-flight missions
- Requires: `shapely`, `numpy`

```bash
pip install shapely numpy
python flight_planner.py
```

---

## Limitations & Notes

- The field split is currently **vertical** (by longitude midpoint), falling back to horizontal if the polygon shape requires it
- Sweep spacing is approximated as `metres ÷ 111,320` degrees — accurate for small fields but may drift slightly at high latitudes
- Flight time estimates assume constant cruise speed with no wind or acceleration
- Always **review generated waypoints in Mission Planner** before flying and verify compliance with local airspace regulations

---

## License

MIT — free to use, modify, and distribute.
