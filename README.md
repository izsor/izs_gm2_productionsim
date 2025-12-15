# izs_gm2_productionsim

ROS 2-alapú gyártóállomás-szimuláció (manufacturing station simulation), ahol virtuális eszközök együttműködnek egy minőségellenőrzési (quality control) workflow-ban.

**Röviden:**  
A **line** új munkadarabokat (workpiece) generál → a **device** “mér” (szimulált szenzoradatok) → a **quality** WPNo alapján párosít és dönt (**OK/NOK**) → a **stats** összesít → a **viz** RViz2-ben vizualizálja a folyamatot.

---

## Feladat / beadandó hivatkozás

- Beadandó leírás: https://sze-info.github.io/ajr/feleves_beadando/nagyfeleves/

---

## Fő funkciók

- ✅ Workpiece generálás (paraméterezhető ütem + hibavalószínűség)
- ✅ Szimulált mérőállomás (2 szenzor, zajjal)
- ✅ WPNo alapú párosítás és kiértékelés (OK/NOK)
- ✅ Statisztika publikálás + reset service
- ✅ RViz2 vizualizáció (MarkerArray)

---

## Követelmények

- ROS 2 (rclpy, launch)
- Python 3
- Függőségek (ROS 2):
  - `std_msgs`
  - `std_srvs`
  - `geometry_msgs`
  - `visualization_msgs`
  - `launch`, `launch_ros`

> Tipp: ha rosdep-et használsz, a függőségek automatikusan telepíthetők.

---

## Projekt struktúra
izs_gm2_productionsim/
izs_gm2_productionsim/
init.py
line_node.py
measure_node.py
quality_node.py
stats_node.py
viz_node.py
utils.py
launch/
productionsim.launch.py
resource/
izs_gm2_productionsim
package.xml
setup.py
setup.cfg
README.md

---

## Gyors indítás

### 1) Workspace előkészítés
```bash
cd ~/ros2_ws/src
git clone https://github.com/izsor/izs_gm2_productionsim.git
```
### 2) Függőségek telepítése (ajánlott)
```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
### 3) Build
```bash
cd ~/ros2_ws
colcon build --packages-select izs_gm2_productionsim --symlink-install
source install/setup.bash
```
### 4) Indítás
```bash
ros2 launch izs_gm2_productionsim productionsim.launch.py
```

## Node-ok / executable-ok

### A csomag executable nevei:

line → izs_gm2_productionsim.line_node:main

device → izs_gm2_productionsim.measure_node:main

quality → izs_gm2_productionsim.quality_node:main

stats → izs_gm2_productionsim.stats_node:main

viz → izs_gm2_productionsim.viz_node:main

### Ellenőrzés: 
```bash
source ~/ros2_ws/install/setup.bash
ros2 pkg executables izs_gm2_productionsim
``` 
flowchart LR
  %% =========================
  %% NODES
  %% =========================
  subgraph N[Nodes]
    L[line]
    D[device]
    Q[quality]
    S[stats]
    V[viz]
    RZ[RViz2]
  end

  %% =========================
  %% TOPICS
  %% =========================
  subgraph T[Topics]
    TW[( /line/workpiece )]
    TA[( /measurements/device/sensor_A )]
    TB[( /measurements/device/sensor_B )]
    TR[( /qc/result )]
    TS[( /qc/stats )]
    TV[( /viz/workpieces )]
  end

  %% =========================
  %% SERVICES
  %% =========================
  subgraph SV[Services]
    SR[( /qc/reset_stats )]
  end

  %% =========================
  %% PUBLISH / SUBSCRIBE LINKS
  %% =========================

  %% line -> workpiece
  L -- "publish: std_msgs/String (JSON)\nWPNo + true_quality + timestamp" --> TW

  %% device subscribes to workpiece, publishes measurements
  TW -- "subscribe\nstd_msgs/String (JSON)" --> D
  D -- "publish: std_msgs/String (JSON)\nsensor_A value + WPNo" --> TA
  D -- "publish: std_msgs/String (JSON)\nsensor_B value + WPNo" --> TB

  %% quality subscribes to both sensors, publishes qc result
  TA -- "subscribe\nstd_msgs/String (JSON)" --> Q
  TB -- "subscribe\nstd_msgs/String (JSON)" --> Q
  Q -- "publish: std_msgs/String (JSON)\ndecision=OK/NOK, diff, threshold" --> TR

  %% stats subscribes qc result, publishes aggregated stats + provides reset service
  TR -- "subscribe\nstd_msgs/String (JSON)" --> S
  S -- "publish: std_msgs/String (JSON)\ncounts + ratios (+ optional accuracy)" --> TS
  SR -. "service: std_srvs/Trigger\nreset counters" .-> S

  %% viz subscribes workpiece + qc result, publishes markers
  TW -- "subscribe\nstd_msgs/String (JSON)" --> V
  TR -- "subscribe\nstd_msgs/String (JSON)" --> V
  V -- "publish: visualization_msgs/MarkerArray\ncolored workpieces" --> TV
  TV -- "display\nMarkerArray" --> RZ