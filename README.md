# Qube – ROS2 Miniprosjekt

Dette prosjektet er et komplett ROS2-oppsett for styring og simulering av en [**Quanser Qube**](https://www.quanser.com/products/qube-servo-2/). Prosjektet integrerer geometri, maskinvare, kontroll og visualisering i ett samlet system.


DISCLAIMER (bug) : Kommandoer fra PID kontroller blir ikke videreført til fysisk qube
## Formål

Prosjektets mål er å:

- Lage en fullverdig ROS2-applikasjon med flere samarbeidsmoduler
- Styre en fysisk Qube og simulere en digital Qube parallelt
- Gi brukeren mulighet til å velge mellom simulator og fysisk enhet
- Visualisere tilstanden i RViz
- Styre vinkelen via terminal og/eller GUI
- Dokumentere systemflyt, interaksjon og oppbygning på linje med standard ROS2-pakker

---

##  Prosjektstruktur

Prosjektet består av tre hovedpakker:

```
src/
├── qube_description/  # Geometrisk modell og URDF/Xacro
├── qube_bringup/      # Samler og launcher hele systemet
├── qube_controller/   # Kontrolleren for motoren
└── qube_driver/       # Maskinvaregrensesnitt og ROS2 Control
```
> Alle pakker bruker `ament_python` og er byggbare med `colcon`.

---

## Kom i gang

### 1. Opprett workspace og klon repoet

Før du bygger prosjektet, må du ha en ROS2-workspace. Klon repoet inn i `src/`.

```bash
mkdir -p ~/ros2_ws/src
cd ~/QUBE/src
git clone https://github.com/JegElskerFisk/Mekatronikk-Miniproj.gr19.git
cd ~/QUBE
colcon build --symlink-install
source install/setup.bash
```

---

## 2. Start systemet

Etter at workspace er bygd, kan systemet startes med:

```bash
ros2 launch qube_bringup bringup.launch.py
```

For simulasjon mode brukes argument:

```bash
ros2 launch qube_bringup bringup.launch.py simulation:=true
```

Du kan også spesifisere seriellport og simulasjonsmodus:

```bash
ros2 launch qube_bringup bringup.launch.py device:=/dev/ttyACM1 simulation:=false
```

PID kontroller startes med:
```bash
ros2 launch qube_controller controller.launch.py
```

Parametre kan endres etter oppstart med:
```bash
ros2 param set /pid_controller setpoint 3.0
```

---

## 3. Ved problemer med RViz

Dersom RViz-vinduet er **helt svart** (ikke engang viser grid), kan det skyldes manglende OpenGL-støtte, dette basert på erfaringer med Windows-maskiner.

**Workaround:** Tving RViz til å bruke programvarebasert rendering med:

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 run rviz2 rviz2
```

Dette gir lavere ytelse, men lar deg fortsatt bruke RViz dersom du mangler full GPU-støtte eller kjører via WSL/VM.

**Workaround 2:** Om det heller ikke skulle fungere er siste alternativ å starte RViz fra terminalen. Så hente inn roboten i RViz.

---

## Pakkeoversikt

### `qube_description/`

Inneholder den geometriske modellen av Quben, bygget i Xacro og URDF:

- `qube.macro.xacro`: Makro med parametrisert modell (base, disk, viser, føtter)
- `qube.urdf.xacro`: Enkel scene med `world` + Qube
- `view_qube.launch.py`: Viser modellen isolert i RViz

---

### `qube_driver/`

ROS2-driver for Quben, bygget på `ros2_control`. Denne pakken er hentet fra [adamleon/qube_driver](https://github.com/adamleon/qube_driver).

---

### `qube_bringup/`

Samler hele systemet: geometri, drivere, kontroll og visualisering.

- `controlled_qube.urdf.xacro`: URDF med ROS2 Control-integrasjon
- `bringup.launch.py`: Starter Qube-driver, RViz, og robot_state_publisher

Gir full kontroll over:

- Simulering vs. fysisk Qube
- Enhetens port og baud rate
- Synkronisert visning i RViz

---

### `qube_controller/`

Inneholder PID-kontrolleren som regulerer vinkelen på Quben. Noden leser fra `/joint_states` og publiserer til `/velocity_controller/commands`.

Hovedfil:
- `pid_controller.py`

Funksjon:
- PID-regulering med parametre: `kp`, `ki`, `kd`, `setpoint`, `max_velocity`, `integral_limit`, `derivative_filter_tau`
- Publiserer kommando som `Float64MultiArray`
- Parametere kan endres ved runtime eller ved launch

Eksempelkjøring:
```bash
ros2 run qube_controller pid_controller
```

Med justerte parametere:
```bash
ros2 run qube_controller pid_controller --ros-args -p kp:=2.0 -p setpoint:=1.57
```

---

## Avhengigheter

Før du bygger og kjører, sørg for at disse ROS2-pakkene er installert:

```bash
sudo apt install ros-${ROS_DISTRO}-xacro \
                 ros-${ROS_DISTRO}-robot-state-publisher \
                 ros-${ROS_DISTRO}-joint-state-publisher-gui \
                 ros-${ROS_DISTRO}-ros2-control \
                 ros-${ROS_DISTRO}-ros2-controllers \
                 ros-${ROS_DISTRO}-rviz2 \
                 ros-${ROS_DISTRO}-rclpy \
                 ros-${ROS_DISTRO}-std-msgs \
                 ros-${ROS_DISTRO}-sensor-msgs
```
