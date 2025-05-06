# qube_description

Denne pakken beskriver den geometriske modellen av Quanser Qube i ROS2-format ved hjelp av Xacro og URDF. Den brukes til visualisering i RViz og danner grunnlag for bruk i simulator eller kontrollsystem. All kode og filstruktur følger standard ROS2-praksis.

## Innhold og struktur

Prosjektstrukturen i `qube_description` er organisert som følger:

```
qube_description/
├── launch/
│   └── view_qube.launch.py
├── rviz/
│   └── view_qube.rviz
├── urdf/
│   ├── qube.macro.xacro
│   └── qube.urdf.xacro
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── qube_description/
│   └── __init__.py
├── resource/
│   └── qube_description
├── package.xml
├── setup.py
└── setup.cfg
```

## Oversikt over filene

### URDF/Xacro-modeller

- `urdf/qube.macro.xacro`: Inneholder makrodefinisjonen `qube` som beskriver selve Quben: base, disk, indikator og tilhørende joints og materialer. Bruker `xacro:property`-variabler for å parametrisere dimensjoner.
- `urdf/qube.urdf.xacro`: Inkluderer makroen over og definerer en statisk kobling mellom Quben og `world` via `world_to_base`-joint.

### Visualisering

- `rviz/view_qube.rviz`: Ferdig konfigurert RViz-layout som viser Quben i `world`-koordinatsystem. Viser en 3D-modell og et grid, og bruker `/robot_description` som kilde.

### Launch

- `launch/view_qube.launch.py`: ROS2-launchfil som gjør følgende:
  - Leser og parser `qube.urdf.xacro` til XML via `xacro`.
  - Starter `robot_state_publisher` med URDF-data.
  - Starter `joint_state_publisher_gui` for manuell manipulering av joints.
  - Starter `rviz2` med ferdig konfigurasjon fra `view_qube.rviz`.

### ROS2-metadata og bygging

- `package.xml`: ROS2-pakkedefinisjon som spesifiserer avhengigheter:
  - `xacro`
  - `robot_state_publisher`
  - `joint_state_publisher_gui`
  - `rviz2`
  - `std_msgs`
- `setup.py`, `setup.cfg`, `resource/`: Gjør pakken byggbar med `ament_python` og installerer launch-, urdf- og rviz-filer under `share/`.

### Python-struktur

- `qube_description/__init__.py`: Init-fil for ROS2-kompatibel Python-pakke. Ikke funksjonell i denne pakken, men nødvendig for strukturen.

### Tester

- `test/test_flake8.py`, `test_pep257.py`, `test_copyright.py`

## Systeminteraksjoner og flyt

Følgende komponenter samarbeider i visualiseringssystemet:

```
                       +---------------------------+     +---------------------------+
                       | /joint_state_broadcaster  |     |   /velocity_controller    |    
                       +---------------------------+     +---------------------------+
                                   |
                                   v
                            +---------------+
                            | /joint_states |
                            +---------------+
                                   |
                                   v
                       +-----------------------------+
                       |  /robot_state_publisher     |
                       +-----------------------------+
                               |                  |
                               v                  v
                  +--------------------+       +---------+
                  | /robot_description |       |   /tf   |
                  +--------------------+       +---------+
                         |                          |
                         v                          v
            +-----------------------+   +--------------------------+
            |  /controller_manager  |   | /transform_listener_impl |
            +-----------------------+   +--------------------------+
```




- `view_qube.launch.py` starter alle noder.
- `robot_state_publisher` får URDF fra `qube.urdf.xacro`, parser via `xacro`.
- `joint_state_publisher_gui` gir manuelt input til `/joint_states`.
- `rviz2` henter data fra `/robot_description` og `/joint_states`.
- `view_qube.rviz` definerer visningsoppsettet (kamera, grid, fixed frame `world`).



## Modellen

### Base (`link name="base"`)

- En svart kube, 10.2 cm x 10.2 cm x 11.2 cm.
- To røde striper (topp og bunn).
- Fire grå sylinderformede føtter.

### Disk (`link name="disk"`)

- Rød roterbar sylinder.
- Hvit indikator (viser) montert på disken.

### Joints

- `disk_joint`: Kontinuerlig joint mellom base og disk (roterer rundt Z-aksen).
- `world_to_base`: Fast joint mellom `world` og `base`.

## Bygging og kjøring

### Bygg pakken

```bash
colcon build --symlink-install
source install/setup.bash

## Start visualisering

```bash
ros2 launch qube_description view_qube.launch.py
```

Dette starter:

- RViz med modellen
- GUI for manuell joint-styring
- Publisering av `/tf`, `/robot_description`, og `/joint_states`

## Testing

Testene kan kjøres slik:

```bash
colcon test
colcon test-result --verbose
```

Pakken inneholder tester for:

- Kodeformat (PEP8)
- Dokumentasjonsstil (PEP257)
- Copyright-header

## Avhengigheter

Disse pakkene må være installert:

```bash
sudo apt install ros-${ROS_DISTRO}-xacro \
                 ros-${ROS_DISTRO}-robot-state-publisher \
                 ros-${ROS_DISTRO}-joint-state-publisher-gui \
                 ros-${ROS_DISTRO}-rviz2
```

## Gjenbruk og utvidelse

`qube.macro.xacro` er designet som en gjenbrukbar makro. Den kan inkluderes i andre URDF/Xacro-filer som følger:

```xml
<xacro:include filename="$(find qube_description)/urdf/qube.macro.xacro"/>
<xacro:qube/>
```
