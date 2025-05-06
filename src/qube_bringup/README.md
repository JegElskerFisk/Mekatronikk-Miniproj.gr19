# qube_bringup

Denne pakken setter sammen de ulike delene av Quanser Qube-systemet i ROS2. Den inkluderer oppstart av både den fysiske eller simulerte Quben, drivere, robotbeskrivelse, og RViz-visualisering – og samler dette i ett system via en hoved-launchfil.

## Innhold og struktur

Prosjektstrukturen i `qube_bringup` er organisert som følger:
```
qube_bringup/
├── launch/
│   └── bringup.launch.py
├── qube_bringup/
│   └── __init__.py
├── resource/
│   └── qube_bringup
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── urdf/
│   └── controlled_qube.urdf.xacro
├── package.xml
├── setup.cfg
└── setup.py
```
# Oversikt over filene

### Launch

- `launch/bringup.launch.py`: Samler hele Qube-systemet. Inkluderer launch for `qube_driver`, starter `robot_state_publisher`, og åpner RViz med ferdigkonfigurasjon.
  - Leser `controlled_qube.urdf.xacro` og genererer URDF via `xacro`
  - Parametere (`device`, `baud_rate`, `simulation`) kan konfigureres ved oppstart
  - Inkluderer `qube_driver.launch.py` fra `qube_driver`
  - Leser `view_qube.rviz` fra `qube_description`

### URDF

- `urdf/controlled_qube.urdf.xacro`: Utvidet versjon av basismodellen med integrasjon for ROS2 Control. Inkluderer drivermakro (`qube_driver_ros2_control`) og parametere for device, baud_rate og simulation.

### ROS2-metadata og bygging

- `package.xml`: Definerer pakkeinformasjon og avhengigheter:
  - `xacro`, `robot_state_publisher`, `rviz2`
  - `qube_driver` og `qube_description` som nødvendige komponentpakker
- `setup.py`, `setup.cfg`, `resource/`: Gjør pakken byggbar med `ament_python`, og sikrer at URDF- og launch-filer installeres til riktig plass.

### Python-struktur

- `qube_bringup/__init__.py`: Init-fil for ROS2-kompatibel Python-pakke. Kreves av strukturen.

### Tester

- `test/test_flake8.py`, `test_pep257.py`, `test_copyright.py`: Sikrer overholdelse av kodestandard og lisens.

## Systeminteraksjoner og flyt

Følgende komponenter samarbeider i bringup-systemet:

```
                                +-----------------------------+
                                | /joint_state_publisher_gui  |
                                +-----------------------------+
                                      ^               |
                                      |               v
                          +-------------------+   +---------------+
                          | /robot_description|   | /joint_states |
                          +-------------------+   +---------------+
                                      ^               |
                                      |               v
                                +-----------------------------+
                                |   /robot_state_publisher    |
                                +-----------------------------+
                                              |
                                              v
                                        +-----------+
                                        |    /tf    |
                                        +-----------+
                                              |
                                              v
                        +--------------------------------------------+
                        | /transform_listener_impl_5f18b7b6a550      |
                        |              (RViz lytter)                 |
                        +--------------------------------------------+
 ```                       
- `bringup.launch.py` starter alle relevante noder og launcher
- `robot_state_publisher` parser `controlled_qube.urdf.xacro` og publiserer `/robot_description` og `/tf`
- `joint_state_broadcaster` og `velocity_controller` settes opp av ROS2 Control via `qube_driver`
- `rviz2` visualiserer hele systemet via ferdigoppsett fra `qube_description/rviz/view_qube.rviz`

## Modellen

Modellen som brukes er identisk med `qube_description`, men er utvidet med en ekstra `ros2_control`-makro for å gjøre modellen kontrollerbar.

### URDF-argumenter

- `device`: Navn på port (f.eks. `/dev/ttyACM0`)
- `baud_rate`: Standard `115200`
- `simulation`: `true` eller `false` – velg mellom fysisk eller simulert enhet

## Bygging og kjøring

### Bygg pakken

```bash
colcon build --symlink-install
source install/setup.bash
```

## Start hele Qube-systemet:

```bash
ros2 launch qube_bringup bringup.launch.py
```

Du kan også spesifisere parametere:

```bash
ros2 launch qube_bringup bringup.launch.py device:=/dev/ttyACM1 simulation:=false
```

Dette starter:

- Driveren (`qube_driver`)
- RViz2 med forhåndsdefinert layout
- Publisering av `/robot_description`, `/joint_states`, `/tf`
- Visualisering og kontroll

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

Denne pakken er ment for å samle og konfigurere et komplett Qube-oppsett. URDF-filen  
`controlled_qube.urdf.xacro` kan også brukes i egne kontrollpakker.

Makroen `qube_driver_ros2_control` i URDF definerer en kontrollerbar Qube-robot, som kan tilpasses med egne parametere eller utvides med flere controllere.

