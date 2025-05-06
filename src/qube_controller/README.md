# qube_controller

Denne ROS 2-pakken inneholder en PID-kontroller for Quanser Qube, implementert som en Python-node som abonnerer på `/joint_states` og publiserer kommandoer til `/velocity_controller/commands`.

Kontrolleren er parameterstyrt og kan enkelt justeres gjennom launchfiler eller ROS2-parametre. Den er kompatibel med ROS 2 control-systemet brukt i `qube_bringup`.

---

## Innhold

```
qube_controller/
├── qube_controller/
│   └── pid_controller.py
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── qube_controller
└── test/
    └── (eventuelle tester)
```

---

## Funksjon

PID-kontrolleren regulerer vinkelen på disken ved å motta målinger fra `JointState`-meldinger og sende ut et kontrollpådrag som begrenset hastighet.

Den:
- Leser motorvinkelen fra `/joint_states`
- Beregner feil mellom aktuell vinkel og ønsket `setpoint`
- Beregner P-, I- og D-ledd med filtrert derivasjon
- Begrenset output i henhold til `max_velocity`
- Publiserer hastighet som `Float64MultiArray` til `/velocity_controller/commands`

---

## Parametre

Disse parameterne kan settes via launch, YAML eller direkte i kode:

| Parameter              | Beskrivelse                              | Standardverdi |
|------------------------|------------------------------------------|---------------|
| `setpoint`             | Ønsket posisjon (rad)                    | `3.0`         |
| `kp`                   | Proporsjonalforsterkning                 | `1.0`         |
| `ki`                   | Integratorforsterkning                   | `0.0`         |
| `kd`                   | Derivatorforsterkning                    | `0.1`         |
| `max_velocity`         | Maksimalt tillatt utsignal               | `5.0`         |
| `integral_limit`       | Begrensning av integrator                | `1.0`         |
| `derivative_filter_tau`| Tidskonstant for lavpassfiltrering av D | `0.01`        |

---

## Kjøre node

Etter at pakken er bygget:

```bash
ros2 run qube_controller pid_controller
```

Krever at `/joint_states` publiseres og at `/velocity_controller/commands` er aktiv i kontrollsystemet.

---

## Integrasjon i systemet

Noden forventer at:
- `qube_driver` er aktiv
- `velocity_controller` er lastet og aktivert
- `motor_joint` finnes i URDF og har en `velocity` kommando

---

## Eksempel: PID i bruk

```bash
ros2 run qube_controller pid_controller
```

Eller sett parametere direkte:

```bash
ros2 run qube_controller pid_controller --ros-args -p kp:=2.0 -p setpoint:=1.57
```

---

## Feilhåndtering

Kontrolleren håndterer:
- Manglende `motor_joint` i `joint_states`
- Tidstap
- Deling på null i derivasjon
- Andre kjente unntak logges med `get_logger()`

---

## Fremtidig arbeid

- Ekstern YAML-konfigurasjon for enklere tuning
- Logging av responsdata
- Dynamisk referanse via nytt topic

---

## Avhengigheter

- `rclpy`
- `sensor_msgs`
- `std_msgs`

```bash
sudo apt install ros-${ROS_DISTRO}-rclpy \
                 ros-${ROS_DISTRO}-sensor-msgs \
                 ros-${ROS_DISTRO}-std-msgs
```
