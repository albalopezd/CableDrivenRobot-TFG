# Robot continuo accionado por cables (2 GDL) — TFG

Trabajo de Fin de Grado. Diseño, construcción y control de un **robot continuo accionado por
cables** (*cable-driven continuum robot*) de 4 segmentos, accionado por 3 tendones, con
2 grados de libertad que posicionan el extremo en el plano XY. El robot se
modela con **cinemática de curvatura constante por tramos (PCC)** y se controla en tiempo real
con **ROS2 Jazzy + ros2_control**, un **Arduino** como intermediario y tres **servos LX-16A**.
Incluye cinemática directa e inversa, visualización en RViz, interfaz gráfica y validación
experimental sobre el robot físico.

<p align="center">
  <img width="32%" alt="IMG_2349" src="https://github.com/user-attachments/assets/8f0c098e-944a-4fb8-b166-56abb914526a" />
  <img width="32%" alt="IMG_2348" src="https://github.com/user-attachments/assets/4a34c468-30c1-42ad-9555-381706a02606" />
  <img width="22%" alt="WhatsApp Image" src="https://github.com/user-attachments/assets/88c259c2-adb1-46a8-80d0-2957801dee6f" />
</p>

---

## 🎬 Vídeos

| Primera aproximación de movimiento | Demostración IK | Demostración sliders
|:---:|:---:|:---:|
| [![Primera aproximación de movimiento](https://img.youtube.com/vi/9lynUKzb-aw/0.jpg)](https://www.youtube.com/shorts/9lynUKzb-aw) | [![Demo IK](https://img.youtube.com/vi/HkvytLLgjOQ/0.jpg)](https://youtu.be/HkvytLLgjOQ) | [![Demo sliders](https://img.youtube.com/vi/xODviwTWwIY/0.jpg)](https://youtu.be/xODviwTWwIY) |
| Pruebas iniciales de actuación de los cables y respuesta del robot. | Funcionamiento completo: control cinemático. | Funcionamiento por variación longitud de cables manual |

---

## Descripción del proyecto

Un robot continuo no tiene articulaciones rígidas: su estructura se deforma de forma continua.
En este caso, una columna central flexible recorre **4 segmentos** separados por discos, y
**3 cables (tendones)** dispuestos a **0°, +90° y −90°** (triángulo isósceles rectángulo) tiran
de los segmentos para curvar el robot en cualquier dirección del plano.

Al ser una **única sección de curvatura constante**, el robot tiene **2 grados de libertad**:
la **curvatura** (cuánto dobla) y la **orientación del plano de doblez** (hacia dónde dobla).
Esos 2 GDL posicionan el extremo en el plano **XY**. Los 3 tendones son redundantes respecto a
esos 2 GDL: aportan tensión y equilibrio, no una dirección de movimiento adicional. La
coordenada **z no es un grado de libertad independiente**, sino una consecuencia geométrica del
doblez (el extremo se acorta en vertical al curvarse).

El comportamiento se describe con el modelo **PCC (Piecewise Constant Curvature)**, que asume
curvatura constante en cada tramo y permite resolver:

- **Cinemática inversa (IK):** dada una posición objetivo del extremo `(x, y, z)`, calcula
  cuánto debe tirar cada cable.
- **Cinemática directa (FK):** dada la longitud de cada cable, estima la posición del extremo.

### Parámetros del robot

| Parámetro | Valor |
|---|---|
| Número de segmentos | 4 |
| Grados de libertad | 2 (curvatura + orientación del plano de doblez) |
| Separación entre discos | 0,053 m |
| Número de cables (tendones) | 3, a 0°, +90° y −90° (triángulo isósceles rectángulo) |
| Servos | LX-16A — IDs 7, 10 y 4 |

---

## Hardware

**Componentes principales:**

- 3× servos **LX-16A** (Lewansoul) en bus serie semidúplex, cada uno acciona un tendón.
- **Arduino** como pasarela entre el PC y el bus de servos.
- Fuente externa **6 V** (LiPo 2S) para alimentar los servos, con masa común con el Arduino.
- Piezas estructurales impresas en **PLA** (FDM): segmentos separadores, base, poleas de
  tendones y soportes.

**Arquitectura de comunicación (dos capas):**

```
PC  ──USB, 9600 baud──►  Arduino  ──SoftwareSerial, 115200 baud──►  Bus LX-16A (3 servos)
```

El Arduino recibe comandos del PC por el puerto USB (Hardware Serial) y los traduce al protocolo
del bus Lewansoul (SoftwareSerial), además de devolver lecturas de voltaje de los servos.

**Recursos de diseño en el repositorio:**

- [`schematics/`](schematics/) — esquema eléctrico (KiCad).
- [`parts/`](parts/) — piezas mecánicas (FreeCAD, `.FCStd`).
- [`meshes/`](meshes/) — mallas `.STL` para impresión y visualización en RViz.

---

## Arquitectura software (ROS2)

El control se reparte en cuatro paquetes ROS2 dentro de [`ros2_ws/src/`](ros2_ws/src/):

| Paquete | Función |
|---|---|
| `cable_robot_kinematics` | Cinemática PCC (IK/FK) y nodos puente, GUI e interfaz de control. |
| `cable_robot_description` | Modelo URDF (xacro), mallas y configuración de RViz. |
| `cable_robot_hardware` | Plugin de `ros2_control` (`SystemInterface`) que comunica por puerto serie con el Arduino. |
| `cable_robot_bringup` | Fichero de *launch* y configuración de controladores. |

**Nodos principales** (`cable_robot_kinematics`):

- `ik_bridge_cpp` — cinemática inversa: `desired_pose` → comandos de cable.
- `fk_bridge_cpp` — cinemática directa: estado de los cables → `estimated_pose`.
- `cable_bend_bridge_cpp` / `joint_state_combiner_cpp` — convierten el estado de los cables a
  curvatura por segmento para visualizar la deformación en RViz.
- `hardware_controller.py` — control proporcional de velocidad sobre los servos.
- `cable_slider_gui.py` — interfaz Tkinter para mover cada cable y enviar objetivos de IK.
- `data_logger.py` — registro de datos a CSV.

**Flujo de control:**

```
            ┌──────────────┐   cables   ┌────────────┐  serie  ┌─────────┐   ┌─────────┐
desired_pose│  ik_bridge   ├───────────►│ ros2_control├────────►│ Arduino ├──►│ LX-16A  │
   (RViz/   │   (IK PCC)   │            │  + plugin   │         │         │   │ (×3)    │
    GUI)    └──────────────┘            └─────┬───────┘         └─────────┘   └─────────┘
                                              │ /joint_states
                                        ┌─────▼───────┐
                                        │  fk_bridge  │──► estimated_pose ──► RViz / GUI
                                        │   (FK PCC)  │
                                        └─────────────┘
```

**Firmware Arduino** en [`Arduino/main/`](Arduino/main/): sketch principal (`main.ino`) más las
librerías `LX16A`, `TendonRobot` y `PCCkinematics` (que permiten también un modo de control
autónomo sin ROS).

---

## Requisitos

- **Ubuntu** con **ROS2 Jazzy**.
- `ros2_control` y `ros2_controllers` (`sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers`).
- `xacro` y `colcon`.
- **Arduino IDE** para cargar el firmware.
- *Opcional:* PlotJuggler (`sudo apt install ros-jazzy-plotjuggler-ros`) para visualizar
  señales en tiempo real.

---

## Instalación y compilación

```bash
git clone https://github.com/albalopezd/CableDrivenRobot-TFG.git
cd CableDrivenRobot-TFG/ros2_ws
colcon build
source install/setup.bash
```

Carga el firmware en el Arduino abriendo `Arduino/main/main.ino` en el Arduino IDE y subiéndolo
a la placa.

---

## Ejecución

Con el robot conectado por USB:

```bash
source /opt/ros/jazzy/setup.bash
source ~/CableDrivenRobot-TFG/ros2_ws/install/setup.bash
ros2 launch cable_robot_bringup bringup.launch.py
```

Esto arranca toda la arquitectura con un solo comando: `ros2_control`, `robot_state_publisher`,
**RViz**, los controladores, los puentes de cinemática (IK/FK) y la **GUI** de control.

El puerto serie se puede ajustar con el argumento `serial_port` (por defecto `/dev/ttyACM0`):

```bash
ros2 launch cable_robot_bringup bringup.launch.py serial_port:=/dev/ttyACM1
```

---

## Resultados y validación

Se grabaron tres pruebas experimentales sobre el robot físico (rosbags en
[`ros2_ws/bags/`](ros2_ws/bags/)):

1. **Sliders manuales** — actuación independiente de cada cable.
2. **Control por cinemática inversa** — seguimiento de una secuencia de *waypoints* en el plano XY.
3. **Evaluación de la cinemática directa** — comparación entre posición estimada y deseada.

**Error de posicionamiento en estado estacionario (plano XY):**

| Tipo de objetivo | Error XY |
|---|---|
| Ejes simples (X o Y) | ≈ 0,8 – 1,2 mm |
| Diagonales | ≈ 7 mm |

Los rosbags de las tres pruebas están disponibles en [`ros2_ws/bags/`](ros2_ws/bags/) para
su reproducción y análisis.

---

## Estructura del repositorio

```
CableDrivenRobot-TFG/
├── Arduino/      Firmware del Arduino (sketch + librerías LX16A, TendonRobot, PCCkinematics)
├── ros2_ws/      Workspace ROS2 (4 paquetes) + rosbags
├── parts/        Piezas mecánicas en FreeCAD (.FCStd)
├── meshes/       Mallas .STL para impresión 3D y RViz
├── schematics/   Esquema eléctrico (KiCad)
├── scenes/       Escenas de CoppeliaSim (validación cinemática previa)
└── csv/          Datos de validación cinemática (simulación)
```

---

## Validación previa en CoppeliaSim

Antes de la implementación física, la cinemática se validó en simulación con **CoppeliaSim**.
Las escenas están en [`scenes/`](scenes/) (`kinematic_3d_validation.ttt`,
`kinematic_3d_validation_complete_arm.ttt`) y los datos generados en [`csv/`](csv/), comparando
posición real (simulada) frente a la teórica para cinemática directa e inversa.

---

## Autoría

Trabajo de Fin de Grado — Alba López.
