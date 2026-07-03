# Controlador Reactivo Follow The Gap — F1TENTH (Brands Hatch)

Implementación de un controlador reactivo de navegación autónoma basado en el algoritmo **Follow The Gap (FTG)**, desarrollado para el simulador oficial de F1TENTH sobre ROS2 (Humble), como parte del proyecto del curso de Vehículos Autónomos. El controlador fue probado en la pista **Brands Hatch**.

---

## Tabla de contenidos

- [1. Descripción del enfoque](#1-descripción-del-enfoque)
- [2. Estructura del código](#2-estructura-del-código)
- [3. Instrucciones de instalación](#3-instrucciones-de-instalación)
- [4. Instrucciones de ejecución](#4-instrucciones-de-ejecución)
- [5. Parámetros ajustables](#5-parámetros-ajustables)
- [6. Contador de vueltas y cronómetro](#6-contador-de-vueltas-y-cronómetro)
- [7. Resultados](#7-resultados)

---

## 1. Descripción del enfoque

El controlador implementa **Follow The Gap**, un algoritmo reactivo (sin mapa ni planificación global) que decide hacia dónde dirigir el carro únicamente a partir de la lectura instantánea del sensor LiDAR (`/scan`). La idea central es: *"dirígete hacia el espacio libre más amplio y profundo que tengas frente a ti"*.

El pipeline de decisión en cada ciclo del LiDAR es el siguiente:

1. **Preprocesamiento del LiDAR** — limpieza de valores inválidos (`NaN`, `inf`) y recorte de distancias fuera de rango.
2. **Disparity Extender** — detecta saltos abruptos de distancia entre rayos vecinos (bordes de obstáculos) y "engorda" artificialmente el obstáculo más cercano, evitando que el carro intente pasar rozando un borde delgado (p. ej. una mediana).
3. **Limitación del campo de visión (FOV)** — se descartan los rayos que apuntan demasiado hacia los costados o atrás, evitando que el algoritmo elija un punto objetivo que obligue a un giro de 180°.
4. **Burbuja de seguridad** — se anula (pone a cero) un rango de rayos alrededor del punto más cercano detectado, como margen de seguridad adicional.
5. **Búsqueda del gap máximo** — se identifica el segmento contiguo de espacio libre más amplio dentro del campo de visión.
6. **Selección del punto objetivo** — se calcula un punto dentro del gap como una combinación ponderada entre el **centro geométrico** del hueco (para estabilidad y trayectoria centrada) y el **punto más lejano** (para aprovechar al máximo el espacio libre).
7. **Control de velocidad adaptativo** — se distingue entre:
   - **Recta confirmada**: gap ancho (> 50°), profundo (> 4 m) y ángulo de giro casi nulo (< 5°) → velocidad máxima.
   - **Curva / transición**: velocidad de crucero escalada según qué tan cerrado es el ángulo de giro requerido.
8. **Publicación del comando** — se envía el ángulo de giro y la velocidad calculados al tópico `/drive` mediante un mensaje `AckermannDriveStamped`.

Este enfoque permite que el vehículo navegue de forma autónoma sin colisiones, alcanzando alta velocidad en tramos rectos y reduciendo la velocidad de forma proporcional en curvas.

---

## 2. Estructura del código

El controlador se encuentra en:

```
src/controllers/controllers/gap_node.py
```

### Clase principal: `ReactiveFollowGap(Node)`

| Método | Función |
|---|---|
| `__init__` | Configura parámetros, suscripciones (`/scan`, `/ego_racecar/odom`) y el publicador (`/drive`). |
| `preprocess_lidar(ranges)` | Limpia el array del LiDAR (NaN/inf, recorte de rango). |
| `extend_disparities(ranges, angle_increment)` | Implementa el "disparity extender": detecta bordes y extiende obstáculos cercanos para evitar colisiones por bordes delgados. |
| `limit_fov(ranges, angle_min, angle_increment, fov_deg)` | Limita el LiDAR a un cono frontal, evitando giros de 180°. |
| `find_max_gap(free_space_ranges)` | Encuentra el segmento contiguo de espacio libre más amplio. |
| `find_best_point(start_i, end_i, ranges)` | Calcula el punto objetivo dentro del gap (combinación centro + punto lejano). |
| `lidar_callback(data)` | Callback principal: ejecuta el pipeline completo y publica el comando de manejo. |
| `odom_callback(msg)` | Callback de odometría: implementa el contador de vueltas y cronómetro (ver sección 6). |
| `main()` | Inicializa el nodo ROS2 y lo mantiene en ejecución (`rclpy.spin`). |

### Tópicos utilizados

| Tópico | Tipo de mensaje | Uso |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Entrada — lectura del LiDAR |
| `/ego_racecar/odom` | `nav_msgs/Odometry` | Entrada — posición del carro (conteo de vueltas) |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | Salida — comando de ángulo y velocidad |

---

## 3. Instrucciones de instalación

### Requisitos previos
- Ubuntu 22.04
- ROS2 Humble instalado y funcionando

### Pasos

```bash
# 1. Instalar el F1TENTH Gym (libreria de fisica del simulador)
cd $HOME
git clone https://github.com/f1tenth/f1tenth_gym
sudo apt install python3-pip
cd f1tenth_gym && pip3 install -e .

# 2. Clonar este repositorio
cd $HOME
git clone https://github.com/Giobapena/f1tenth-follow-the-gap-brandshatch-VNT.git F1Tenth-Repository

# 3. Instalar dependencias de ROS2
cd ~/F1Tenth-Repository
sudo apt install python3-rosdep2
rosdep update
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y

# 4. Compilar el workspace
colcon build
```

### Configuración del mapa (Brands Hatch)

```bash
cd ~/F1Tenth-Repository/src/f1tenth_gym_ros/maps
git clone https://github.com/f1tenth/f1tenth_racetracks.git
mkdir -p BrandsHatch
cp f1tenth_racetracks/BrandsHatch/BrandsHatch_map.yaml BrandsHatch/
cp f1tenth_racetracks/BrandsHatch/BrandsHatch_map.png BrandsHatch/
```

Editar `src/f1tenth_gym_ros/config/sim.yaml` y ajustar:

```yaml
map_path: '/home/<tu_usuario>/F1Tenth-Repository/src/f1tenth_gym_ros/maps/BrandsHatch/BrandsHatch_map'
```

---

## 4. Instrucciones de ejecución

### Terminal 1 — Levantar el simulador

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Esto abre RViz mostrando el carro sobre la pista de Brands Hatch.

### Terminal 2 — Ejecutar el controlador Follow The Gap

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run controllers gap_node
```

El carro debería comenzar a moverse de forma autónoma, navegando la pista sin intervención manual.

---

## 5. Parámetros ajustables

Todos los parámetros se encuentran al inicio de la clase o dentro de `lidar_callback`, y pueden ajustarse según la pista o el comportamiento deseado:

| Parámetro | Descripción | Valor usado |
|---|---|---|
| `bubble_radius` | Radio de seguridad alrededor del obstáculo más cercano | `0.5` m |
| `car_half_width` | Medio ancho del carro (usado en disparity extender) | `0.25` m |
| `disparity_threshold` | Diferencia mínima entre rayos para considerarla un borde | `0.25` m |
| `fov_deg` | Campo de visión frontal considerado | `85°` |
| `max_steering_angle` | Ángulo máximo de giro permitido | `34°` |
| `max_speed` | Velocidad máxima en recta confirmada | `7.0` |
| `cruise_speed` | Velocidad de crucero en curvas/transición | `4.5` |
| `min_speed` | Velocidad mínima en curvas cerradas | `1.5` |

---

## 6. Contador de vueltas y cronómetro

El nodo se suscribe a `/ego_racecar/odom` para obtener la posición (x, y) del carro en tiempo real. La lógica implementada es:

1. Se registra la posición inicial del carro como **línea de salida**.
2. Se marca cuando el carro se **aleja** lo suficiente de esa posición (`leave_zone_radius`), confirmando que está dando la vuelta.
3. Cuando el carro **regresa** cerca de la línea de salida (`start_zone_radius`) después de haberse alejado, se cuenta una vuelta y se calcula el tiempo transcurrido desde la vuelta anterior.

Cada vuelta completada se muestra en la terminal como evidencia:

```
[INFO] [reactive_node]: VUELTA 1 completada - Tiempo: 105.34 segundos
[INFO] [reactive_node]: VUELTA 2 completada - Tiempo: 98.21 segundos
```

---

## 7. Resultados

- El controlador completa vueltas consecutivas en la pista Brands Hatch sin colisionar.
- Alcanza velocidad máxima en tramos rectos confirmados y reduce velocidad de forma proporcional en curvas.
- El tiempo de vuelta y el conteo de vueltas se registran automáticamente y se muestran por consola durante la ejecución.

---

## Autor

Proyecto desarrollado para el curso de Vehículos Autónomos — Controlador reactivo Follow The Gap sobre F1TENTH / ROS2 Humble.
