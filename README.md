# Tutorial: Controlador Follow The Gap para F1TENTH (pista Brands Hatch)

---

## 1. ¿Cómo funciona el enfoque? (Follow The Gap)

Para entender el código, primero hay que entender la idea. En cada lectura del LiDAR (tópico `/scan`), el nodo hace lo siguiente, en orden:

**Paso 1 — Limpiar los datos del LiDAR.**
El LiDAR a veces entrega valores raros (`NaN`, `inf`). Antes de usar los datos, se limpian y se recortan a un rango máximo razonable (`max_range`).

**Paso 2 — "Engordar" los obstáculos cercanos (disparity extender).**
Si el LiDAR detecta un salto brusco entre dos rayos vecinos (por ejemplo, un rayo mide 1 m y el de al lado mide 5 m), eso significa que hay un borde de un obstáculo ahí. El problema es que si solo miras "espacio libre", el carro podría intentar pasar rozando ese borde. Para evitarlo, el código toma el valor más cercano de ese salto y lo "expande" varios rayos hacia el lado del espacio libre, simulando que el obstáculo es un poco más ancho de lo que realmente es. Así el carro le da más margen.

**Paso 3 — Limitar el campo de visión (FOV).**
No tiene sentido que el carro considere lo que hay a los costados o atrás — eso podría hacer que el algoritmo elija un "gap" que obligue a un giro de 180°. Por eso se recorta el LiDAR a un cono frontal (en mi caso, 85° hacia cada lado).

**Paso 4 — Poner una burbuja de seguridad alrededor de lo más cercano.**
Se busca el punto más cercano detectado y se anula (se pone en cero) un rango de rayos alrededor de él. Esto es una segunda capa de seguridad, además del disparity extender.

**Paso 5 — Encontrar el gap más grande.**
Con todo lo anterior ya aplicado, se busca el tramo contiguo de espacio libre más amplio dentro del campo de visión. Ese es el "gap".

**Paso 6 — Elegir el punto objetivo dentro del gap.**
Aquí no simplemente se apunta al punto más lejano ni tampoco solo al centro del gap — se hace una combinación de ambos (una especie de promedio ponderado). Apuntar solo al punto más lejano puede hacer que la trayectoria sea inestable; apuntar solo al centro puede ser demasiado conservador. La combinación da un resultado más estable en la práctica.

**Paso 7 — Decidir la velocidad.**
Si el gap es ancho, profundo y casi no hay que girar, se interpreta como una recta y se va a velocidad máxima. Si no, se usa una velocidad de crucero que se reduce mientras más cerrado sea el giro necesario.

**Paso 8 — Publicar el comando.**
Finalmente se publica el ángulo de giro y la velocidad calculados al tópico `/drive`, como un mensaje `AckermannDriveStamped`, que es lo que el simulador espera para mover el carro.

---

## 2. Cómo está organizado el código

Todo el controlador vive en un solo archivo:

```
src/controllers/controllers/gap_node.py
```

Es un nodo de ROS2 (`ReactiveFollowGap`, que hereda de `Node`). Si lo abres, vas a encontrar estos métodos, más o menos en el orden en que se usan:

- **`__init__`** — Aquí se configuran los parámetros del algoritmo y se crean las suscripciones (`/scan` para el LiDAR, `/ego_racecar/odom` para la posición del carro) y el publicador (`/drive`).
- **`preprocess_lidar`** — Corresponde al Paso 1: limpia el array de distancias del LiDAR.
- **`extend_disparities`** — Corresponde al Paso 2: el disparity extender.
- **`limit_fov`** — Corresponde al Paso 3: recorta el LiDAR al cono frontal.
- **`find_max_gap`** — Corresponde al Paso 5: encuentra el gap más grande.
- **`find_best_point`** — Corresponde al Paso 6: calcula el punto objetivo dentro del gap.
- **`lidar_callback`** — Es el método que se ejecuta cada vez que llega una lectura nueva del LiDAR. Aquí se encadenan todos los pasos anteriores (incluyendo la burbuja de seguridad del Paso 4) y se decide la velocidad (Paso 7), y se publica el comando (Paso 8). Es básicamente el "director" que llama a todo lo demás en orden.
- **`odom_callback`** — Este es independiente del pipeline de LiDAR. Se explica en la sección 4 (contador de vueltas y cronómetro).
- **`main`** — Levanta el nodo de ROS2 y lo deja corriendo.

En cuanto a tópicos, el nodo usa:

- `/scan` (`sensor_msgs/LaserScan`) — entrada, lectura del LiDAR.
- `/ego_racecar/odom` (`nav_msgs/Odometry`) — entrada, posición del carro (para contar vueltas).
- `/drive` (`ackermann_msgs/AckermannDriveStamped`) — salida, el comando de manejo.

---

## 3. Cómo instalar todo desde cero

Vas a necesitar Ubuntu 22.04 con ROS2 Humble ya instalado y funcionando. Con eso listo, sigue estos pasos en orden.

**Primero, instala el F1TENTH Gym** (es la librería de física que usa el simulador):

```bash
cd $HOME
git clone https://github.com/f1tenth/f1tenth_gym
sudo apt install python3-pip
cd f1tenth_gym && pip3 install -e .
```

**Luego, clona este repositorio** (ya trae el controlador listo):

```bash
cd $HOME
git clone https://github.com/Giobapena/f1tenth-follow-the-gap-brandshatch-VNT.git F1Tenth-Repository
```

**Después, instala las dependencias de ROS2:**

```bash
cd ~/F1Tenth-Repository
sudo apt install python3-rosdep2
rosdep update
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y
```

**Y compila el workspace:**

```bash
colcon build
```

**Por último, descarga y configura el mapa de Brands Hatch:**

```bash
cd ~/F1Tenth-Repository/src/f1tenth_gym_ros/maps
git clone https://github.com/f1tenth/f1tenth_racetracks.git
mkdir -p BrandsHatch
cp f1tenth_racetracks/BrandsHatch/BrandsHatch_map.yaml BrandsHatch/
cp f1tenth_racetracks/BrandsHatch/BrandsHatch_map.png BrandsHatch/
```

Ahora abre `src/f1tenth_gym_ros/config/sim.yaml` y cambia la ruta del mapa por la tuya:

```yaml
map_path: '/home/<tu_usuario>/F1Tenth-Repository/src/f1tenth_gym_ros/maps/BrandsHatch/BrandsHatch_map'
```

Con esto ya tienes todo instalado y configurado.

---

## 4. Cómo ejecutarlo

Necesitas dos terminales abiertas al mismo tiempo.

**En la primera terminal, levanta el simulador:**

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Esto abre RViz con el carro ya posicionado sobre la pista de Brands Hatch.

**En la segunda terminal, corre el controlador:**

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run controllers gap_node
```

En cuanto lo ejecutes, el carro debería empezar a moverse solo, navegando la pista sin que tengas que tocar nada.

---

## 5. Incorporación de un oponente y obstáculos en la pista

Para evaluar el comportamiento del controlador en un escenario más cercano a una carrera real, se añadió un vehículo oponente y obstáculos estáticos dentro del mapa de Brands Hatch.

Debido a limitaciones del simulador, únicamente fue posible trabajar con **un solo oponente simultáneamente**. La implementación se realizó mediante un nodo independiente llamado `opp_node`, mientras que el controlador principal del vehículo autónomo se mantiene dentro de `gap_node`.

La estructura final de los nodos es:

```
src/controllers/controllers/
│
├── gap_node.py      # Controlador Follow The Gap del vehículo autónomo
│
└── opp_node.py      # Controlador del vehículo oponente
```

---

### 5.1 Controlador del vehículo autónomo (`gap_node.py`)

El archivo `gap_node.py` mantiene toda la lógica del algoritmo **Follow The Gap** descrita anteriormente.

Sus principales funciones son:

- Recibir las lecturas del LiDAR mediante `/scan`.
- Procesar los obstáculos detectados.
- Encontrar el espacio libre más grande (`maximum gap`).
- Calcular el punto objetivo dentro del espacio disponible.
- Publicar los comandos de dirección y velocidad mediante `/drive`.

El vehículo autónomo no tiene información directa del oponente, sino que lo interpreta como un obstáculo más debido a la información proporcionada por el LiDAR.

---

### 5.2 Controlador del oponente (`opp_node.py`)

El nodo `opp_node.py` fue creado como un controlador independiente encargado únicamente del movimiento del vehículo oponente.

A diferencia del vehículo principal, este nodo no utiliza Follow The Gap, sino una trayectoria y velocidad predefinidas para recorrer la pista.

Su objetivo es generar una condición de adelantamiento, por lo que la velocidad del oponente debe ser menor que la del vehículo autónomo.

Configuración utilizada:

| Vehículo | Nodo | Velocidad aproximada |
|----------|------|----------------------|
| Vehículo autónomo | `gap_node` | 7.0 m/s |
| Vehículo oponente | `opp_node` | 2.0 - 3.0 m/s |

La diferencia de velocidades permite que el vehículo autónomo alcance progresivamente al oponente y tenga que realizar una maniobra evasiva utilizando únicamente la información del LiDAR.

---

### 5.3 Comunicación entre nodos

El funcionamiento del escenario se basa en la siguiente interacción:

```
              ┌─────────────────┐
              │   opp_node.py   │
              │   Oponente      │
              └────────┬────────┘
                       │
                       ▼
              Vehículo detectado
              como obstáculo móvil
                       │
                       ▼
              ┌─────────────────┐
              │     LiDAR       │
              │    /scan        │
              └────────┬────────┘
                       │
                       ▼
              ┌─────────────────┐
              │   gap_node.py   │
              │ Follow The Gap  │
              └────────┬────────┘
                       │
                       ▼
              ┌─────────────────┐
              │     /drive      │
              │ Dirección +     │
              │ velocidad       │
              └─────────────────┘
```

El controlador principal mantiene su comportamiento reactivo, ya que no diferencia entre un obstáculo fijo y el vehículo oponente. Toda la decisión de adelantamiento se realiza mediante el procesamiento de los datos del LiDAR.

---

### 5.4 Creación de obstáculos mediante GIMP

Para aumentar la complejidad del circuito también se añadieron obstáculos estáticos utilizando el software **GIMP**.

El procedimiento realizado fue:

1. Abrir el mapa original de Brands Hatch:

```bash
BrandsHatch_map.png
```

2. Crear nuevas capas dentro de GIMP para agregar obstáculos sin modificar la imagen original.

3. Dibujar obstáculos en diferentes posiciones de la pista.

4. Mantener la misma resolución del mapa original para evitar problemas de escala dentro del simulador.

5. Exportar nuevamente el mapa modificado en formato `.png`.

6. Actualizar el archivo de configuración del simulador:

```yaml
map_path: '/home/<usuario>/F1Tenth-Repository/src/f1tenth_gym_ros/maps/BrandsHatch/BrandsHatch_map'
```

Los obstáculos fueron utilizados para probar la capacidad del controlador frente a:

- Reducción del espacio disponible.
- Cambios repentinos de trayectoria.
- Obstáculos durante zonas de alta velocidad.
- Maniobras evasivas antes de realizar un adelantamiento.

---

### 5.5 Ejecución con oponente y obstáculos

Para ejecutar la simulación completa se requieren tres terminales:

#### Terminal 1 — Simulador

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

---

#### Terminal 2 — Vehículo autónomo Follow The Gap

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run controllers gap_node
```

---

#### Terminal 3 — Vehículo oponente

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run controllers opp_node
```

El comportamiento esperado es:

1. El vehículo autónomo inicia la navegación utilizando Follow The Gap.
2. El oponente circula por delante a menor velocidad.
3. El LiDAR detecta al oponente y los obstáculos del circuito.
4. El algoritmo calcula nuevamente el espacio libre disponible.
5. El vehículo autónomo modifica su trayectoria para evitar colisiones y realizar el adelantamiento cuando existe suficiente espacio.


---

## 6. Si quieres ajustar el comportamiento

Todos estos valores están al inicio de la clase o dentro de `lidar_callback`, y los puedes tocar para adaptar el controlador a otra pista o a un estilo de manejo distinto:

| Parámetro | Qué controla | Valor que usé |
|---|---|---|
| `bubble_radius` | Qué tan grande es la zona de seguridad alrededor del obstáculo más cercano | `0.5` m |
| `car_half_width` | Medio ancho del carro (lo usa el disparity extender) | `0.25` m |
| `disparity_threshold` | Qué tan grande debe ser un salto entre rayos para considerarlo un borde | `0.25` m |
| `fov_deg` | Qué tan amplio es el cono de visión frontal | `85°` |
| `max_steering_angle` | Ángulo máximo de giro permitido | `34°` |
| `max_speed` | Velocidad en recta confirmada | `7.0` |
| `cruise_speed` | Velocidad de crucero en curvas | `4.5` |
| `min_speed` | Velocidad mínima en curvas cerradas | `1.5` |

Por ejemplo, si notas que el carro choca en curvas cerradas, puedes subir un poco `bubble_radius` o bajar `max_speed`. Si lo ves demasiado lento en rectas, puedes subir `max_speed` o relajar la condición de "recta confirmada" dentro de `lidar_callback`.

---

## Autor

Giovanny Baño — Proyecto de Vehículos no tripulados
