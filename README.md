# mi_solucion_dock

Solución para el Create 3 Dock Challenge (HRFEST 2026). El robot encuentra el
dock y se acopla usando solo el LiDAR, sin la acción `/dock`, sin los sensores
IR y sin ground truth.

## Equipo

| Nombre | Correo |
|---|---|
| _completar_ | _completar_ |

## Instalación

Probado en Ubuntu 22.04 con ROS 2 Humble y Gazebo Classic 11. Además de lo que
ya instala el reto, solo hace falta NumPy.

```bash
mkdir -p ~/sim_ws/src && cd ~/sim_ws/src
git clone https://github.com/Kalman-Robotics/create3_dock_challenge.git
git clone https://github.com/sergio100899/mi_solucion_dock_hrfest.git mi_solucion_dock

cd ~/sim_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Ejecución

```bash
# Terminal 1: escenario del reto
ros2 run create3_dock_challenge clean_sim.sh
ros2 launch create3_dock_challenge challenge_world.launch.py

# Terminal 2: la solución
ros2 launch mi_solucion_dock solucion.launch.py
```

El nodo arranca solo y se detiene cuando `/dock_status` marca `is_docked: true`.
Para probar otras poses iniciales:

```bash
ros2 launch create3_dock_challenge challenge_world.launch.py x:=0.9 y:=0.7 yaw:=-1.2
```

También está el nodo `scan_points`, que hace toda la detección pero no mueve
el robot. Lo usé para depurar moviendo el robot con `teleop_twist_keyboard`:

```bash
ros2 run mi_solucion_dock scan_points --ros-args -p use_sim_time:=true
```

Los dos nodos publican marcadores en `/debug/scan_points` para verlos en RViz
(con Fixed Frame en `odom`): puntos del scan en cian, paredes en naranja, cajas
en magenta, el eje detectado en amarillo y el eje filtrado en verde.

## Cómo funciona

El código está en `mi_solucion_dock/`. La lógica está en tres módulos que no
dependen de ROS (`detector.py`, `filtro.py` y `control.py`), y `dock_lidar.py`
es el nodo que los junta: lee `/scan` y las TF, y publica en `/cmd_vel`.

### Detección

1. Los puntos del scan se pasan a `base_link` con la TF del LiDAR. Así se
   corrigen de una vez el giro de 180° y el desplazamiento del sensor.
2. Las paredes se encuentran con RANSAC: se eligen dos puntos al azar, se
   cuentan los que quedan a menos de 1.5 cm de esa recta y se repite hasta
   quedarse con la mejor. Luego se quitan esos puntos y se busca la siguiente.
3. Frente a cada pared se buscan puntos que sobresalgan entre 5 y 11 cm (las
   cajas sobresalen 8). Se agrupan a lo largo de la pared y se busca un par de
   grupos de unos 8 cm de ancho con 17.5 cm entre centros. Si aparece, el eje
   del dock pasa por el punto medio entre las dos cajas, perpendicular a la
   pared.

De lejos los rayos llegan más separados y el ancho medido de las cajas sale más
corto, así que las tolerancias crecen con la separación entre rayos. Con esto
detecta el marcador hasta unos 5 m. En pruebas con la sala sin cajas no dio
falsos positivos.

### Filtrado

La posición del dock se guarda en el frame `odom`, donde no debería moverse.
Cada detección nueva se promedia con la anterior (media exponencial, peso 0.3)
y se descartan las que saltan más de 10 cm o 10°. El control solo empieza
después de tres detecciones, y si se pierde el marcador sigue con la última
estimación durante 1.5 s.

### Control

Es una máquina de estados:

- **Búsqueda**: si no ve el dock, avanza hacia el centro de la sala (calculado
  con las paredes detectadas) y gira. Desde ahí el marcador se detecta bien.
- **Preposición**: si está muy de lado y cerca de la pared, primero se coloca
  en un punto del eje más alejado para no rozar las cajas.
- **Aproximación**: sigue el eje con pure pursuit, apuntando a un punto del eje
  que queda por delante. Esa distancia de mira se acorta al acercarse, así que
  el robot termina centrado.
- **Acople**: los últimos centímetros a 4 cm/s, y solo avanza si está casi
  perpendicular a la pared.
- **Retirada**: si llega a 25 cm de la pared sin acoplar, retrocede y lo
  vuelve a intentar.

El robot gira y avanza a la vez: la velocidad lineal baja de forma suave con el
error de ángulo (factor cos²) en lugar de pararse para girar.

La velocidad sigue un perfil trapezoidal calculado en cada ciclo: sube con una
aceleración limitada hasta 0.30 m/s y frena con `v = √(2·a·d)`, de forma que
llega al tramo de acople (34 cm de la pared) ya a velocidad lenta. Si entra
rápido se pasa del eje y llega torcido, y como `is_docked` se activa en cuanto
el ángulo baja de 6°, eso se nota en la precisión.

No hay una distancia objetivo en el código: el robot avanza hasta que
`/dock_status` confirma el acople. La única distancia fija es el límite de
seguridad de 25 cm, que sale del radio del robot (16.95 cm) más lo que
sobresalen las cajas (8 cm).

Todos los parámetros están como constantes al principio de cada archivo.

## Resultados

Ajusté los parámetros con un simulador cinemático propio (no incluido) que
reproduce la sala, las cajas y el montaje del LiDAR, y corre el mismo código de
detección, filtrado y control. En 100 poses aleatorias dentro del rango de
evaluación acopló en todas, sin golpes, en unos 9 s de media (13 s como
máximo). El error final quedó por debajo de 2 mm en lateral y de 3° en ángulo.

## Limitaciones

- Casi todo el ajuste se hizo en el simulador cinemático, que no modela la
  rampa del dock ni la dinámica real del Create 3. Los resultados en Gazebo
  pueden variar, sobre todo en el tramo final.
- Entre los 25 cm del límite de seguridad y los ~26.8 cm a partir de los que
  engancha hay muy poco margen. Si el robot entra en retirada una y otra vez,
  lo primero sería bajar `V_ACOPLE`.
- A más de 5 m de la pared del dock no detecta el marcador y depende de la
  búsqueda para acercarse.
- No hay tests automáticos de la lógica.
