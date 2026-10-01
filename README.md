# mi_solucion_dock

Solución para el Create 3 Dock Challenge (HRFEST 2026). El robot localiza el
dock y se acopla usando únicamente el LiDAR (`/scan`), sin la acción `/dock`,
sin sensores IR y sin ground truth.

## Equipo

| Nombre | Correo |
|---|---|
| Oliver Beizaga | olibeizaga@hotmail.com |
| Rafael Neciosup | rafaelnv2002@gmail.com |
| Sergio Ortiz | sergio100899@gmail.com |

## Requisitos

- Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11
- Paquete `create3_dock_challenge`
- NumPy (`python3-numpy`)

## Instalación

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
# Terminal 1: escenario
ros2 run create3_dock_challenge clean_sim.sh
ros2 launch create3_dock_challenge challenge_world.launch.py

# Terminal 2: solución
ros2 launch mi_solucion_dock solucion.launch.py
```

El nodo inicia sin intervención y detiene el robot cuando `/dock_status`
reporta `is_docked: true`.

Pose inicial distinta:

```bash
ros2 launch create3_dock_challenge challenge_world.launch.py x:=0.9 y:=0.7 yaw:=-1.2
```

## Depuración

El nodo `scan_points` ejecuta la detección y el filtrado sin publicar en
`/cmd_vel`:

```bash
ros2 run mi_solucion_dock scan_points --ros-args -p use_sim_time:=true
```

Ambos nodos publican marcadores en `/debug/scan_points` (Fixed Frame `odom`):

| Color | Contenido |
|---|---|
| Cian | Puntos del scan en `base_link` |
| Naranja | Paredes detectadas |
| Magenta | Puntos de las cajas |
| Amarillo | Eje del dock del scan actual |
| Verde | Eje del dock filtrado |

## Estructura

| Archivo | Contenido |
|---|---|
| `detector.py` | Paredes (RANSAC), cajas y eje del dock |
| `filtro.py` | Estimación del eje en `odom` |
| `control.py` | Máquina de estados y perfil de velocidad |
| `scan_points.py` | Nodo de percepción |
| `dock_lidar.py` | Nodo principal: percepción y control |
| `launch/solucion.launch.py` | Comando de lanzamiento |

`detector.py`, `filtro.py` y `control.py` no dependen de ROS. Los parámetros
están definidos como constantes al inicio de cada archivo.

Interfaces utilizadas:

| Interfaz | Uso |
|---|---|
| `/scan` | Percepción |
| `/tf`, `/tf_static` | `base_link → laser_link` y `odom → base_link` |
| `/dock_status` | Condición de parada |
| `/cmd_vel` | Comando de velocidad |

## Algoritmo

### Detección

1. Los puntos del scan se transforman a `base_link` mediante la TF del LiDAR,
   lo que compensa el montaje girado 180° y desplazado.
2. Las paredes se obtienen con RANSAC: se toman dos puntos al azar, se cuentan
   los puntos a menos de 1.5 cm de la recta que definen y se conserva la recta
   con más puntos. Se eliminan esos puntos y se repite, hasta cuatro paredes.
3. Para cada pared se seleccionan los puntos que sobresalen entre 5 y 11 cm
   (las cajas sobresalen 8 cm), se agrupan a lo largo de la pared y se busca un
   par de grupos de ~8 cm de ancho separados 17.5 cm entre centros.
4. El eje del dock pasa por el punto medio entre las dos cajas,
   perpendicular a la pared.

La separación entre rayos crece con la distancia, por lo que las tolerancias de
ancho y separación se escalan con ella. El alcance de detección es de ~5 m.

### Filtrado

La estimación del eje se mantiene en el frame `odom`:

- Media exponencial con peso 0.3 para cada detección nueva.
- Se descartan detecciones a más de 10 cm o 10° de la estimación; tres
  detecciones consecutivas coherentes reemplazan la estimación.
- El control se habilita a partir de tres detecciones.
- Sin detecciones, la última estimación se mantiene durante 1.5 s.

### Control

| Estado | Comportamiento |
|---|---|
| Búsqueda | Sin estimación del dock: desplazamiento hacia el centro de la sala (calculado a partir de las paredes) y giro en el sitio. |
| Preposición | Con desplazamiento lateral grande cerca de la pared: desplazamiento previo a un punto del eje más alejado. |
| Aproximación | Seguimiento del eje con pure pursuit. La distancia de mira se reduce al acercarse a la pared. |
| Acople | A partir de 40 cm de la pared, a 4 cm/s. Un error de orientación mayor a 4° provoca una retirada. |
| Retirada | Retroceso hasta 55 cm de la pared y nuevo intento. También se activa al llegar a 25 cm sin acople. |
| Acoplado | `is_docked: true`: velocidad cero. |

La velocidad lineal se escala con `cos²` del error de orientación, de modo que
el robot gira y avanza simultáneamente.

### Perfil de velocidad

Perfil trapezoidal recalculado en cada ciclo a partir de la distancia medida:

```
v = min(V_MAX, √(V_ACOPLE² + 2·A_FRENO·(d − D_ACOPLE)))
```

- Aceleración limitada a 0.4 m/s².
- Velocidad de crucero de 0.30 m/s.
- Frenado hasta 4 cm/s al inicio del tramo de acople (40 cm de la pared).
- Velocidad y aceleración angulares limitadas a 1.5 rad/s y 3 rad/s².

### Distancias

El código no define una distancia objetivo de acople; la parada depende de
`/dock_status`. La única distancia fija es el límite de seguridad
`D_MIN = radio del robot + saliente de las cajas = 16.95 + 8 ≈ 25 cm`.