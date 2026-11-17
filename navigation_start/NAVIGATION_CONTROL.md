# Navegación ROS 2

`navigation_manager` ofrece la Action `navigate` (`navigation_manager/action/Navigate`).
Los cuatro launch AMCL/SLAM ejecutan un único coordinador con `simple_move`,
`path_planner`, `map_augmenter` y `potential_fields`.

El objetivo puede ser una pose (`POSE`) o una localización (`LOCATION`); estas últimas
se resuelven mediante `/locations_manager/get`. La pose resuelta permanece inmutable.
`move_head` se aplica atómicamente en `simple_move` y se comprueba mediante lectura
antes de despachar cualquier trayectoria. La percepción de obstáculos permanece activa.

El contexto pertenece a un objetivo y cada operación lleva su propia generación.
La cancelación y sustitución invalidan respuestas pendientes, confirman que el controlador
está inactivo y verifican reposo por odometría fresca antes de cerrar la ejecución.
Un fallo de verificación de parada impide activar objetivos posteriores hasta reiniciar
el coordinador y comprobar físicamente el estado del robot.

El monitor diagnostica geometría, percepción, localización y falta de progreso.
Las estrategias seleccionan observación, espera o replanificación; los presupuestos
por estrategia y condición se configuran en `navigation_manager/config/navigation_manager.yaml`.
El desplazamiento físico reinicia presupuestos del episodio; el tiempo total sigue limitado.
Las maniobras locales, colisión, escape acotado y frenado pertenecen a `simple_move`.
El éxito requiere pose final, tolerancia angular, seguridad y reposo, también cuando
el planificador utiliza un destino alternativo dentro del radio permitido.

Los clientes antiguos y las configuraciones de RViz publican poses en `/nav_control/goal`.
`legacy_move_head` define su política de cabeza. `motion_node` identifica el nodo del
controlador para aplicar y verificar su configuración (por defecto, `simple_move` en
el espacio de nombres de navegación).
`status` y `path` se conservan como salidas de compatibilidad y visualización.
Los nuevos clientes deben usar la Action para cancelación y resultados correlacionados.

Pruebas:

```bash
colcon build --packages-select path_planner map_augmenter potential_fields simple_move navigation_manager navigation_start --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ROS_LOG_DIR=/tmp/navigation_ros_logs ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=173 colcon test --packages-select navigation_manager navigation_start simple_move path_planner
colcon test-result --verbose
```

Las pruebas de Action utilizan servidor y controlador reales con una base cinemática,
servicios controlados e inyección de fallos. No certifican comportamiento en hardware.
