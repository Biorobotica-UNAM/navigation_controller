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
Un fallo de verificación de parada bloquea las solicitudes de movimiento. El servicio
`reset_fault` (`std_srvs/srv/Trigger`, en el mismo namespace que `navigate`) solicita
una nueva parada y verifica una ventana de odometría fresca tras la confirmación de
reposo del controlador. `success=true` indica que comenzó la verificación. El tópico
`navigation_status` informa el resultado de la verificación y el bloqueo efectivo
en `faulted`. El restablecimiento no reactiva metas anteriores.

El monitor diagnostica geometría, percepción, localización y falta de progreso.
Las estrategias seleccionan observación, espera o replanificación; los presupuestos
por estrategia y condición se configuran en `navigation_manager/config/navigation_manager.yaml`.
El desplazamiento físico reinicia presupuestos del episodio; el tiempo total sigue limitado.
Las maniobras locales, colisión, escape acotado y frenado pertenecen a `simple_move`.
El éxito requiere pose final, tolerancia angular, seguridad y reposo, también cuando
el planificador utiliza un destino alternativo dentro del radio permitido.

El coordinador recibe poses en `/navigation/goal` y mantiene `/nav_control/goal`
para los clientes y configuraciones de RViz existentes. `legacy_move_head` define
la política de cabeza para ambos tópicos. El parámetro `motion_node` identifica el
nodo del controlador. Su valor por defecto es `simple_move_node`, relativo al
namespace del coordinador. Los launch lo configuran como `simple_move`.

`navigation_status` (`navigation_manager/msg/NavigationStatus`) publica el estado
inicial, el progreso y el resultado de metas de ambos orígenes. Usa QoS Reliable,
Transient Local y Keep Last con profundidad 1. Durante una ejecución se actualiza
a 5 Hz y ante cambios relevantes. La última instantánea queda disponible para
suscriptores tardíos. `result_valid` indica si contiene un resultado terminal y
`source` identifica el origen de la solicitud. Con la configuración por defecto
de los launch, el tópico es `/navigation/navigation_status`. Se conserva
`status` (`actionlib_msgs/msg/GoalStatus`) para consumidores existentes y
`path` para visualización.

Pruebas:

```bash
colcon build --packages-select path_planner map_augmenter potential_fields simple_move navigation_manager navigation_start --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ROS_LOG_DIR=/tmp/navigation_ros_logs ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=173 colcon test --packages-select navigation_manager navigation_start simple_move path_planner
colcon test-result --verbose
```

Las pruebas de Action utilizan servidor y controlador reales con una base cinemática,
servicios controlados e inyección de fallos. No certifican comportamiento en hardware.
