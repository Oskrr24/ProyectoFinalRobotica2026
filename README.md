# Navegación autónoma de un robot e-puck mediante A*

Proyecto final de **Robótica y Sistemas Autónomos - ICI 4150**.

## Integrantes

- **Integrante 1:** Oscar Ruiz
- **Integrante 2:** Andrés García
- **Integrante 3:** Milovan Fuentes
- **Integrante 4:** Marcos Cádiz
- **Integrante 5:** Amaro Fibla

## Línea seleccionada

**Línea A: planificación de rutas.**

El proyecto implementa un sistema de navegación autónoma para un robot diferencial e-puck en Webots. El robot calcula una ruta global mediante A* sobre una grilla de ocupación, transforma la ruta en puntos intermedios y utiliza odometría y sensores de proximidad para seguirla, corregir desviaciones y evitar obstáculos.

## Objetivo

Diseñar, implementar y evaluar un sistema capaz de:

- Desplazarse autónomamente desde una posición inicial hasta una meta
- Calcular una ruta global considerando los obstáculos conocidos
- Seguir puntos intermedios mediante control cinemático diferencial
- Estimar el movimiento utilizando encoders y odometría
- Detectar obstáculos mediante sensores de proximidad
- Realizar correcciones locales sin abandonar el objetivo global
- Detenerse al alcanzar la meta
- Registrar datos para analizar el desempeño

## Tecnologías utilizadas

- **Simulador:** Webots R2025a
- **Robot:** GCtronic e-puck
- **Lenguaje:** Python
- **Planificación global:** A*
- **Mapa:** grilla de ocupación 2D de 20 × 20 celdas
- **Resolución:** 0,1 m por celda
- **Sensores:** ocho sensores infrarrojos de proximidad
- **Estimación:** encoders y odometría diferencial
- **Filtrado:** media móvil exponencial, EMA
- **Control:** seguimiento proporcional de orientación y velocidades diferenciales

## Estructura del proyecto

```
ProyectoFinalRobotica/
├── controllers/
│   ├── e-puckSimple/
│   │   ├── e-puckSimple.py
│   │   └── registro_simple_*.csv
│   └── e-puckComplejo/
│       ├── e-puckComplejo.py
│       └── registro_complex_*.csv
├── worlds/
│   ├── MundoSimple.wbt
│   ├── MundoComplejo.wbt
│   ├── .MundoSimple.jpg
│   └── .MundoComplejo.jpg
└── README.md
```

Los controladores y mundos principales son:

- `MundoSimple.wbt`
- `e-puckSimple.py`
- `MundoComplejo.wbt`
- `e-puckComplejo.py`

## Robot, sensores y actuadores

El e-puck es un robot móvil de accionamiento diferencial. El controlador utiliza:

- Motor de la rueda izquierda;
- Motor de la rueda derecha;
- Encoder de la rueda izquierda;
- Encoder de la rueda derecha;
- Sensores de proximidad `ps0` a `ps7`.

Los sensores frontales permiten detectar riesgo de colisión. Los sensores laterales y diagonales se emplean para corregir la trayectoria dentro de pasillos y alrededor de obstáculos.

Parámetros físicos utilizados:

| Parámetro | Valor |
| --- | --- |
| Radio de rueda | 0,0205 m |
| Distancia entre ruedas | 0,052 m |
| Velocidad máxima de rueda | 6,28 rad/s |

## Escenarios de prueba

### Escenario simple

El escenario simple contiene una cantidad reducida de obstáculos y una ruta relativamente directa. Permite comprobar el funcionamiento básico de A*, la conversión a waypoints, la odometría y el seguimiento de la ruta.

Mundo simple

Características:

- Arena de 2 × 2 m
- Grilla de ocupación de 20 × 20
- Obstáculos estáticos y paredes
- Robot e-puck controlado por `e-puckSimple`
- Meta representada visualmente en el mundo
- Ruta calculada automáticamente por A*, sin waypoints manuales

### Escenario complejo

El escenario complejo incorpora más paredes, obstáculos, curvas, pasillos estrechos y zonas que exigen correcciones locales.

Mundo complejo

Características:

- Arena de 2 × 2 m
- Grilla de ocupación de 20 × 20
- Mayor densidad de obstáculos
- Pasillos estrechos y cambios de orientación
- Robot e-puck controlado por `e-puckComplejo`
- Meta representada visualmente por el perro
- Ruta calculada automáticamente por A*

## Representación del entorno

Cada escenario se representa mediante una grilla de ocupación:

| Símbolo | Significado |
| --- | --- |
| `#` | Celda ocupada |
| `.` | Celda libre |
| `S` | Posición inicial |
| `G` | Meta |

La grilla utiliza un sistema de coordenadas discreto. Cada celda se transforma posteriormente a coordenadas métricas del mundo y, finalmente, al marco local inicial del robot.

## Planificación global mediante A*

El planificador A* utiliza una vecindad de ocho movimientos:

- Movimientos horizontales y verticales con costo 1
- Movimientos diagonales con costo √2
- Prohibición de cortar esquinas ocupadas
- Heurística de distancia octil

La función de evaluación es:

```
f(n) = g(n) + h(n)
```

donde:

- `g(n)` es el costo acumulado desde el inicio;
- `h(n)` es la estimación del costo restante hasta la meta.

Después de encontrar la ruta, se eliminan puntos colineales innecesarios. Las celdas restantes se convierten en waypoints métricos que el robot sigue secuencialmente.

No se introducen waypoints manuales: en ambos escenarios la ruta utilizada proviene del planificador A*.

## Control cinemático

El controlador calcula una velocidad lineal `v` y una velocidad angular `ω`. Estas se convierten en velocidades de rueda mediante:

```
ω_izquierda = (v - ωL/2) / r
ω_derecha   = (v + ωL/2) / r
```

donde:

- `r` es el radio de las ruedas;
- `L` es la distancia entre ruedas.

El error angular respecto del waypoint se calcula con:

```
error = atan2(y_objetivo - y, x_objetivo - x) - θ
```

Se aplica un control proporcional:

```
ω = Kp × error
```

También se utiliza:

- zona muerta angular para reducir oscilaciones;
- reducción de velocidad durante giros;
- frenado progresivo cerca de la meta;
- tolerancia para considerar alcanzado cada waypoint;
- límite proporcional de las velocidades de rueda.

## Odometría

Los encoders permiten calcular el desplazamiento de cada rueda:

```
Δs_izquierda = r × Δθ_izquierda
Δs_derecha   = r × Δθ_derecha
```

El desplazamiento y cambio de orientación del robot se estiman como:

```
Δs = (Δs_derecha + Δs_izquierda) / 2
Δφ = (Δs_derecha - Δs_izquierda) / L
```

La pose se actualiza mediante:

```
x = x + Δs × cos(φ + Δφ/2)
y = y + Δs × sin(φ + Δφ/2)
φ = φ + Δφ
```

Esta estimación se utiliza para medir la trayectoria ejecutada y calcular la dirección hacia los waypoints.

## Filtrado EMA

Para reducir variaciones rápidas en los sensores de proximidad se utiliza una media móvil exponencial:

```
valor_filtrado[k] =
    α × valor_crudo[k] +
    (1 - α) × valor_filtrado[k-1]
```

El filtro reduce ruido sin requerir almacenar una ventana completa de mediciones. Las lecturas filtradas se utilizan para las correcciones suaves. En las decisiones de emergencia puede conservarse la lectura cruda para evitar retrasar la reacción ante un obstáculo repentino.

El controlador complejo utiliza un valor de `α = 0,6`, escogido para conservar una respuesta rápida en esquinas y pasillos estrechos.

## Navegación local y prevención de colisiones

La navegación local complementa a A*. El planificador determina la ruta global, mientras que los sensores modifican temporalmente el movimiento:

- **Seguimiento normal:** avance hacia el waypoint
- **Obstáculo frontal suave:** reducción de velocidad y giro gradual
- **Riesgo frontal alto:** detención lineal y giro hacia el lado más despejado
- **Obstáculo lateral:** corrección angular para separarse de la pared
- **Bloqueo:** retroceso y giro durante una maniobra de escape

Después de una corrección local, el robot vuelve a orientarse hacia el waypoint global.

## Detección de bloqueo

El controlador compara periódicamente la distancia odométrica ejecutada. Si el robot avanza menos que el umbral configurado durante la ventana de tiempo establecida, activa una maniobra de escape:

1. Retrocede
2. Gira
3. Vuelve a intentar alcanzar el waypoint

Esta lógica evita que el robot permanezca indefinidamente detenido contra una pared u obstáculo.

## Relación con los laboratorios

### Laboratorio 1

El proyecto reutiliza y extiende:

- Cinemática diferencial
- Control independiente de las ruedas
- Movimiento recto y curvo
- Rotación del robot
- Conversión entre velocidad lineal/angular y velocidades de rueda
- Seguimiento de trayectorias.

### Laboratorio 2

El proyecto reutiliza y extiende:

- Lectura de sensores de proximidad
- Encoders
- Odometría
- Filtrado de mediciones
- Navegación reactiva
- Prevención de colisiones.

La principal extensión consiste en combinar estas capacidades locales con una estrategia global basada en A*.

## Registro de datos

Cada ejecución genera un archivo CSV dentro de la carpeta del controlador correspondiente.

Los registros incluyen, según la versión del controlador:

- Tiempo de simulación;
- Posición y orientación estimadas;
- Índice y coordenadas del waypoint;
- Sensores de proximidad;
- Sensores crudos y filtrados;
- Distancia ejecutada;
- Estado de navegación;
- Longitud planificada;
- Diferencia entre ruta planificada y ejecutada;
- Cantidad de casi colisiones;
- Confirmación de llegada.

Estados posibles:

| Estado | Descripción |
| --- | --- |
| `follow_path` | Seguimiento normal |
| `front_avoidance` | Evasión frontal suave |
| `emergency_turn` | Giro de emergencia |
| `side_left_soft/hard` | Corrección por obstáculo izquierdo |
| `side_right_soft/hard` | Corrección por obstáculo derecho |
| `escape` | Maniobra de desbloqueo |
| `goal` | Meta alcanzada |

## Resultados

### Escenario simple

Se registraron cinco ejecuciones completas con estado final `goal`.

| Métrica | Resultado observado |
| --- | --- |
| Ejecuciones completas registradas | 5 |
| Tiempo promedio de llegada | 26,14 s |
| Trayectoria ejecutada promedio | 1,53 m |
| Casi colisiones en las ejecuciones completas | 0 |
| Resultado | Meta alcanzada |

El tiempo de las ejecuciones exitosas estuvo aproximadamente entre 25,06 y 26,82 segundos. Las diferencias entre longitud planificada y ejecutada se explican por la tolerancia de llegada, la simplificación de la ruta, la odometría y las correcciones locales.

### Escenario complejo

La ruta planificada por A* tiene una longitud aproximada de **2,11 m**. Las ejecuciones completas observadas se encuentran alrededor de 70 segundos y aproximadamente 2,0 m de trayectoria odométrica ejecutada.

El escenario requiere más correcciones que el simple debido a:

- Mayor cantidad de obstáculos
- Giros consecutivos
- Pasillos estrechos
- Mayor sensibilidad al error odométrico
- Necesidad de reducir la velocidad en segmentos cortos

La llegada se informa en la consola mediante:

```
Meta alcanzada
Tiempo total
Ruta planificada
Trayectoria real
Casi-colisiones
```

## Comparación de escenarios

| Característica | Simple | Complejo |
| --- | --- | --- |
| Cantidad de obstáculos | Baja/media | Alta |
| Longitud de ruta | Menor | Mayor |
| Pasillos estrechos | Pocos | Varios |
| Cambios de orientación | Moderados | Frecuentes |
| Tiempo de ejecución | Aproximadamente 26 s | Aproximadamente 70 s |
| Uso de evasión local | Ocasional | Más frecuente |
| Resultado | Meta alcanzada | Meta alcanzada |

## Métricas consideradas

- Tiempo total hasta la meta
- Longitud de la ruta planificada
- Longitud aproximada de la trayectoria ejecutada
- Diferencia entre ruta planificada y trayectoria real
- Cantidad de casi colisiones
- Posición y orientación estimadas por odometría
- Comportamiento de sensores crudos y filtrados
- Porcentaje de ejecuciones exitosas
- Estados de navegación activados durante el recorrido.

## Cómo ejecutar la simulación

### Requisitos

1. Instalar Webots R2025a.
2. Descargar o clonar este repositorio.
3. Mantener la estructura de carpetas `worlds/` y `controllers/`.

### Mundo simple

1. Abrir Webots.
2. Seleccionar **File > Open World**.
3. Abrir `worlds/MundoSimple.wbt`.
4. Verificar que el e-puck tenga asignado el controlador `e-puckSimple`.
5. Reiniciar la simulación para restablecer la pose inicial.
6. Ejecutar la simulación.
7. Esperar el mensaje `Meta alcanzada` en la consola.
8. Revisar el CSV generado en `controllers/e-puckSimple/`.

### Mundo complejo

1. Abrir `worlds/MundoComplejo.wbt`.
2. Verificar que el e-puck tenga asignado el controlador `e-puckComplejo`.
3. Reiniciar la simulación.
4. Ejecutar la simulación.
5. Esperar el mensaje `Meta alcanzada` en la consola.
6. Revisar el CSV generado en `controllers/e-puckComplejo/`.

## Evidencias

- Captura del mundo simple: `worlds/.MundoSimple.jpg`
- Captura del mundo complejo: `worlds/.MundoComplejo.jpg`
- Registros simples: `controllers/e-puckSimple/`
- Registros complejos: `controllers/e-puckComplejo/`

## Limitaciones

- La odometría acumula error porque integra los desplazamientos de las ruedas.
- El mapa es conocido previamente y no se actualiza durante la navegación.
- La evasión local puede separar temporalmente al robot de la ruta ideal.
- Los umbrales de proximidad dependen del escenario y de la velocidad.
- El filtro EMA introduce un compromiso entre suavizado y rapidez de respuesta.
- No se utiliza localización absoluta mediante GPS, supervisor o cámara.
- La ruta se planifica una vez; no se ejecuta una replanificación global completa ante cambios dinámicos.

## Posibles mejoras

- Replanificación A* cuando una zona permanezca bloqueada.
- Comparación automática entre trayectoria planificada y ejecutada.
- Gráficos generados desde los CSV.
- Medición automática del error final de posición.
- Ajuste adaptativo del filtro EMA.
- Inflación dinámica de obstáculos según el radio del robot.
- Uso de LiDAR o cámara.
- Fusión sensorial o filtro de Kalman.
- Pruebas automatizadas con múltiples posiciones iniciales.

## Conclusiones

El proyecto integra planificación global, control cinemático, percepción y estimación de movimiento en un sistema de navegación autónoma para un robot diferencial.

A* permite encontrar una ruta válida desde el inicio hasta la meta en ambos escenarios. La ruta se transforma en waypoints que el robot sigue mediante control proporcional. Los sensores de proximidad complementan la planificación con evasión local, mientras que los encoders permiten estimar la trayectoria mediante odometría.

Las pruebas muestran que el robot puede alcanzar la meta tanto en el escenario simple como en el complejo. El escenario complejo requiere más tiempo y correcciones debido a sus pasillos estrechos y mayor densidad de obstáculos. El sistema cumple el objetivo de demostrar navegación autónoma con propósito global y prevención local de colisiones.
