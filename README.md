# ProyectoFinalRobotica2026
Este repositorio guarda el Proyecto Final de Robótica y sistemas autónomos (ICI4150-2) de la PUCV

## Integrantes
Oscar Ruiz
Milovan Fuentes
Andrés García
Amaro Fibla
Marcos Cádiz.

## Línea seleccionada
Línea A: planificación de rutas con A* sobre grilla de ocupación.

## Objetivo
Diseñar e implementar un sistema de navegación autónoma para un robot diferencial e-puck en Webots.

## Relación con Laboratorios 1 y 2
- Lab 1: control diferencial, velocidades de ruedas, giros y seguimiento.
- Lab 2: sensores de proximidad, encoders, odometría y evasión reactiva.

## Robot y sensores utilizados
- Robot: e-puck.
- Actuadores: motores diferenciales izquierdo y derecho.
- Sensores: ps0-ps7.
- Encoders: sensores de posición de rueda.

## Escenarios de prueba
### Escenario simple
Descripción, imagen y grilla.

### Escenario complejo
Descripción, imagen y grilla.

## Algoritmo implementado
Explicación de:
- grilla de ocupación;
- inflación de obstáculos;
- A* 8-conectado;
- heurística octile;
- conversión de ruta a waypoints;
- seguimiento de puntos;
- evasión local.

## Pseudocódigo
1. Cargar escenario.
2. Crear grilla.
3. Ejecutar A*.
4. Convertir ruta a waypoints.
5. Leer encoders y sensores.
6. Estimar posición por odometría.
7. Seguir waypoint actual.
8. Si hay obstáculo, aplicar evasión local.
9. Registrar datos.
10. Detenerse al llegar a la meta.

## Resultados
Tabla con métricas.

## Gráficos
Trayectorias, sensores y comparación entre escenarios.

## Video demostrativo

## Instrucciones de ejecución
1. Abrir Webots.
2. Cargar mundo simple o complejo.
3. Seleccionar controlador Python.
4. Cambiar `SCENARIO = "simple"` o `"complex"`.
5. Ejecutar simulación.

## Conclusiones
Qué funcionó, qué limitaciones hubo y mejoras posibles.
