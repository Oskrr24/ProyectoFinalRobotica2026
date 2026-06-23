from controller import Robot
import csv
import heapq
import math
import os
from datetime import datetime




SCENARIO = "complex"   # solo informativo (usado en nombre de log y prints)

INFLATION_RADIUS_CELLS = 0

# ============================================================
# DEBUG DE NAVEGACIÓN
# ============================================================

DEBUG_NAV = True
DEBUG_PERIOD_S = 1.00

# Arena: RectangleArena floorSize 2 2 -> x,y en [-1.0, 1.0].
# Robot (S) en la posición inicial real del e-puck.
# Dog (G) arriba, al otro extremo del laberinto.
# Robot: yaw inicial ~= 1.5705 rad (~90° = mirando +y global, igual que en simple).
CONFIG = {
    # Calibrado desde MundoComplejo.wbt (arena real 2x2 m).
    # Grilla 20x20 celdas, cada celda mide 0.1 m (igual criterio que el mundo simple).
    "cell_size_m": 0.1,
    "map_min_x": -1.0,
    "map_max_y": 1.0,
    "start_world": (-0.650432, -0.650323),

    "goal_world": (-0.27, 0.76),

    "initial_heading_rad": 1.57053,

    "start": (3, 16),
    "goal": (7, 2),
    # Grilla 20×20, cada celda = 0.1 m. Fila 0 = arriba (y=+1.0), fila 19 = abajo (y=-1.0).
    # '#' = obstáculo/pared, '.' = libre, 'S' = inicio, 'G' = meta (perro).
    "grid": [
        "####################",
        "#...........#......#",
        "#......G....#......#",
        "#...........#......#",
        "#......#....#......#",
        "#.##...#....#......#",
        "###...####..#......#",
        "#.#....#....#......#",
        "#.....##..###......#",
        "#.#....#...##......#",
        "#...........########",
        "#.#................#",
        "########....########",
        "#..........#.......#",
        "#....######........#",
        "##.................#",
        "#..S...............#",
        "#..................#",
        "#..................#",
        "####################",
    ],
}


# ---------------------------------------------------------------------------
# Inflación de obstáculos
# ---------------------------------------------------------------------------

def inflate_grid(grid_lines, radius):
    """Devuelve un conjunto de celdas bloqueadas que incluye los obstáculos
    originales más un margen 'radius' celdas alrededor de cada uno."""
    height = len(grid_lines)
    width = len(grid_lines[0])
    original = set()
    for y, row in enumerate(grid_lines):
        for x, ch in enumerate(row):
            if ch == "#":
                original.add((x, y))

    if radius == 0:
        return original

    inflated = set(original)
    for (ox, oy) in original:
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                nx, ny = ox + dx, oy + dy
                if 0 <= nx < width and 0 <= ny < height:
                    inflated.add((nx, ny))
    return inflated

def print_grid_with_indices(grid):
    width = len(grid[0])

    print("    " + "".join(str(i % 10) for i in range(width)))
    print("   +" + "-" * width + "+")

    for row_index, row in enumerate(grid):
        print(f"{row_index:02d} |{row}|")

    print("   +" + "-" * width + "+")
    
# ---------------------------------------------------------------------------
# Planificador A* (8-conectado con distancia octile)
# ---------------------------------------------------------------------------

class AStarPlanner:
    SQRT2 = math.sqrt(2)

    def __init__(self, grid_lines, inflation_radius=1):
        self.grid_lines = grid_lines
        self.height = len(grid_lines)
        self.width = len(grid_lines[0])
        for y, row in enumerate(grid_lines):
            if len(row) != self.width:
                raise ValueError("Todas las filas de la grilla deben tener el mismo largo")
        self.occupied = inflate_grid(grid_lines, inflation_radius)

    def in_bounds(self, cell):
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free(self, cell):
        return self.in_bounds(cell) and cell not in self.occupied

    def neighbors(self, cell):
        """8 vecinos con costo 1 (cardinal) o sqrt(2) (diagonal)."""
        x, y = cell
        result = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nb = (x + dx, y + dy)
                if not self.is_free(nb):
                    continue
                # Evitar cortar esquinas: ambas celdas cardinales adyacentes
                # deben ser libres para un movimiento diagonal.
                if dx != 0 and dy != 0:
                    if not self.is_free((x + dx, y)) or not self.is_free((x, y + dy)):
                        continue
                cost = self.SQRT2 if (dx != 0 and dy != 0) else 1.0
                result.append((nb, cost))
        return result

    @staticmethod
    def heuristic(a, b):
        """Octile distance (admisible para 8-conectado)."""
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return max(dx, dy) + (AStarPlanner.SQRT2 - 1) * min(dx, dy)

    def plan(self, start, goal):
        # Si la celda inflada bloquea el inicio/meta, busca la celda libre más cercana.
        start = self._nearest_free(start)
        goal = self._nearest_free(goal)

        frontier = []
        heapq.heappush(frontier, (0.0, start))
        came_from = {start: None}
        cost_so_far = {start: 0.0}

        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal:
                break
            for next_cell, move_cost in self.neighbors(current):
                new_cost = cost_so_far[current] + move_cost
                if next_cell not in cost_so_far or new_cost < cost_so_far[next_cell]:
                    cost_so_far[next_cell] = new_cost
                    priority = new_cost + self.heuristic(next_cell, goal)
                    heapq.heappush(frontier, (priority, next_cell))
                    came_from[next_cell] = current

        if goal not in came_from:
            raise RuntimeError("A* no encontró ruta entre inicio y meta")

        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return simplify_path(path)

    def _nearest_free(self, cell):
        if self.is_free(cell):
            return cell
        # BFS para celda libre más cercana
        from collections import deque
        q = deque([cell])
        visited = {cell}
        while q:
            c = q.popleft()
            if self.is_free(c):
                print(f"[WARN] Celda {cell} bloqueada por inflación; usando {c} en su lugar.")
                return c
            x, y = c
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                nb = (x+dx, y+dy)
                if nb not in visited and self.in_bounds(nb):
                    visited.add(nb)
                    q.append(nb)
        raise RuntimeError(f"No se encontró celda libre cerca de {cell}")


# ---------------------------------------------------------------------------
# Simplificación de ruta (colinear merge)
# ---------------------------------------------------------------------------

def simplify_path(path):
    """Elimina puntos intermedios colineales (misma dirección)."""
    if len(path) <= 2:
        return path
    result = [path[0]]
    for i in range(1, len(path) - 1):
        dx_prev = path[i][0] - path[i-1][0]
        dy_prev = path[i][1] - path[i-1][1]
        dx_next = path[i+1][0] - path[i][0]
        dy_next = path[i+1][1] - path[i][1]
        # Cambia de dirección → es un vértice, conservar
        if (dx_prev, dy_prev) != (dx_next, dy_next):
            result.append(path[i])
    result.append(path[-1])
    return result


def path_length_in_cells(path):
    total = 0.0
    for prev, curr in zip(path, path[1:]):
        total += math.hypot(curr[0]-prev[0], curr[1]-prev[1])
    return total


# ---------------------------------------------------------------------------
# Utilidades de geometría / conversión
# ---------------------------------------------------------------------------

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def cell_to_local_xy(cell, start_cell, cell_size_m):
    dx_cells = cell[0] - start_cell[0]
    dy_cells = start_cell[1] - cell[1]
    return dx_cells * cell_size_m, dy_cells * cell_size_m


def cell_to_world_xy(cell, config):
    x = config["map_min_x"] + (cell[0] + 0.5) * config["cell_size_m"]
    y = config["map_max_y"] - (cell[1] + 0.5) * config["cell_size_m"]
    return x, y


def world_to_robot_local_xy(world_xy, start_world, initial_heading_rad):
    dx = world_xy[0] - start_world[0]
    dy = world_xy[1] - start_world[1]
    c = math.cos(initial_heading_rad)
    s = math.sin(initial_heading_rad)
    forward = c * dx + s * dy
    left = -s * dx + c * dy
    return forward, left


def cell_to_robot_local_xy(cell, config):
    world_xy = cell_to_world_xy(cell, config)
    return world_to_robot_local_xy(
        world_xy,
        config["start_world"],
        config["initial_heading_rad"],
    )
    
# ---------------------------------------------------------------------------
# Debug grilla / celdas / sensores
# ---------------------------------------------------------------------------

def debug_event(t, tag, msg):
    print(f"[{tag} t={t:7.2f}s] {msg}")


def world_to_cell(x, y, config):
    cell_size = config["cell_size_m"]

    col = int((x - config["map_min_x"]) // cell_size)
    row = int((config["map_max_y"] - y) // cell_size)

    width = len(config["grid"][0])
    height = len(config["grid"])

    col = max(0, min(width - 1, col))
    row = max(0, min(height - 1, row))

    return (col, row)


def robot_local_to_world_xy(local_xy, config):
    """
    Convierte odometría local del robot a coordenadas globales Webots.
    Usa la pose inicial start_world + initial_heading_rad.
    """
    forward, left = local_xy
    heading = config.get("initial_heading_rad", 0.0)

    c = math.cos(heading)
    s = math.sin(heading)

    dx_world = c * forward - s * left
    dy_world = s * forward + c * left

    sx, sy = config["start_world"]

    return (sx + dx_world, sy + dy_world)


def robot_local_to_cell(local_xy, config, start_cell=None):
    """
    Convierte la posición odométrica local x,y a celda de grilla.
    Para tu escenario simple usa start_world/map_min_x/map_max_y.
    """
    if "start_world" in config and "map_min_x" in config and "map_max_y" in config:
        wx, wy = robot_local_to_world_xy(local_xy, config)
        return world_to_cell(wx, wy, config)

    if start_cell is None:
        start_cell = config["start"]

    cell_size = config["cell_size_m"]

    col = int(round(start_cell[0] + local_xy[0] / cell_size))
    row = int(round(start_cell[1] - local_xy[1] / cell_size))

    width = len(config["grid"][0])
    height = len(config["grid"])

    col = max(0, min(width - 1, col))
    row = max(0, min(height - 1, row))

    return (col, row)


def grid_char_at(config, cell):
    col, row = int(round(cell[0])), int(round(cell[1]))

    if row < 0 or row >= len(config["grid"]):
        return "OUT"
    if col < 0 or col >= len(config["grid"][0]):
        return "OUT"

    return config["grid"][row][col]


def cell_debug_status(cell, planner, config):
    """
    Devuelve estado legible de la celda:
    '.', '#', 'S', 'G', 'inflated' u 'OUT'.
    """
    cell = (int(round(cell[0])), int(round(cell[1])))

    if not planner.in_bounds(cell):
        return "OUT"

    if cell in planner.occupied:
        original = grid_char_at(config, cell)
        if original == "#":
            return "#"
        return "inflated"

    original = grid_char_at(config, cell)

    if original in ("S", "G"):
        return original

    return "."


def validate_planned_path(path_cells, planner, config):
    """
    Verifica al inicio si A* está entregando una ruta por celdas libres.
    """
    print("=== Validación de ruta planificada ===")

    bad = []

    for i, cell in enumerate(path_cells):
        status = cell_debug_status(cell, planner, config)

        if "map_min_x" in config:
            world_center = cell_to_world_xy(cell, config)
        else:
            world_center = "N/A"

        print(
            f"  wp[{i:02d}] celda={cell} "
            f"estado={status} centro_world={world_center}"
        )

        if not planner.is_free(cell):
            bad.append((i, cell, status))

    if bad:
        print("[ERROR] La ruta contiene celdas bloqueadas:", bad)
    else:
        print("[OK] La ruta planificada sólo usa celdas libres.")


def print_debug_grid_snapshot(config, path_cells, current_cell=None, target_cell=None):
    """
    Imprime una grilla con:
    o = ruta
    R = robot
    T = target actual
    F = celdas delante del robot
    """
    overlay = [list(row) for row in config["grid"]]

    for cell in path_cells:
        col, row = cell
        if 0 <= row < len(overlay) and 0 <= col < len(overlay[0]):
            if overlay[row][col] == ".":
                overlay[row][col] = "o"

    if target_cell is not None:
        col, row = target_cell
        if 0 <= row < len(overlay) and 0 <= col < len(overlay[0]):
            overlay[row][col] = "T"

    if current_cell is not None:
        col, row = current_cell
        if 0 <= row < len(overlay) and 0 <= col < len(overlay[0]):
            overlay[row][col] = "R"

    print_grid_with_indices(["".join(row) for row in overlay])


def wheel_speeds_from_unicycle(v, omega, wheel_radius, axle_length, max_wheel_speed):
    left  = (v - omega * axle_length / 2.0) / wheel_radius
    right = (v + omega * axle_length / 2.0) / wheel_radius
    # Escalar proporcionalmente si se supera el límite (sin recortar asimétricamente)
    peak = max(abs(left), abs(right))
    if peak > max_wheel_speed:
        left  *= max_wheel_speed / peak
        right *= max_wheel_speed / peak
    return left, right


# ---------------------------------------------------------------------------
# Log CSV
# ---------------------------------------------------------------------------

def open_log_file(name):
    folder = os.path.dirname(os.path.abspath(__file__))
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"registro_{name}_{timestamp}.csv"
    path = os.path.join(folder, filename)
    file = open(path, "w", newline="")
    writer = csv.writer(file)
    writer.writerow([
        "time", "x_odom_m", "y_odom_m", "theta_rad",
        "x_real_m", "y_real_m", "theta_real_rad",
        "pos_error_m", "heading_error_real_rad",
        "target_index", "target_x_m", "target_y_m",
        "ps0", "ps1", "ps2", "ps5", "ps6", "ps7",
        "ps0_raw", "ps1_raw", "ps2_raw", "ps5_raw", "ps6_raw", "ps7_raw",
        "planned_distance_m", "executed_distance_m", "distance_difference_m",
        "almost_collisions", "unnecessary_turns", "state",
    ])
    return file, writer, path


# ---------------------------------------------------------------------------
# Bucle principal
# ---------------------------------------------------------------------------

def run_robot():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    config        = CONFIG

    cell_size_m   = config["cell_size_m"]
    start_cell    = config["start"]
    goal_cell     = config["goal"]
    

    # --- Planificación ---
    planner    = AStarPlanner(config["grid"], inflation_radius=INFLATION_RADIUS_CELLS)
    path_cells = planner.plan(start_cell, goal_cell)

    if "start_world" in config:
        waypoints = [cell_to_robot_local_xy(c, config) for c in path_cells]
    else:
        waypoints = [cell_to_local_xy(c, start_cell, cell_size_m) for c in path_cells]

    planned_length_m = path_length_in_cells(path_cells) * cell_size_m

    # Detección automática de segmentos cortos: cualquier tramo entre dos
    # vértices consecutivos de A* que mida menos de SHORT_SEGMENT_THRESHOLD_M
    # suele aparecer en zigzags estrechos del laberinto, rodeado de giros
    # agudos. A velocidad de crucero normal el robot no llega a estabilizar
    # el ángulo antes de pasarse de largo ese waypoint. Se guarda el índice
    # del waypoint DESTINO de cada segmento corto, para frenar sólo ahí
    # (sin afectar el resto del recorrido), sin necesidad de hardcodear
    # índices a mano cada vez que cambia la grilla o el plan resultante.
    SHORT_SEGMENT_THRESHOLD_M = 0.15
    short_segment_targets = set()
    for i in range(1, len(waypoints)):
        seg_len = math.hypot(
            waypoints[i][0] - waypoints[i - 1][0],
            waypoints[i][1] - waypoints[i - 1][1],
        )
        if seg_len < SHORT_SEGMENT_THRESHOLD_M:
            short_segment_targets.add(i)

    # --- Parámetros físicos e-puck ---
    wheel_radius    = 0.0205
    axle_length     = 0.052
    max_wheel_speed = 6.28   # rad/s (Webots e-puck max ~6.28)

    # --- Parámetros de seguimiento ---
    cruise_speed      = 0.035   # m/s velocidad normal
    slow_speed        = 0.02    # m/s al girar mucho o acercarse
    k_angle           = 4.5     # ganancia proporcional angular
    max_omega         = 3.25    # rad/s límite de giro
    angle_deadband    = 0.04    # rad: zona muerta para reducir oscilación en recto
    slow_angle_thresh = 0.5     # rad: ángulo a partir del cual se reduce velocidad

    # Tolerancias de waypoint (metros)
    waypoint_tol      = 0.04
    goal_tol          = 0.05

    # Frenado: reducir velocidad cuando faltan menos de brake_dist al waypoint final
    brake_dist        = 0.20

    # --- Umbrales de sensores de proximidad (raw ADC) ---
    # ps0=delantero-derecha, ps7=delantero-izquierda,
    # ps1=lateral-derecha,   ps6=lateral-izquierda,
    # ps2=trasero-derecha,   ps5=trasero-izquierda
    FRONT_SOFT   = 80    # Primer aviso: reducir velocidad
    FRONT_HARD   = 150   # Emergencia: parar y girar
    SIDE_SOFT    = 120    # Corrección lateral suave
    SIDE_HARD    = 250    # Corrección lateral agresiva

    # --- Filtro EMA (Exponential Moving Average) de sensores de proximidad ---
    # filtrado[k] = alpha * crudo[k] + (1 - alpha) * filtrado[k-1]
    # alpha cercano a 1.0 -> casi sin filtrar (reacciona rápido, ruidoso)
    # alpha cercano a 0.0 -> muy suavizado (estable, pero reacciona más lento)
    PS_EMA_ALPHA = 0.6

    # --- Detección de bloqueo ---
    stuck_timeout     = 3.0    # segundos sin avance → maniobra escape
    stuck_dist_thresh = 0.01   # metros: si no avanzó esto en stuck_timeout, está bloqueado

    # --- Inicializar dispositivos ---
    left_motor  = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    left_enc  = left_motor.getPositionSensor()
    right_enc = right_motor.getPositionSensor()
    left_enc.enable(timestep)
    right_enc.enable(timestep)

    ps_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
    ps = []
    for name in ps_names:
        s = robot.getDevice(name)
        s.enable(timestep)
        ps.append(s)

    # GPS y Compass: SOLO para registrar la posición/orientación reales y
    # poder calcular el error de odometría (PDF, Sección 10). 
    gps = robot.getDevice("gps")
    compass = robot.getDevice("compass")
    gps_compass_available = (gps is not None) and (compass is not None)
    if gps_compass_available:
        gps.enable(timestep)
        compass.enable(timestep)
    else:
        print("[WARN] gps/compass no encontrados en el robot: "
              "el error de odometría no se podrá calcular (se registrará vacío).")

    log_file, log_writer, log_path = open_log_file(SCENARIO)

    # --- Estado del filtro EMA de sensores ---
    ps_filtered = None  # se inicializa con la primera lectura cruda (evita arranque en 0)

    # --- Estado odométrico ---
    x = 0.0
    y = 0.0
    theta = 0.0
    executed_distance = 0.0
    prev_left  = None
    prev_right = None

    # --- Estado de control ---
    target_index    = 1 if len(waypoints) > 1 else 0
    reached_goal    = False
    last_log_time   = -1.0
    almost_collisions = 0
    was_in_emergency  = False

    # --- Conteo de giros innecesarios ---
    # Un "giro innecesario" se cuenta cuando el signo de omega se invierte
    # (izquierda <-> derecha) dentro de una ventana corta de tiempo durante
    # seguimiento normal de ruta (no durante escape/emergencia, donde el
    # cambio de signo es intencional). 
    unnecessary_turns      = 0
    last_omega_sign        = 0
    last_omega_sign_time   = -999.0
    OSCILLATION_WINDOW_S   = 0.6   # cambios de signo más rápidos que esto cuentan
    OMEGA_SIGN_DEADBAND    = 0.3   # rad/s: ignora omega muy pequeño (ruido cerca de 0)

    # --- Anti-bloqueo ---
    last_progress_dist = 0.0
    last_progress_time = 0.0
    escape_until       = -1.0   # tiempo hasta el que ejecutar maniobra escape
    
    # --- Estado debug ---
    last_debug_time = -999.0
    last_debug_cell = None
    last_debug_target_index = None
    last_debug_state = None
    last_grid_issue_cell = None
    
    
    # ---- MOSTRAR INFORMACIÓN DE ESCENARIO Y DEBUG ----
    print("=== Controlador A* v2 ===")
    print(f"Escenario       : {SCENARIO}")
    print(f"Inflación       : {INFLATION_RADIUS_CELLS} celda(s)")
    print(f"Ruta (celdas)   : {path_cells}")
    print(f"Waypoints (m)   : {[(round(a,3), round(b,3)) for a,b in waypoints]}")
    print(f"Longitud plan.  : {planned_length_m:.3f} m")
    print(f"Log CSV         : {log_path}")
    print_grid_with_indices(config["grid"])
    
    if DEBUG_NAV:
        validate_planned_path(path_cells, planner, config)
        print_debug_grid_snapshot(
            config,
            path_cells,
            current_cell=start_cell,
            target_cell=path_cells[target_index],
        )

    while robot.step(timestep) != -1:
        t = robot.getTime()

        # ---- Lectura de sensores (cruda + filtrado EMA) ----
        ps_raw = [s.getValue() for s in ps]
        if ps_filtered is None:
            ps_filtered = list(ps_raw)  # arranque: sin historial previo, usa la cruda
        else:
            ps_filtered = [
                PS_EMA_ALPHA * raw + (1.0 - PS_EMA_ALPHA) * prev
                for raw, prev in zip(ps_raw, ps_filtered)
            ]
        ps_val = ps_filtered  # todo el control de aquí en adelante usa el valor filtrado
        # Frontales: ps7 (izq), ps0 (der)
        front_left_val  = ps_val[7]
        front_right_val = ps_val[0]
        # Laterales: ps6 (izq), ps1 (der)
        side_left_val   = ps_val[6]
        side_right_val  = ps_val[1]
        # Diagonales frontales adicionales: ps5 y ps2 (más angulados)
        diag_left_val   = ps_val[5]
        diag_right_val  = ps_val[2]

        front_max = max(front_left_val, front_right_val)

        # ---- Odometría ----
        lp = left_enc.getValue()
        rp = right_enc.getValue()
        if prev_left is None:
            prev_left, prev_right = lp, rp

        dl = wheel_radius * (lp - prev_left)
        dr = wheel_radius * (rp - prev_right)
        prev_left, prev_right = lp, rp

        ds     = (dl + dr) / 2.0
        dtheta = (dr - dl) / axle_length
        if not math.isfinite(ds):     ds     = 0.0
        if not math.isfinite(dtheta): dtheta = 0.0

        x     += ds * math.cos(theta + dtheta / 2.0)
        y     += ds * math.sin(theta + dtheta / 2.0)
        theta  = normalize_angle(theta + dtheta)
        executed_distance += abs(ds)
        
        # ---- Debug: celda actual estimada desde odometría ----
        current_cell = robot_local_to_cell((x, y), config, start_cell)

        # ---- Fin de misión ----
        if reached_goal:
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            continue

        # ---- Avance hacia waypoint ----
  
        last_progress_time = t
        last_progress_dist = executed_distance

        target_x, target_y = waypoints[target_index]
        target_cell = path_cells[target_index]
        
        dx_wp = target_x - x
        dy_wp = target_y - y
        dist_wp = math.hypot(dx_wp, dy_wp)

        is_last = (target_index == len(waypoints) - 1)
        tol = goal_tol if is_last else waypoint_tol

        # Detección por proyección sobre el segmento (sólo para waypoints
        # intermedios, no para la meta final): si el robot ya cruzó el plano
        # perpendicular al segmento [waypoint_anterior -> waypoint_actual]
        # que pasa por waypoint_actual, se considera "alcanzado" aunque nunca
        # haya entrado en el radio `tol`. 
        passed_waypoint = False
        if not is_last:
            prev_x, prev_y = waypoints[target_index - 1]
            seg_dx = target_x - prev_x
            seg_dy = target_y - prev_y
            seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy
            if seg_len_sq > 1e-9:
                # Producto escalar entre la dirección del segmento y el
                # vector (waypoint actual -> robot). Si es positivo, el
                # robot ya quedó "delante" del waypoint en esa dirección,
                # es decir, ya lo pasó.
                advance_dot = seg_dx * (x - target_x) + seg_dy * (y - target_y)
                if advance_dot > 0:
                    passed_waypoint = True

        if dist_wp < tol or passed_waypoint:
            if is_last:
                reached_goal = True
                left_motor.setVelocity(0.0)
                right_motor.setVelocity(0.0)
                print("✓ Meta alcanzada")
                print(f"  Tiempo total       : {t:.2f} s")
                print(f"  Ruta planificada   : {planned_length_m:.3f} m")
                print(f"  Trayectoria real   : {executed_distance:.3f} m")
                print(f"  Casi-colisiones    : {almost_collisions}")
                print(f"  Giros innecesarios : {unnecessary_turns}")
                log_file.flush()
                continue
            target_index += 1
            last_progress_time = t
            last_progress_dist = executed_distance
            target_x, target_y = waypoints[target_index]
            dx_wp = target_x - x
            dy_wp = target_y - y
            dist_wp = math.hypot(dx_wp, dy_wp)
            is_last = (target_index == len(waypoints) - 1)

        # ---- Control seguimiento de ruta ----
        desired_heading = math.atan2(dy_wp, dx_wp)
        angle_err = normalize_angle(desired_heading - theta)

        # Zona muerta angular: pequeños errores no generan corrección
        if abs(angle_err) < angle_deadband:
            angle_err = 0.0

        # Velocidad lineal base
        if abs(angle_err) > slow_angle_thresh:
            v_base = slow_speed
        elif is_last and dist_wp < brake_dist:
            # Frenado progresivo en el tramo final
            v_base = slow_speed + (cruise_speed - slow_speed) * (dist_wp / brake_dist)
        else:
            v_base = cruise_speed

        # Frenado automático: si el waypoint actual es destino de un
        # segmento corto (detectado al planificar, ver short_segment_targets),
        # se fuerza slow_speed sólo mientras se dirige a ESE waypoint
        # puntual, sin afectar el resto del recorrido.
        if target_index in short_segment_targets:
            v_base = min(v_base, slow_speed)

        omega_base = max(min(k_angle * angle_err, max_omega), -max_omega)
        state = "follow_path"

        # ---- Evasión local reactiva ----
        # Se mantiene el plan global; la evasión sólo modifica omega/v temporalmente.
        if t < escape_until:
            # Maniobra de escape activa (anti-bloqueo)
            state = "escape"
            v_base = -0.04
            omega_base = 3.0   # giro hacia la izquierda mientras retrocede

        elif front_max > FRONT_HARD:
            # Emergencia: parar y girar hacia el lado más despejado
            if not was_in_emergency:
                almost_collisions += 1
                was_in_emergency = True
            state = "emergency_turn"
            v_base = 0.0
            # Girar hacia el lado que tenga sensor frontal menor (más espacio)
            turn_dir = -1.0 if front_left_val > front_right_val else 1.0
            omega_base = turn_dir * 4.0

        elif front_max > FRONT_SOFT:
            # Obstáculo frontal: frenar + girar suave
            state = "front_avoidance"
            was_in_emergency = False
            v_base = 0.01
            turn_dir = -1.0 if front_left_val > front_right_val else 1.0
            # Mezcla con el omega del seguidor para no perder la dirección global
            avoidance_omega = turn_dir * 3.0
            blend = min((front_max - FRONT_SOFT) / (FRONT_HARD - FRONT_SOFT), 1.0)
            omega_base = (1.0 - blend) * omega_base + blend * avoidance_omega

        else:
            was_in_emergency = False
        
            # Correcciones laterales suaves, solo si no hay conflicto frontal
            if side_left_val > SIDE_HARD or diag_left_val > SIDE_HARD:
                state = "side_left_hard"
                omega_base -= 2.6
        
            elif side_left_val > SIDE_SOFT:
                state = "side_left_soft"
                omega_base -= 1.4
        
            elif side_right_val > SIDE_HARD or diag_right_val > SIDE_HARD:
                state = "side_right_hard"
                omega_base += 2.6
        
            elif side_right_val > SIDE_SOFT:
                state = "side_right_soft"
                omega_base += 1.4
                
        # ---- Debug runtime: celdas, transiciones y bloqueos ----
        if DEBUG_NAV:

            current_cell_status = cell_debug_status(current_cell, planner, config)
            target_cell_status = cell_debug_status(target_cell, planner, config)
        
            # 1) Transición de celda
            if current_cell != last_debug_cell:
                prev = "None" if last_debug_cell is None else str(last_debug_cell)
        
                debug_event(
                    t,
                    "CELDA",
                    f"{prev} -> {current_cell} "
                    f"estado={current_cell_status} "
                    f"odom=({x:.3f},{y:.3f}) theta={theta:.2f}"
                )
        
                last_debug_cell = current_cell
        
            # 2) Cambio de waypoint objetivo
            if target_index != last_debug_target_index:
                debug_event(
                    t,
                    "TARGET",
                    f"target_index={target_index}/{len(path_cells)-1}, "
                    f"target_cell={target_cell}, "
                    f"target_estado={target_cell_status}, "
                    f"target_local=({target_x:.3f},{target_y:.3f}), "
                    f"dist={dist_wp:.3f}m"
                )
        
                last_debug_target_index = target_index
        
            # 3) Aviso si la celda actual está bloqueada según la grilla
            if not planner.is_free(current_cell):
                if current_cell != last_grid_issue_cell:
                    debug_event(
                        t,
                        "GRID",
                        f"Robot estimado en celda bloqueada {current_cell}, "
                        f"estado={current_cell_status}. "
                        "Revisar grilla, start_world, initial_heading_rad u odometría."
                    )
        
                    last_grid_issue_cell = current_cell
        
            # 4) Estado periódico
            if (t - last_debug_time) >= DEBUG_PERIOD_S or state != last_debug_state:
                debug_event(
                    t,
                    "STATUS",
                    f"cell={current_cell} "
                    f"cell_estado={current_cell_status} "
                    f"target={target_cell} "
                    f"target_estado={target_cell_status} "
                    f"idx={target_index}/{len(path_cells)-1} "
                    f"dist={dist_wp:.3f}m "
                    f"angle_err={angle_err:.2f} "
                    f"state={state}"
                )
        
                last_debug_time = t
                last_debug_state = state
    
        # ---- Detección de bloqueo ----
        if state not in ("escape", "emergency_turn"):
            delta_since_check = executed_distance - last_progress_dist
            time_since_check  = t - last_progress_time
            if time_since_check > stuck_timeout:
                if delta_since_check < stuck_dist_thresh:
                    print(f"[t={t:.1f}s] Bloqueo detectado → maniobra escape")
                    escape_until = t + 1.5   # 1.5 s de escape
                last_progress_time = t
                last_progress_dist = executed_distance

        # ---- Clamp final de omega ----
        omega_base = max(min(omega_base, max_omega), -max_omega)

        # ---- Conteo de giros innecesarios ----
      
        if state not in ("escape", "emergency_turn"):
            if abs(omega_base) > OMEGA_SIGN_DEADBAND:
                current_sign = 1 if omega_base > 0 else -1
                if (
                    last_omega_sign != 0
                    and current_sign != last_omega_sign
                    and (t - last_omega_sign_time) < OSCILLATION_WINDOW_S
                ):
                    unnecessary_turns += 1
                last_omega_sign = current_sign
                last_omega_sign_time = t

        # ---- Convertir a velocidades de rueda ----
        left_spd, right_spd = wheel_speeds_from_unicycle(
            v_base, omega_base, wheel_radius, axle_length, max_wheel_speed
        )
        left_motor.setVelocity(left_spd)
        right_motor.setVelocity(right_spd)

        # ---- Log ----
        if t - last_log_time >= 0.25:
            if gps_compass_available:
                gx, gy, _gz = gps.getValues()
                cx, cy, _cz = compass.getValues()
                theta_real = math.atan2(cx, cy)
                pos_error = math.hypot(x - gx, y - gy)
                heading_error_real = normalize_angle(theta - theta_real)
                x_real_str, y_real_str = f"{gx:.4f}", f"{gy:.4f}"
                theta_real_str = f"{theta_real:.4f}"
                pos_error_str = f"{pos_error:.4f}"
                heading_error_real_str = f"{heading_error_real:.4f}"
            else:
                x_real_str = y_real_str = theta_real_str = ""
                pos_error_str = heading_error_real_str = ""

            distance_difference = executed_distance - planned_length_m

            log_writer.writerow([
                f"{t:.3f}",
                f"{x:.4f}", f"{y:.4f}", f"{theta:.4f}",
                x_real_str, y_real_str, theta_real_str,
                pos_error_str, heading_error_real_str,
                target_index,
                f"{target_x:.4f}", f"{target_y:.4f}",
                f"{ps_val[0]:.1f}", f"{ps_val[1]:.1f}",
                f"{ps_val[2]:.1f}", f"{ps_val[5]:.1f}",
                f"{ps_val[6]:.1f}", f"{ps_val[7]:.1f}",
                f"{ps_raw[0]:.1f}", f"{ps_raw[1]:.1f}",
                f"{ps_raw[2]:.1f}", f"{ps_raw[5]:.1f}",
                f"{ps_raw[6]:.1f}", f"{ps_raw[7]:.1f}",
                f"{planned_length_m:.4f}",
                f"{executed_distance:.4f}",
                f"{distance_difference:.4f}",
                almost_collisions,
                unnecessary_turns,
                state,
            ])
            log_file.flush()
            last_log_time = t

    log_file.close()


if __name__ == "__main__":
    run_robot()