# 🧠 Optimización de Enjambres y Colonias --- ABC, ACO y PSO con Drones

Este repositorio contiene tres simulaciones inspiradas en algoritmos de
inteligencia colectiva aplicados a escenarios con drones o agentes
autónomos.\
Cada archivo implementa un método distinto de optimización bioinspirada:

1.  **`PSOdronespunto1.py`** → *Optimización por Enjambre de Partículas
    (PSO)*\
2.  **`ACOpunto2.py`** → *Optimización por Colonia de Hormigas (ACO)*\
3.  **`ABCpunto3.py`** → *Algoritmo de Colonia de Abejas (ABC)*

------------------------------------------------------------------------

## 🐝 1. ABCpunto3.py --- *Algoritmo de Colonia de Abejas (Artificial Bee Colony)*

### 🎯 Objetivo

Optimizar la ubicación de un conjunto de drones para maximizar la
eficiencia de polinización de flores distribuidas aleatoriamente en un
área 2D, considerando la prioridad de cada flor y el consumo de batería.

### ⚙️ Funcionamiento

1.  **Inicialización:**

    -   Se generan `num_flores` puntos aleatorios (flores) con
        prioridades (1 a 4).
    -   Se colocan `num_drones` drones en posiciones aleatorias con
        batería máxima (`bateria_max`).

2.  **Función de fitness:** \[ `\text{fitness}`{=tex} =
    `\sum `{=tex}`\frac{\text{prioridad}}{\text{distancia}}`{=tex} -
    `\text{penalización\_batería}`{=tex} \] Cuanto más cerca esté un
    dron de flores prioritarias y con mayor batería, mejor su desempeño.

3.  **Fases del algoritmo:**

    -   **Empleadas:** cada dron explora una vecindad aleatoria y decide
        si mejora su posición.
    -   **Observadoras:** seleccionan soluciones buenas con probabilidad
        proporcional a su fitness.
    -   **Exploradoras:** si un dron no mejora durante cierto tiempo
        (`limite`), se reposiciona aleatoriamente (abandona la fuente de
        néctar).

4.  **Actualización del mejor resultado:**

    -   Se guarda el dron con mayor fitness global.
    -   Se imprime la evolución del mejor valor por iteración.

5.  **Visualización:**

    -   Flores representadas como estrellas (`*`), coloreadas según
        prioridad.
    -   Drones en azul.
    -   El mejor dron en verde (`x`).

### ▶️ Ejecución

``` bash
python ABCpunto3.py
```

### 📊 Resultado

Se genera una gráfica mostrando la distribución final de los drones y el
mejor punto de exploración alcanzado.

------------------------------------------------------------------------

## 🐜 2. ACOpunto2.py --- *Optimización por Colonia de Hormigas (Ant Colony Optimization)*

### 🎯 Objetivo

Encontrar el camino más corto entre dos puntos en una malla
bidimensional utilizando feromonas para guiar la exploración de las
hormigas.

### ⚙️ Funcionamiento

1.  **Inicialización:**
    -   Se crea una **rejilla 20x20** como grafo (usando `networkx`).
    -   Cada arista tiene un peso (distancia) aleatorio entre 1 y 20.
    -   Algunas conexiones se eliminan aleatoriamente (simulando
        obstáculos).
2.  **Feromonas:**
    -   Cada arista inicia con un nivel de feromona = 1.
    -   A medida que las hormigas encuentran caminos más cortos,
        depositan más feromonas.
3.  **Selección probabilística:**
    -   Cada hormiga decide su siguiente paso según: \[ P\_{ij} =
        `\frac{(\tau_{ij})^{\alpha} \cdot (\eta_{ij})^{\beta}}{\sum_k (\tau_{ik})^{\alpha} \cdot (\eta_{ik})^{\beta}}`{=tex}
        \] donde
        -   (`\tau`{=tex}\_{ij}): nivel de feromona,\
        -   (`\eta`{=tex}\_{ij}): heurística inversa a la distancia.
4.  **Evaporación y actualización:**
    -   Las feromonas se evaporan con una tasa `RHO`.
    -   Las rutas más eficientes reciben más depósito de feromona
        proporcional a ( Q / L ).
5.  **Criterio de parada:**
    -   Se ejecutan `NUM_ITER` iteraciones o se detiene si no hay mejora
        en 20 iteraciones.
6.  **Visualización:**
    -   La red se dibuja en celeste, y la mejor ruta encontrada se
        resalta en rojo.

### ▶️ Ejecución

``` bash
python ACOpunto2.py
```

### 📊 Resultado

Muestra en consola la mejor longitud encontrada y dibuja el camino más
corto entre los extremos de la cuadrícula.

------------------------------------------------------------------------

## 🚁 3. PSOdronespunto1.py --- *Optimización por Enjambre de Partículas (Particle Swarm Optimization)*

### 🎯 Objetivo

Coordinar un grupo de drones para formar patrones 3D (estrella, robot o
dragón), evitando colisiones y minimizando energía y distancia.

### ⚙️ Funcionamiento

1.  **Inicialización del enjambre:**
    -   Cada dron tiene una posición y velocidad aleatoria dentro de los
        límites `[−10, 10]`.
    -   Los objetivos (targets) se definen según la figura elegida:
        -   `star`, `robot`, o `dragon`.
2.  **Función de costo (fitness_swarm):** Evalúa:
    -   **Distancia promedio a los objetivos asignados.**
    -   **Consumo de energía (magnitud de velocidad).**
    -   **Penalización por colisiones o cercanía (\<0.6m).**
    -   **Penalización por obstáculos.**
3.  **Actualización PSO:**
    -   Velocidad: \[ v_i = w v_i + c_1 r_1 (p_i - x_i) + c_2 r_2 (g -
        x_i) \]
    -   Posición: \[ x_i = x_i + v_i \]
    -   Se mantiene la velocidad máxima (`max_vel`) y los drones dentro
        de los límites.
4.  **Simulación:**
    -   Se ejecutan múltiples iteraciones (`simulate()`).
    -   Puede incluir fallos aleatorios de drones (`fail_rate`) o
        aparición de nuevos obstáculos.
5.  **Visualización animada:**
    -   Se crea una animación con `matplotlib.animation.FuncAnimation`
        donde los drones se mueven para formar la figura deseada.
6.  **Salida:**
    -   Guarda la animación como `pso_drones_robot.gif` (si `ffmpeg`
        está instalado).
    -   También imprime métricas de la simulación (coste, colisiones,
        energía).

### ▶️ Ejecución

``` bash
python PSOdronespunto1.py
```

### 📊 Resultado

Una animación donde los drones se organizan dinámicamente para formar
una figura tridimensional, adaptándose a fallos y obstáculos.

------------------------------------------------------------------------

## 🧩 Dependencias

Instala las librerías necesarias:

``` bash
pip install numpy matplotlib networkx
```

------------------------------------------------------------------------

## 📘 Comparación General

  -------------------------------------------------------------------------
  Algoritmo    Inspiración Biológica  Objetivo Principal   Representación
  ------------ ---------------------- -------------------- ----------------
  **PSO**      Enjambre de aves o     Coordinación de      Animación
               peces                  drones y formación   dinámica
                                      de patrones 3D       

  **ACO**      Colonia de hormigas    Búsqueda de rutas    Grafo en 2D
                                      más cortas           

  **ABC**      Colonia de abejas      Exploración          Mapa de flores y
                                      eficiente de puntos  drones
                                      de interés           
  -------------------------------------------------------------------------
