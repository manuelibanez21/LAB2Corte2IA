import numpy as np
import matplotlib.pyplot as plt
import random

num_drones = 5           # número de drones
num_flores = 30           # número de flores (puntos a polinizar)
dim = 2                   # espacio 2D (x, y)
max_iter = 100             # iteraciones del algoritmo ABC
limite = 5                # límite de intentos sin mejora (abandono de fuente)
bateria_max = 100         # batería máxima de cada drone

flores = np.random.rand(num_flores, dim) * 100
prioridades = np.random.randint(1, 5, size=num_flores)  

def fitness(drone_pos, bateria):    
    distancias = np.linalg.norm(flores - drone_pos, axis=1)
    eficiencia = np.sum(prioridades / (distancias + 1e-6))
    penalizacion = (bateria_max - bateria) / bateria_max
    return eficiencia - penalizacion

drones = np.random.rand(num_drones, dim) * 100
baterias = np.ones(num_drones) * bateria_max
mejor_drone = drones[0]
mejor_fit = -np.inf
limite_contador = np.zeros(num_drones)

for it in range(max_iter):
    for i in range(num_drones):
        k = random.choice([x for x in range(num_drones) if x != i])
        j = random.randint(0, dim-1)
        phi = random.uniform(-1, 1)
        candidato = drones[i].copy()
        candidato[j] = drones[i][j] + phi * (drones[i][j] - drones[k][j])
        candidato = np.clip(candidato, 0, 100)
        baterias[i] -= np.random.randint(1, 5)  
        # Evaluar fitness
        fit_actual = fitness(drones[i], baterias[i])
        fit_candidato = fitness(candidato, baterias[i])
        if fit_candidato > fit_actual:
            drones[i] = candidato
            limite_contador[i] = 0
        else:
            limite_contador[i] += 1
    
    fitness_vals = np.array([fitness(drones[i], baterias[i]) for i in range(num_drones)])
    prob = fitness_vals / (np.sum(fitness_vals) + 1e-6)
    
    for i in range(num_drones):
        if random.random() < prob[i]:
            k = random.choice(range(num_drones))
            j = random.randint(0, dim-1)
            candidato = drones[i].copy()
            candidato[j] = drones[i][j] + random.uniform(-1, 1) * (drones[i][j] - drones[k][j])
            candidato = np.clip(candidato, 0, 100)
            fit_candidato = fitness(candidato, baterias[i])
            if fit_candidato > fitness_vals[i]:
                drones[i] = candidato
    
    for i in range(num_drones):
        if limite_contador[i] >= limite:
            drones[i] = np.random.rand(dim) * 100
            baterias[i] = bateria_max
            limite_contador[i] = 0
    
    for i in range(num_drones):
        f = fitness(drones[i], baterias[i])
        if f > mejor_fit:
            mejor_fit = f
            mejor_drone = drones[i].copy()
    
    print(f"Iteración {it+1} - Mejor Fitness: {mejor_fit:.2f}")

plt.scatter(flores[:,0], flores[:,1], c=prioridades, cmap="autumn", s=100, marker="*")
plt.scatter(drones[:,0], drones[:,1], c="blue", label="Drones")
plt.scatter(mejor_drone[0], mejor_drone[1], c="green", marker="x", s=200, label="Mejor Drone")
plt.legend()
plt.title("Distribución de Drones y Flores (ABC)")
plt.show()
