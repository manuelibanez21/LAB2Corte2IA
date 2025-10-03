import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import random

NUM_HORMIGAS = 15     # hormigas
NUM_ITER = 50         # iteraciones
ALPHA = 1             # peso de feromona
BETA = 3              # mayor importancia a la heurística (distancia)
RHO = 0.5             # evaporación
Q = 100


N = 20 
G = nx.grid_2d_graph(N, N)
pos = dict((n, n) for n in G.nodes())


for (i, j) in G.edges():
    G.edges[i, j]['weight'] = random.randint(1, 20)

for (i, j) in list(G.edges()):
    if random.random() < 0.05:
        G.remove_edge(i, j)

pheromone = {edge: 1.0 for edge in G.edges()}


def probabilidad(i, vecinos, pheromone, alpha, beta):
    tau = np.array([pheromone.get((min(i, j), max(i, j)), 1.0) for j in vecinos])
    dist = np.array([G.edges[i, j]['weight'] for j in vecinos])
    eta = 1.0 / (dist + 1e-9)
    numerador = (tau ** alpha) * (eta ** beta)
    return numerador / numerador.sum()

def construir_ruta(start, end):
    ruta = [start]
    actual = start
    while actual != end:
        vecinos = list(G.neighbors(actual))
        vecinos = [v for v in vecinos if v not in ruta] or list(G.neighbors(actual))
        probs = probabilidad(actual, vecinos, pheromone, ALPHA, BETA)
        siguiente = random.choices(vecinos, weights=probs, k=1)[0]
        ruta.append(siguiente)
        actual = siguiente
        if len(ruta) > N * N:  # seguridad para evitar bucles eternos
            break
    return ruta

def longitud_ruta(ruta):
    return sum(G.edges[ruta[i], ruta[i+1]]['weight'] for i in range(len(ruta)-1) if G.has_edge(ruta[i], ruta[i+1]))

def actualizar_feromonas(rutas):
    global pheromone
    for edge in pheromone:
        pheromone[edge] *= (1 - RHO)
    for ruta in rutas:
        L = longitud_ruta(ruta)
        if L > 0:  # evitar división por cero
            deposito = Q / L
            for i in range(len(ruta)-1):
                if G.has_edge(ruta[i], ruta[i+1]):
                    edge = (min(ruta[i], ruta[i+1]), max(ruta[i], ruta[i+1]))
                    pheromone[edge] += deposito

start = (0, 0)
end = (N-1, N-1)

mejor_ruta = None
mejor_longitud = float('inf')
sin_mejora = 0 

for it in range(NUM_ITER):
    rutas = []
    for k in range(NUM_HORMIGAS):
        ruta = construir_ruta(start, end)
        rutas.append(ruta)
        L = longitud_ruta(ruta)
        if L < mejor_longitud and L > 0:
            mejor_ruta = ruta
            mejor_longitud = L
            sin_mejora = 0
    actualizar_feromonas(rutas)
    sin_mejora += 1
    print(f"Iter {it+1}: Mejor longitud = {mejor_longitud}")
    
    # Early stopping si no mejora en 20 iteraciones
    if sin_mejora >= 20:
        print("No mejora en 20 iteraciones, deteniendo...")
        break

print("Mejor ruta encontrada:", mejor_ruta)
print("Longitud:", mejor_longitud)

plt.figure(figsize=(8,8))
nx.draw(G, pos, node_size=20, node_color="lightblue", edge_color="gray")
if mejor_ruta:
    path_edges = [(mejor_ruta[i], mejor_ruta[i+1]) for i in range(len(mejor_ruta)-1) if G.has_edge(mejor_ruta[i], mejor_ruta[i+1])]
    nx.draw_networkx_edges(G, pos, edgelist=path_edges, width=2, edge_color='r')
plt.show()
