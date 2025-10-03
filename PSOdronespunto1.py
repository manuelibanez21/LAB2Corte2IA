import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
import random

def normalize(v):
    norm = np.linalg.norm(v)
    return v / (norm + 1e-9)

def pairwise_distances(X):
    diffs = X[:, None, :] - X[None, :, :]
    return np.sqrt((diffs**2).sum(axis=2))

def target_star(n_points=20, radius=5.0):
    angles = np.linspace(0, 2*np.pi, 11)[:-1]
    outer = [(np.cos(a), np.sin(a)) for a in angles[::2]]
    inner = [(0.5*np.cos(a+0.2), 0.5*np.sin(a+0.2)) for a in angles[1::2]]
    pts2d = []
    for i in range(5):
        pts2d.append(outer[i])
        pts2d.append(inner[i])
    pts2d = np.array(pts2d)
    t = np.linspace(0, 1, n_points)
    idx = (t * len(pts2d)).astype(int) % len(pts2d)
    z = np.sin(np.linspace(0, 4*np.pi, n_points))*0.5
    return np.column_stack((pts2d[idx,0]*radius, pts2d[idx,1]*radius, z))

def target_robot(n_points=40, scale=2.0):
    pts = []
    hx = np.linspace(-0.6, 0.6, 6)
    hy = np.linspace(1.5, 2.2, 5)
    for x in hx:
        for y in hy:
            pts.append((x, y))
    bx = np.linspace(-1.0, 1.0, 8)
    by = np.linspace(-0.5, 1.5, 12)
    for x in bx:
        for y in by:
            pts.append((x, y))
    for x in np.linspace(-2.0, -1.0, 6):
        for y in np.linspace(0.0, 0.8, 4):
            pts.append((x,y))
    for x in np.linspace(1.0, 2.0, 6):
        for y in np.linspace(0.0, 0.8, 4):
            pts.append((x,y))
    for x in np.linspace(-0.6, -0.2, 4):
        for y in np.linspace(-2.0, -0.6, 6):
            pts.append((x,y))
    for x in np.linspace(0.2, 0.6, 4):
        for y in np.linspace(-2.0, -0.6, 6):
            pts.append((x,y))
    pts = np.array(pts)
    idx = np.linspace(0, len(pts)-1, n_points).astype(int)
    sampled = pts[idx]*scale
    z = np.cos(np.linspace(0, 2*np.pi, n_points))*0.2
    return np.column_stack((sampled[:,0], sampled[:,1], z))

def target_dragon(n_points=60, scale=1.5):
    t = np.linspace(0, 4*np.pi, n_points)
    x = (1 + 0.3*np.cos(6*t)) * np.cos(t)
    y = (1 + 0.3*np.cos(6*t)) * np.sin(t)
    z = 0.4*np.sin(3*t)
    pts = np.column_stack((x,y,z)) * scale * 1.5
    head = pts[-1] + normalize(np.array([pts[-1,0]-pts[-2,0], pts[-1,1]-pts[-2,1], 0]))*1.5
    pts = np.vstack((pts, head))
    return pts

def greedy_assign(drones_pos, targets):
    n_d = drones_pos.shape[0]
    n_t = targets.shape[0]
    distances = np.linalg.norm(drones_pos[:,None,:] - targets[None,:,:], axis=2)
    assigned_drone = -np.ones(n_t, dtype=int)
    used = np.zeros(n_d, dtype=bool)
    for _ in range(min(n_d, n_t)):
        i,j = np.unravel_index(np.argmin(np.where(used[:,None], np.inf, distances)), distances.shape)
        if used[i]:
            break
        assigned_drone[j] = i
        used[i] = True
        distances[i,:] = np.inf
    return assigned_drone

def fitness_swarm(drones_pos, drones_vel, targets, safe_dist=0.6, energy_weight=0.1, collision_weight=5.0, obstacle_penalty=10.0, obstacles=None):
    assigned = greedy_assign(drones_pos, targets)
    matched_mask = assigned >= 0
    if matched_mask.sum() > 0:
        dists = np.linalg.norm(drones_pos[assigned[matched_mask]] - targets[matched_mask], axis=1)
        fit_dist = dists.mean()
    else:
        fit_dist = np.linalg.norm(drones_pos).mean()
    D = pairwise_distances(drones_pos)
    np.fill_diagonal(D, np.inf)
    close_pairs = (D < safe_dist)
    collision_pen = close_pairs.sum()
    energy = np.linalg.norm(drones_vel, axis=1).sum()
    obs_pen = 0.0
    if obstacles is not None:
        for (center, radius) in obstacles:
            d_to_obs = np.linalg.norm(drones_pos - center, axis=1)
            inside = np.maximum(0, radius + 0.2 - d_to_obs)
            obs_pen += inside.sum()
    cost = fit_dist + energy_weight*energy + collision_weight*collision_pen + obstacle_penalty*obs_pen
    return cost, {'fit_dist':fit_dist, 'energy':energy, 'collisions':int(collision_pen), 'obs':obs_pen, 'assigned_count': int(matched_mask.sum())}

class PSOSwarm:
    def __init__(self, n_drones=30, targets=None, bounds=[-10,10], obstacles=None, safe_dist=0.6):
        self.n = n_drones
        self.dim = 3
        self.bounds = np.array(bounds)
        self.pos = np.random.uniform(self.bounds[0], self.bounds[1], size=(self.n, self.dim))
        self.vel = np.random.normal(scale=0.5, size=(self.n, self.dim))
        self.personal_best_pos = self.pos.copy()
        self.personal_best_val = np.full(self.n, np.inf)
        self.targets = targets if targets is not None else np.zeros((n_drones,3))
        self.global_best_pos = None
        self.global_best_val = np.inf
        self.obstacles = obstacles if obstacles is not None else []
        self.safe_dist = safe_dist
        self.w = 0.7
        self.c1 = 1.4
        self.c2 = 1.4
        self.max_vel = 2.0
        self.alive = np.ones(self.n, dtype=bool)
    
    def step(self):
        cost, metrics = fitness_swarm(self.pos[self.alive], self.vel[self.alive], self.targets, safe_dist=self.safe_dist, obstacles=self.obstacles)
        if cost < self.global_best_val:
            self.global_best_val = cost
            self.global_best_pos = self.pos.copy()
        for i, idx in enumerate(np.where(self.alive)[0]):
            dists = np.linalg.norm(self.pos[idx] - self.targets, axis=1)
            nearest = dists.min() if len(dists)>0 else np.linalg.norm(self.pos[idx])
            local_cost = nearest + 0.05*np.linalg.norm(self.vel[idx])
            if local_cost < self.personal_best_val[idx]:
                self.personal_best_val[idx] = local_cost
                self.personal_best_pos[idx] = self.pos[idx].copy()
        r1 = np.random.rand(self.n, self.dim)
        r2 = np.random.rand(self.n, self.dim)
        gb_pos = self.global_best_pos if self.global_best_pos is not None else self.pos
        cognitive = self.c1 * r1 * (self.personal_best_pos - self.pos)
        social = self.c2 * r2 * (gb_pos - self.pos)
        self.vel = self.w*self.vel + cognitive + social
        speed = np.linalg.norm(self.vel, axis=1)
        too_fast = speed > self.max_vel
        self.vel[too_fast] = (self.vel[too_fast].T * (self.max_vel / speed[too_fast])).T
        self.pos[self.alive] += self.vel[self.alive]
        self.pos = np.clip(self.pos, self.bounds[0], self.bounds[1])
        return cost, metrics

    def simulate(self, iterations=200, fail_rate=0.0, adapt_obstacles=False, verbose=False):
        history = []
        for it in range(iterations):
            if fail_rate > 0:
                for i in range(self.n):
                    if self.alive[i] and random.random() < fail_rate:
                        self.alive[i] = False
                        if verbose:
                            print(f"Drone {i} failed at iter {it}")
            if adapt_obstacles and random.random() < 0.02:
                center = np.random.uniform(self.bounds[0]+2, self.bounds[1]-2, size=3)
                radius = np.random.uniform(0.6, 1.8)
                self.obstacles.append((center, radius))
                if verbose:
                    print(f"New obstacle at {center} r={radius} at iter {it}")
            cost, metrics = self.step()
            history.append({'iter':it, 'cost':cost, **metrics, 'alive': self.alive.sum()})
        return history

def run_demo(shape='star', n_drones=30, iterations=200, fail_rate=0.0, adapt_obstacles=False):
    if shape == 'star':
        targets = target_star(n_points=28, radius=6.0)
    elif shape == 'robot':
        targets = target_robot(n_points=40, scale=2.0)
    elif shape == 'dragon':
        targets = target_dragon(n_points=50, scale=2.0)
    else:
        raise ValueError("Unknown shape")

    swarm = PSOSwarm(n_drones=n_drones, targets=targets, bounds=[-8,8], obstacles=[], safe_dist=0.6)
    history = swarm.simulate(iterations=iterations, fail_rate=fail_rate, adapt_obstacles=adapt_obstacles, verbose=False)
    positions_over_time = []
    swarm = PSOSwarm(n_drones=n_drones, targets=targets, bounds=[-8,8], obstacles=[], safe_dist=0.6)
    for it in range(iterations):
        swarm.step()
        positions_over_time.append(swarm.pos.copy())

    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)
    ax.set_aspect('equal')
    scat = ax.scatter([], [], c='blue', s=40)
    target_scat = ax.scatter([], [], c='red', marker='x')

    def init():     
     scat.set_offsets(np.zeros((1, 2)))
     target_scat.set_offsets(np.zeros((1, 2)))
     return scat, target_scat

    def update(frame):
        positions = swarm.pos
        scat.set_offsets(positions[:, :2])
        target_scat.set_offsets(targets[:, :2])
        return scat, target_scat


    anim = animation.FuncAnimation(fig, update, frames=len(positions_over_time), init_func=init, interval=80, blit=False)
    plt.close(fig)
    return anim, history, targets

if __name__ == "__main__":
    # Quick demo: generates a short MP4 if ffmpeg is available
    anim, history, targets = run_demo('robot', n_drones=30, iterations=140, fail_rate=0.001, adapt_obstacles=True)
    try:
        anim.save("pso_drones_robot.gif", fps=20, dpi=150)
        print("Saved pso_drones_robot.gif in current directory")
    except Exception as e:
        print("Couldn't save MP4 automatically:", e)
        print("But the script will run the animation in a notebook or interactive session.")

