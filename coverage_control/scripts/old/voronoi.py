import sys
import time
import yaml
import math
import signal
import datetime
import threading
import traceback
import numpy as np
from cvxopt import matrix, solvers
#from scipy.spatial import ConvexHull
import matplotlib.patches as ptc
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# from actions import *

COLORS = [(0.0, 0.0, 0.0), (0.99, 0.0, 0.0), (0.0, 0.99, 0.0), (0.0, 0.0, 0.99), (0.99, 0.99, 0.0), (0.99, 0.0, 0.99), (0.0, 0.99, 0.99)]
global_boundary = []
xlim = []
ylim = []
test_type = 0

world = None

def is_in_space(p, tol):
    global xlim, ylim
    return xlim[0] - tol <= p[0] <= xlim[1] + tol and ylim[0] - tol <= p[1] <= ylim[1] + tol

def is_in_bounding_polygon(p, tol):
    global global_boundary
    pass

def angle_in_2pi(v):
    angle = np.arctan2(v[1], v[0])
    #if angle <= 0:
    #    angle += 2 * np.pi
    return angle

def to_grid(x, y, x_off, y_off):
    return (x - x_off, y - y_off)

#def get_convex_hull(V):
#    hull = ConvexHull(V)
#    return [V[vertex] for vertex in hull.vertices]

def appendGlobalBoundaries(B):
    bottom_left = globals()['global_boundary'][0]
    top_right = globals()['global_boundary'][3]
    B.append((np.array([1., 0.], dtype=float), np.array(bottom_left, dtype=float)))
    B.append((np.array([0., 1.], dtype=float), np.array(bottom_left, dtype=float)))
    B.append((np.array([1., 0.], dtype=float), np.array(top_right, dtype=float)))
    B.append((np.array([0., 1.], dtype=float), np.array(top_right, dtype=float)))

def angularSort(reference, vertices):
    vectors = [p - reference for p in vertices]
    indexed_angles = [(angle_in_2pi(vectors[i]), i) for i in range(len(vectors))]
    #if self.name == "uav1":
    #    print("------")
    #    for i in range(len(vectors)):
    #        print(vectors[i], indexed_angles[i][0])
    #    print("------")
    indexed_angles.sort()
    return [vertices[i] for _, i in indexed_angles]

class StateBuffer:

    def __init__(self):
        self.buffers = dict()

    def getState(self, name):
        return self.buffers[name]

    def getAllStates(self):
        return dict(self.buffers)

    def updateState(self, name, s):
        self.buffers[name] = s

class Agent:

    def __init__(self, name, init, goal, vmax):
        self.name = name
        self.move_thread = threading.Thread(name="{}_move".format(self.name), target=self.move)
        self.sim_log = open('LOG_{}.txt'.format(self.name), 'w+')

        self.terminate = False
        self.phys_radius = 2.0
        self.safe_radius = 3.0
        self.comm_radius = 10.0
        self.dt = 0.1
        self.vmax = vmax
        self.vmin = 0.5
        self.velocity = np.zeros(2)
        self.position = np.array(init, dtype=float)
        self.voronoi_graph = []
        #self.color = tuple(np.random.rand(3))
        self.color = globals()['COLORS'][int(self.name[3:])]
        self.inter_sort_type = [('angle', float), ('vector', np.ndarray)]
        self.world = None
        self.world_offset = (globals()['xlim'][0], globals()['ylim'][0])
        self.frontier = set()

        self._B = np.array([[1., 0.], [0., 1.], [1., 0.], [0., 1.]], dtype=float)

        self.neighbours = dict()
        # self.path = []
        # self.curves = []
        self.xhistory = []
        self.yhistory = []
        self.goal = np.array(goal, dtype=float)
        self.goal_change = 10.
        self.converged = False
        self.H = matrix([[2., 0.], [0., 2.]], tc='d')

        # STATE:
        self.state = {'pos': self.position, 'vel': self.velocity, 'end': False}
        self.advertiseState()
    
    def initialize_world(self):
        #global xlim, ylim
        #W = xlim[1] - xlim[0]
        #H = ylim[1] - ylim[0]
        #self.world = np.zeros((H, W))
        #grid_node = to_grid(self.position[0], self.position[1], xlim[1], ylim[1])
        #v_act = valid_actions(self.world, grid_node)
        #for act in v_act:
        #    applied_coord = apply_action_to_node(grid_node, act)
        #    pass
        pass

    def initialize(self):
        #print("Initializing agent {}".format(self.name))
        #print("Agent {} --> {}".format(self.name, self.goal))
        self.move_thread.start()

    def setGoal(self, g):
        self.goal_change = np.linalg.norm(g - self.goal)
        self.converged = self.goal_change <= 0.1
        self.goal = np.array(g, dtype=float)

    def hasReachedGoal(self):
        return np.linalg.norm(self.goal - self.state['pos']) <= 0.1 and self.converged

    def getCentroid(self):
        ### SOURCE: https://en.wikipedia.org/wiki/Centroid

        # Calculate area with Shoelace Formula
        area = 0
        for i in range(len(self.voronoi_graph) - 1):
            x_i, y_i = self.voronoi_graph[i]
            x_j, y_j = self.voronoi_graph[i + 1]
            area += x_i * y_j - x_j * y_i

        area *= 0.5

        # Calculate centroid of voronoi cell
        Cx, Cy = 0, 0
        for i in range(len(self.voronoi_graph) - 1):
            x_i, y_i = self.voronoi_graph[i]
            x_j, y_j = self.voronoi_graph[i + 1]
            product = (x_i * y_j - x_j * y_i)
            Cx += (x_i + x_j) * product
            Cy += (y_i + y_j) * product

        return np.array([Cx, Cy], dtype=float) / (6. * area)

    def computeBisectors(self):
        bisectors = [] # (normal, point)
        cons, vals = [], []
        tol = 0.1

        for a, st in self.neighbours.items():
            if st is None:
                continue

            if np.any(np.isnan(st['pos'])):
                print(f'Agent {self.name} neighbour {a} has NaN!')

            normal = (st['pos'] - self.state['pos']).round(4)
            m = ((st['pos'] + self.state['pos']) * 0.5).round(4)
            bisectors.append((normal, m))
            cons.append(normal)
            #vals.append(m.dot(normal) - self.safe_radius)
            vals.append((m.dot(normal)).round(4))

        # bottom_left = globals()['global_boundary'][0]
        # top_right = globals()['global_boundary'][3]

        # bisectors.append((np.array([1., 0.], dtype=float), np.array(bottom_left, dtype=float)))
        # bisectors.append((np.array([0., 1.], dtype=float), np.array(bottom_left, dtype=float)))
        # bisectors.append((np.array([1., 0.], dtype=float), np.array(top_right, dtype=float)))
        # bisectors.append((np.array([0., 1.], dtype=float), np.array(top_right, dtype=float)))
        appendGlobalBoundaries(bisectors)

        A = np.array(cons, dtype=float)
        b = np.array(vals, dtype=float)
        self.voronoi_graph = []
        for i in range(len(bisectors)):
            n_i, m_i = bisectors[i]
            d_i = m_i.dot(n_i)

            for j in range(i + 1, len(bisectors)):
                n_j, m_j = bisectors[j]
                d_j = m_j.dot(n_j)

                try:
                    A_ = np.array([n_i.round(4), n_j.round(4)], dtype=float)
                    b_ = np.array([d_i.round(4), d_j.round(4)], dtype=float)
                    p = (np.linalg.solve(A_, b_)).round(4)

                except np.linalg.LinAlgError:
                    continue

                except:
                    print(traceback.format_exc())
                    continue

                if is_in_space(p, tol) and np.all(A.dot(p) <= b + 0.1):
                    self.voronoi_graph.append(p)

        A_iq = matrix(np.array(cons), tc='d')
        b_iq = matrix(np.array(vals), tc='d')
        self.voronoi_graph = angularSort(self.position, self.voronoi_graph)
        #self.voronoi_graph = get_convex_hull(self.voronoi_graph)
        return A_iq, b_iq

    def solveStep(self, A_iq, b_iq, _t=0):
        v_next = self.state['vel']

        if _t == 0:
            ## Buffered Voronoi Cell

            if A_iq and b_iq:
                solvers.options['show_progress'] = False
                sol = solvers.qp(self.H, matrix(-2. * self.goal, tc='d'), A_iq, b_iq)
                #print("Agent {} SOLN: {}".format(self.name, sol['x']))

                v_next = (np.array(sol['x'][0]) - self.state['pos']) / self.dt
                _norm = np.linalg.norm(v_next)

                if _norm > self.vmax:
                    v_next = self.vmax * v_next / _norm

            return v_next

        elif _t == 1:
            ## Lloyd's Descent
            if len(self.voronoi_graph):
                self.voronoi_graph.append(self.voronoi_graph[0])
                self.setGoal(self.getCentroid())
                v_next = self.goal - self.state['pos']
                _norm = np.linalg.norm(v_next)

                if _norm > self.vmax:
                    v_next *= self.vmax / np.linalg.norm(v_next)

                return v_next

            print(f'Agent {self.name} stopped momentarily.')
            return np.zeros(2)

    def doStep(self, v_next):
        x_, y_ = self.state['pos'][0], self.state['pos'][1]
        self.xhistory.append(x_)
        self.yhistory.append(y_)
        self.state['pos'] = self.state['pos'] + self.dt * v_next
        self.state['vel'] = v_next

    def stepLog(self, _t=0):
        if _t == 0:
            self.sim_log.write('{} - pos: {} - vel: {} - at: {}\n'.format(self.name, self.position, self.velocity, datetime.datetime.now()))

        elif _t == 1:
            # Agent name; current position; next goal
            #self.sim_log.write('{};{};{}\n'.format(self.name, self.position, self.goal))
            #self.sim_log.write(f'{self.name};{self.voronoi_graph.dfs_traversal()}\n')
            #self.sim_log.write(f'{self.name};{self.voronoi_graph}\n')
            pass

    def updateNeighbours(self):
        for uav, st in globals()['buf'].buffers.items():
            if uav == self.name or st is None:
                continue

            self.neighbours[uav] = dict(st)

    def advertiseState(self):
        globals()['buf'].updateState(self.name, self.state)

    def stop(self):
        self.terminate = True

    def move(self):
        test = globals()['test_type']
        pre_flight_count = 20
        #while not self.terminate and not self.hasReachedGoal():
        while not self.terminate:
            _start = time.time()

            self.advertiseState()
            self.updateNeighbours()

            if pre_flight_count < 1:
                A, b = self.computeBisectors()
                v_next = self.solveStep(A, b, test)
                self.doStep(v_next)
                self.stepLog(test)

            else:
                pre_flight_count -= 1

            _elapsed = time.time() - _start
            fail_hard = _elapsed >= self.dt
            if fail_hard:
                #print('Agent {} failed hard real-time constraint at {}'.format(self.name, datetime.datetime.now()))
                pass

            else:
                time.sleep(self.dt - _elapsed)

        self.state['end'] = True
        if self.hasReachedGoal():
            print("Agent {} has reached goal at {}".format(self.name, datetime.datetime.now()))

        self.sim_log.close()

class Simulator:

    def __init__(self, pfile):
        self.xlim = [-20, 80]
        self.ylim = [-20, 80]
        self.count = 0
        self.agents = dict()
        self.vmax = 0
        self.iteration = 0
        self.loadParams(pfile)
        #self.logfile = open('SimulatorLog.txt', 'w+')

        self.terminate = False
        self.distance_thread = threading.Thread(name='distance_thread', target=self.checkCollision)

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)
        #self.fig, self.axs = plt.subplots(2)
        self.ani = None

    def loadParams(self, pfile):
        params = None
        with open(pfile) as P:
            params = yaml.load(P, Loader=yaml.FullLoader)

        self.xlim = np.array(params['xlim'], dtype=float)
        self.ylim = np.array(params['ylim'], dtype=float)
        self.count = params['count']
        self.vmax = params['vmax']
        globals()['test_type'] = params['test_type']
        globals()['world'] = np.zeros((int(self.ylim[1] - self.ylim[0]), int(self.xlim[1] - self.xlim[0])), dtype=int)
        globals()['xlim'] = np.array(self.xlim, dtype=float)
        globals()['ylim'] = np.array(self.ylim, dtype=float)
        #globals()['global_boundary'] = np.array([[i, j] for i in self.xlim for j in self.ylim], dtype=float)
        globals()['global_boundary'] = np.array([vertex for vertex in params['bounding_polygon']], dtype=float)
        #sorted_boundary = angularSort(np.mean(globals()['global_boundary'], axis=0), globals()['global_boundary'])
        self.bounding_poly_plt = ptc.Polygon(angularSort(np.mean(globals()['global_boundary'], axis=0), globals()['global_boundary']), 
                                             color=(0, 0, 0), fill=False)

        for entry in params['uav']:
            self.agents[entry[0]] = Agent(entry[0], entry[1], entry[2], self.vmax)

    def isDone(self):
        return all([a.state['end'] for _, a in self.agents.items()])

    def checkCollision(self):
        if not self.agents:
            return

        try:
            while not self.terminate:
                ax, ay = list(zip(*[tuple(a.state['pos']) for _, a in self.agents.items()]))
                X = np.array(ax, dtype=float)
                Y = np.array(ay, dtype=float)
                XX1, XX2 = np.meshgrid(X, X)
                YY1, YY2 = np.meshgrid(Y, Y)
                pairwise_dists = np.sqrt((XX2 - XX1) ** 2 + (YY2 - YY1) ** 2)
                R, C = pairwise_dists.shape

                for i in range(R):
                    for j in range(C):
                        if j < i and pairwise_dists[i, j] <= 2.0:
                            print('COLLISION between agents uav{} and uav{} at {}'.format(i, j, datetime.datetime.now()))

                time.sleep(1)

        except Exception:
            print(traceback.format_exc())

    def animate_motion(self, i):
        self.ax.clear()
        self.ax.set_xlim(self.xlim[0] - 5, self.xlim[1] + 5)
        self.ax.set_ylim(self.ylim[0] - 5, self.ylim[1] + 5)
        self.iteration += 1

        for _, a in self.agents.items():
            pos = a.state['pos']
            vel = a.state['vel']
            angle = np.arctan2(vel[1], vel[0])
            circle = plt.Circle(tuple(pos), 2., color=a.color)
            self.ax.quiver(pos[0], pos[1], np.cos(angle), np.sin(angle), color=a.color)
            self.ax.add_artist(circle)
            self.ax.plot(a.xhistory, a.yhistory, color=a.color)
            self.ax.add_patch(self.bounding_poly_plt)

            polygon = a.voronoi_graph
            if len(polygon) < 3:
                continue

            poly = plt.Polygon(polygon, alpha=0.4, color=a.color)
            self.ax.add_patch(poly)

    def stop(self):
        self.terminate = True

    def run(self):
        print("Run starts at {}".format(datetime.datetime.now()))

        for _, a in self.agents.items():
            a.initialize()

        self.ani = animation.FuncAnimation(self.fig, self.animate_motion, interval=100)
        #self.ani = animation.FuncAnimation(self.fig, self.animate_motion, frames=3000, interval=100)
        #self.ani.save(f'lloyd_{self.count}_uav.mp4', writer='ffmpeg', fps=30)
        self.distance_thread.start()
        plt.show()

        while not self.terminate and not self.isDone():
            time.sleep(1)

        for _, a in self.agents.items():
            a.stop()

        self.distance_thread.join()
        #self.logfile.close()
        print("Run done at {}".format(datetime.datetime.now()))

def ctrl_c_handler(signum, frame):
    globals()['sim'].stop()
    print('Closing...')

if __name__ == '__main__':
    buf = StateBuffer()
    sim = Simulator(sys.argv[1])
    signal.signal(signal.SIGINT, ctrl_c_handler)
    sim.run()