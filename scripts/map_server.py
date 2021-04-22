from multi_robot.msg import particles as ParticlesMessageType
from threading import Event, Thread
from queue import Queue, Empty
from enum import Enum

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import signal
import rospy
import time
import sys


# This OUGHT to be the same as the one defined on `robot.h`
class ParticleType(Enum):
    ROBOT = 0
    PARTICLE = 1


class MapServer:
    def __init__(self, particles_topic):
        rospy.init_node("mapserver")

        self._message_queue = Queue()
        self.shutdown = Event()
        self._setup_map_plot()
        self._particles = {}

        rospy.Subscriber(
            particles_topic,
            ParticlesMessageType,
            lambda message: self._message_queue.put(message),
        )

    def _setup_map_plot(self):
        self._occupancy_grid = np.loadtxt(
            "/home/leoni/catkin_ws/src/multi_robot/occupancy_grid/base_occupancy_grid.csv",
            delimiter=",",
        )

        fig = plt.figure()
        self._axis = fig.add_subplot(1, 1, 1)
        self._axis.matshow(self._occupancy_grid, cmap="YlGnBu")
        self.arrow = None

    def render_map_sync(self):
        while not self.shutdown.is_set():
            try:
                message = self._message_queue.get(timeout=3)
            except Empty:
                continue
            self._handle_message(message)
            self._update_plot()

    def _handle_message(self, message):
        print(message.particles)
        self._remove_old_particles()
        self._create_new_particles(message.particles, message.robot_index)

    def _update_plot(self):
        plt.draw()
        plt.pause(0.001)

    def _remove_old_particles(self):
        for _particle_type, particle in self._particles.items():
            particle.remove()
        self._particles.clear()

    def _create_new_particles(self, particles_msg, robot_index):
        for p in particles_msg:
            if ParticleType(p.type) == ParticleType.PARTICLE:
                dx = 15*np.cos(np.pi/2 - p.angle)
                dy = 15*np.sin(np.pi/2 - p.angle)
                particle_marker = patches.FancyArrow(p.x, p.y, dx, dy, width=3, head_length=10, alpha=0.8, color="red")
            elif ParticleType(p.type) == ParticleType.ROBOT:
                particle_marker = patches.Circle((p.x, p.y), 10, alpha=0.8, color="blue")
            else:
                raise Exception("Invalid particle type")

            self._particles[f"{p.id}-{robot_index}"] = particle_marker
            self._axis.add_patch(particle_marker)


if __name__ == "__main__":
    mapserver = MapServer("particles_broadcast")

    def signal_handler(signal, frame):
        print("exitting...")
        mapserver.shutdown.set()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    mapserver.render_map_sync()
