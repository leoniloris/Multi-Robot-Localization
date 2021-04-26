from multi_robot_localization.msg import particles as ParticlesMessageType
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
import os


# the following OUGHT to be the same as the one defined on `robot.h``
class ParticleType(Enum):
    ROBOT = 0
    PARTICLE = 1


class MapServer:
    def __init__(self, particles_topic):
        rospy.init_node("mapserver")

        self._message_queue = Queue()
        self.shutdown = Event()
        self._setup_map_plot()
        self._particles_marker = {}
        rospy.Subscriber(
            particles_topic,
            ParticlesMessageType,
            lambda message: self._message_queue.put(message),
        )

    def _setup_map_plot(self):
        self._occupancy_grid = np.loadtxt(
            os.environ["HOME"] +
            "/catkin_ws/src/multi_robot_localization/occupancy_grid/base_occupancy_grid.csv",
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
        # print(message.particles)
        self._remove_old_particles()
        self._create_new_particles(message.particles, message.robot_index)

    def _update_plot(self):
        plt.draw()
        plt.pause(0.001)

    def _remove_old_particles(self):
        for _particle_type, particle_marker in self._particles_marker.items():
            particle_marker.remove()
        self._particles_marker.clear()

    def _create_new_particles(self, particles_msg, robot_index):
        for p in particles_msg:
            self._create_new_particle(p, robot_index)

    def _create_new_particle(self, particle, robot_index):
        # INFO: swapped x and y, since that's the way the map is generated in gazebo.
        y_cells = particle.x
        x_cells = particle.y
        angle = particle.angle
        particle_type = particle.type

        particle_marker = self._create_particle_marker(
            x_cells, y_cells, angle, particle_type)

        self._particles_marker[f"{particle.id}-{robot_index}"] = particle_marker
        self._axis.add_patch(particle_marker)

    def _create_particle_marker(self, x_cells, y_cells, angle, particle_type):
        if ParticleType(particle_type) == ParticleType.PARTICLE:
            dx = 15*np.sin(angle)
            dy = 15*np.cos(angle)
            print(angle, x_cells, y_cells, dx, dy)
            return patches.FancyArrow(x_cells, y_cells, dx, dy, width=3, head_length=10, alpha=0.8, color="red")
        elif ParticleType(particle_type) == ParticleType.ROBOT:
            return patches.Circle((x_cells, y_cells), 10, alpha=0.8, color="blue")
        else:
            raise Exception("Invalid particle type")


if __name__ == "__main__":
    mapserver = MapServer("particles_broadcast")

    def signal_handler(signal, frame):
        print("exitting...")
        mapserver.shutdown.set()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    mapserver.render_map_sync()
