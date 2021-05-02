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

MAX_N_ROBOTS = 30

colors = np.random.random(size=(MAX_N_ROBOTS, 3))


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
            "/catkin_ws/src/multi_robot_localization/occupancy_grid/cross.csv",
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
            print(len(self._particles_marker))

    def _handle_message(self, message):
        self._remove_old_particles_from_robot(message.robot_index)
        self._create_new_particles(message.particles, message.robot_index)

    def _update_plot(self):
        plt.draw()
        plt.pause(0.1)

    def _remove_old_particles_from_robot(self, robot_index_to_remove):
        entries_to_remove = []
        for (robot_index, particle_id), particle_marker in self._particles_marker.items():
            if robot_index == robot_index_to_remove:
                particle_marker.remove()
                # print(f'removing {robot_index}, {particle_id}')
                entries_to_remove.append((robot_index, particle_id))

        # for entry_to_remove in entries_to_remove:
        #     del self._particles_marker[entry_to_remove]

    def _create_new_particles(self, particles_msg, robot_index):
        np.random.choice(particles_msg)
        particles = np.random.choice(particles_msg, 500 if 500 < len(
            particles_msg) else len(particles_msg), replace=False)
        for p in np.concatenate((particles, particles_msg[-1:])):
            self._create_new_particle(p, robot_index)

    def _create_new_particle(self, particle, robot_index):
        particle_marker = self._create_particle_marker(
            particle.x, particle.y, particle.angle, particle.type, robot_index)

        self._particles_marker[(robot_index, particle.id)] = particle_marker
        self._axis.add_patch(particle_marker)

    def _create_particle_marker(self, x_cells, y_cells, angle, particle_type, robot_index):
        # inverted x-y!
        x_grid = y_cells
        y_grid = x_cells
        plot_angle = angle - (np.pi/2)
        # print(x_grid, y_grid, angle)
        color = colors[robot_index]
        if ParticleType(particle_type) == ParticleType.PARTICLE:
            dx = 15*np.cos(plot_angle)
            dy = (-15)*np.sin(plot_angle)
            return patches.FancyArrow(x_grid, y_grid, dx, dy, width=3, head_length=10, alpha=0.5, color=color)
        elif ParticleType(particle_type) == ParticleType.ROBOT:
            return patches.Circle((x_grid, y_grid), 10, alpha=1, color=color)
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
