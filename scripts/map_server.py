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

MAX_N_ROBOTS = 15

colors = np.random.random(size=(MAX_N_ROBOTS, 3))


# the following OUGHT to be the same as the one defined on `robot.h``
MEASUREMENT_ANGLES = [0, 90, 270, 180]
class ParticleType(Enum):
    ROBOT = 0
    PARTICLE = 1


class MapServer:
    def __init__(self, particles_topic):
        rospy.init_node("mapserver")

        self._message_queue = Queue()
        self.shutdown = Event()
        self._setup_map_plot()
        self._particles_patches = {}

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
            print("handling message")
            self._handle_message(message)
            self._update_plot()
            # print(len(self._particles_patches))

    def _handle_message(self, message):
        self._remove_old_particles_from_robot(message.robot_index)
        self._create_new_particles(message.particles, message.robot_index)

    def _update_plot(self):
        plt.draw()
        plt.pause(0.1)

    def _remove_old_particles_from_robot(self, robot_index_to_remove):
        entries_to_remove = []
        for (robot_index, particle_id, patch_id), particle_marker in self._particles_patches.items():
            if robot_index == robot_index_to_remove:
                particle_marker.remove()
                entries_to_remove.append((robot_index, particle_id, patch_id))

        for entry_to_remove in entries_to_remove:
            del self._particles_patches[entry_to_remove]

        # self._axis.patches = []

    def _create_new_particles(self, particles_msg, robot_index):
        for p in particles_msg:
            self._create_new_particle(p, robot_index)

    def _create_new_particle(self, particle, robot_index):
        particle_patches = self._create_particle_patches(
            particle.x, particle.y, particle.angle, particle.type, particle.measurements, robot_index)

        for patch_id, particle_patch in enumerate(particle_patches):
            self._particles_patches[(
                robot_index, particle.id, patch_id)] = particle_patch
            self._axis.add_patch(particle_patch)

    def _create_particle_patches(self, x_cells, y_cells, angle, particle_type, measurements, robot_index):
        # inverted x-y!
        x_grid = y_cells
        y_grid = x_cells
        plot_angle = angle - (np.pi/2)
        color = colors[robot_index]
        if ParticleType(particle_type) == ParticleType.PARTICLE:
            dx = 15*np.cos(plot_angle)
            dy = (-15)*np.sin(plot_angle)
            return [patches.FancyArrow(x_grid, y_grid, dx, dy, width=3, head_length=10, alpha=0.3, color=color)]
        elif ParticleType(particle_type) == ParticleType.ROBOT:
            robot_patch = patches.Circle(
                (x_grid, y_grid), 15, alpha=1, color=color)
            assert len(measurements) == len(MEASUREMENT_ANGLES)
            return [robot_patch] +\
                [patches.Rectangle((x_grid, y_grid), 8, measurement, angle=(-angle*180/np.pi - measurement_angle), color='red', alpha=0.3)
                 for (measurement, measurement_angle) in zip(measurements, MEASUREMENT_ANGLES)]
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
