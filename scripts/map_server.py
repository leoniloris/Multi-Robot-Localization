from multi_robot.msg import particles as ParticlesMessageType
from threading import Event, Thread
from queue import Queue, Empty

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import logging as log
import numpy as np
import rospy


class MapServer:
    def __init__(self, particles_topic):
        log.basicConfig(level=log.DEBUG)
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
        print(message)
        log.info(message)
        self._remove_old_particles()
        # self.arrow = patches.FancyArrow(
        #     x, x, 15, 15, width=3, head_length=10, alpha=0.8, color="red"
        # )
        # self._axis.add_patch(self.arrow)

    def _update_plot(self):
        log.debug("Plot updated.")
        plt.draw()
        plt.pause(0.05)

    def _remove_old_particles(self):
        for _particle_type, particle in self._particles.items():
            particle.remove()
        self._particles.clear()


import signal
import sys
import time

if __name__ == "__main__":
    mapserver = MapServer("particles_broadcast")

    def signal_handler(signal, frame):
        print("exitting...")
        mapserver.shutdown.set()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    mapserver.render_map_sync()
