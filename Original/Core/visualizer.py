#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import matplotlib.pyplot as plt

class Visualizer:
    def __init__(self):
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_aspect('equal')
        self.lidar_points, = self.ax.plot([], [], 'bo')
        
    def init_lidar_plot(self):
        plt.ion()
        plt.show()
        print("Lidar Visualizer Ready")
        
    def show_camera(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cv2.imshow("original", image)
        cv2.imshow("gray", gray)
        cv2.waitKey(1)
        
    def update_lidar(self, ranges):
        if ranges is not None:
            angles = np.linspace(0, 2*np.pi, len(ranges)) + np.pi/2
            x = ranges * np.cos(angles)
            y = ranges * np.sin(angles)
            
            self.lidar_points.set_data(x, y)
            self.fig.canvas.draw_idle()
            plt.pause(0.01)
