#!/usr/bin/env python

import rospy
from dylan_msc.msg import obj
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation


class Plotter():
    def __init__(self):
        rospy.init_node('plotter_node', anonymous=True)
        rospy.Subscriber("plot2", obj, self.sub_cb)

        plt.ion()
        self.fig = plt.figure()
        self.ax = Axes3D(self.fig)
        self.graph = self.ax.scatter([], [], [], c='r', marker='o')
        axes_size = 2
        self.ax.set_xlim(-axes_size, axes_size)
        self.ax.set_ylim(0, axes_size)
        self.ax.set_zlim(-axes_size, axes_size)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.fig.canvas.draw()

        self.cent_x = []
        self.cent_y = []
        self.cent_z = []
        self.min_x = []
        self.min_y = []
        self.min_z = []
        self.max_x = []
        self.max_y = []
        self.max_z = []
        self.verts = []

        self.final_cent_x = []
        self.final_cent_y = []
        self.final_cent_z = []
        self.final_min_x = []
        self.final_min_y = []
        self.final_min_z = []
        self.final_max_x = []
        self.final_max_y = []
        self.final_max_z = []
        self.final_verts = []

        self.bb = []

    def sub_cb(self, msg):
        # rospy.loginfo("recv: %d", msg.index)

        if msg.index == 0:
            # Save entire arrays
            self.final_cent_x = self.cent_x
            self.final_cent_y = self.cent_y
            self.final_cent_z = self.cent_z
            self.final_min_x = self.min_x
            self.final_min_y = self.min_y
            self.final_min_z = self.min_z
            self.final_max_x = self.max_x
            self.final_max_y = self.max_y
            self.final_max_z = self.max_z
            self.final_verts = self.verts

            # Reset arrays
            self.cent_x = []
            self.cent_y = []
            self.cent_z = []
            self.min_x = []
            self.min_y = []
            self.min_z = []
            self.max_x = []
            self.max_y = []
            self.max_z = []
            self.verts = []

        self.cent_x.append(msg.centroid.x)
        self.cent_y.append(msg.centroid.z)
        self.cent_z.append(-msg.centroid.y)
        self.min_x.append(msg.min.x)
        self.min_y.append(msg.min.z)
        self.min_z.append(-msg.min.y)
        self.max_x.append(msg.max.x)
        self.max_y.append(msg.max.z)
        self.max_z.append(-msg.max.y)
        p1 = [self.min_x[msg.index], self.min_y[msg.index], self.min_z[msg.index]]
        p2 = [self.max_x[msg.index], self.min_y[msg.index], self.min_z[msg.index]]
        p3 = [self.max_x[msg.index], self.max_y[msg.index], self.min_z[msg.index]]
        p4 = [self.min_x[msg.index], self.max_y[msg.index], self.min_z[msg.index]]
        p5 = [self.min_x[msg.index], self.min_y[msg.index], self.max_z[msg.index]]
        p6 = [self.max_x[msg.index], self.min_y[msg.index], self.max_z[msg.index]]
        p7 = [self.max_x[msg.index], self.max_y[msg.index], self.max_z[msg.index]]
        p8 = [self.min_x[msg.index], self.max_y[msg.index], self.max_z[msg.index]]

        self.verts.append([[p1, p2, p3, p4],
                           [p5, p6, p7, p8],
                           [p1, p2, p6, p5],
                           [p3, p4, p8, p7],
                           [p2, p3, p7, p6],
                           [p5, p8, p4, p1]])

    def animation_cb(self, unused_iterator):
        for i in range(0, len(self.bb)):
            self.bb[i].remove()

        self.bb = []

        self.graph._offsets3d = (
            self.final_cent_x, self.final_cent_y, self.final_cent_z)

        ani_final_verts = self.final_verts
        for i in range(0, len(self.final_verts)):
            self.bb.append(Poly3DCollection(
                ani_final_verts[i], facecolors='cyan', linewidths=1, edgecolors='r', alpha=.25))
            self.ax.add_collection3d(self.bb[i])

        return self.graph

# TODO plot straight path of where the robot might go


if __name__ == '__main__':
    plotter1 = Plotter()
    ani = animation.FuncAnimation(
        plotter1.fig, plotter1.animation_cb, interval=200, blit=False)
    plt.show(block=True)
    # rospy.spin() not need because of plt.show()
