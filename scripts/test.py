#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D


# class VelocityAnalysis:
#     def __init__(self):
#         self.fig = plt.figure(figsize=(8, 6))

#         # Creating the axis to plot on
#         self.ax1 = self.fig.add_axes([0.3125, 0.5, 0.625, (2.5/6)])
#         # self.ax2 = self.fig.add_axes([0.0625, 0.5, 0.125, (2.5/6)])
#         # self.ax3 = self.fig.add_axes([0.0625, (0.5/6), 0.375, 0.25])
#         # self.ax4 = self.fig.add_axes([0.5625, (0.5/6), 0.375, 0.25])

#         # self.ax1.set_ylim(-1, 1)

#         # # Creating the line to add to the axis
#         # self.line1 = Line2D([], [])
#         # self.line2 = Line2D([], [])
#         # self.line3 = Line2D([], [])
#         # self.line4 = Line2D([], [])

#         # # Add the lines to the axis
#         # self.ax1.add_line(self.line1)
#         # self.ax1.add_line(self.line2)
#         # self.ax1.add_line(self.line3)
#         # self.ax1.add_line(self.line4)

#         # Initialize data list
#         self.vel1, self.vel11 = [], []
#         self.vel2, self.vel21 = [], []
#         self.vel3, self.vel31 = [], []
#         self.vel4, self.vel41 = [], []
#         self.seq1 = 0
#         self.seq2 = 0
#         self.count1, self.count11 = [], []
#         self.count2, self.count21 = [], []

#     # def plot_init(self):
#     #     self.ax1.set_ylim(-1, 1)

#     def odom_callback(self, msg):
#         self.vel11.append(msg.twist.twist.linear.x)
#         if (len(self.vel11) > 10):
#             self.vel1 = self.vel11[-9:]
#         else:
#             self.vel1 = self.vel11
        
#         self.count11.append(self.seq1)
#         self.seq1 += 1
#         if (len(self.count11) > 10):
#             self.count1 = self.count11[-9:]
#         else:
#             self.count1 = self.count11


#         self.vel21.append(msg.twist.twist.angular.z)
#         if (len(self.vel21) > 10):
#             self.vel2 = self.vel21[-9:]
#         else:
#             self.vel2 = self.vel21

    

#     def cmd_callback(self, msg):
#         self.vel31.append(msg.linear.x)
#         if (len(self.vel31) > 10):
#             self.vel3 = self.vel31[-9:]
#         else:
#             self.vel3 = self.vel31

#         self.count21.append(self.seq1)
#         self.seq2 += 1
#         if (len(self.count11) > 10):
#             self.count2 = self.count21[-9:]
#         else:
#             self.count2 = self.count21

#         self.vel4.append(msg.angular.z)
#         if (len(self.vel41) > 10):
#             self.vel4 = self.vel41[-9:]
#         else:
#             self.vel4 = self.vel41

        

#     def update_plot(self, frame):
#         # self.line1.set_data(self.count1, self.vel1)
#         # self.line2.set_data(self.count1, self.vel2)
#         # self.line3.set_data(self.count2, self.vel3)
#         # self.line4.set_data(self.count2, self.vel4)
#         # return self.line1, self.line2, self.line3, self.line4
#         self.ax1.cla()
#         self.ax1.set_ylim(-1, 1)
#         # self.ax1.plot(self.count1, self.vel1)
#         # self.ax1.plot(self.count1, self.vel2)
#         self.ax1.plot(self.count2, self.vel3)
#         # self.ax1.plot(self.count2, self.vel4)

class VelocityAnalysis:
    def __init__(self):
        self.fig = plt.figure(figsize=(8, 6))

        # Creating the axis to plot on
        self.ax1 = self.fig.add_axes([0.3125, 0.5, 0.625, (2.5/6)])
        self.ax2 = self.fig.add_axes([0.0625, 0.5, 0.125, (2.5/6)])
        self.ax3 = self.fig.add_axes([0.0625, (0.5/6), 0.375, 0.25])
        self.ax4 = self.fig.add_axes([0.5625, (0.5/6), 0.375, 0.25])

        # Creating the line to add to the axis
        self.line1 = Line2D([], [])
        self.line2 = Line2D([], [])
        self.line3 = Line2D([], [])
        self.line4 = Line2D([], [])

        # Add the lines to the axis 
        self.ax1.add_line(self.line1)
        self.ax1.add_line(self.line2)
        self.ax1.add_line(self.line3)
        self.ax1.add_line(self.line4)

        # Initialize data list
        self.vel1, self.vel11 = [], []
        self.vel2, self.vel21 = [], []
        self.vel3, self.vel31 = [], []
        self.vel4, self.vel41 = [], []
        self.seq1 = 0
        self.seq2 = 0
        self.count1, self.count11 = [], []
        self.count2, self.count21 = [], []

    def plot_init(self):
        self.ax1.set_ylim(-1, 1)
        return self.line1, self.line2

    def odom_callback(self, msg):
        self.vel11.append(msg.twist.twist.linear.x)
        if (len(self.vel11) > 10):
            self.vel1 = self.vel11[-9:]
        else:
            self.vel1 = self.vel11
        
        self.count11.append(self.seq1)
        self.seq1 += 1
        if (len(self.count11) > 10):
            self.count1 = self.count11[-9:]
        else:
            self.count1 = self.count11


        self.vel21.append(msg.twist.twist.angular.z)
        if (len(self.vel21) > 10):
            self.vel2 = self.vel21[-9:]
        else:
            self.vel2 = self.vel21

    

    def cmd_callback(self, msg):
        self.vel31.append(msg.linear.x)
        if (len(self.vel31) > 10):
            self.vel3 = self.vel31[-9:]
        else:
            self.vel3 = self.vel31

        self.count21.append(self.seq1)
        self.seq2 += 1
        if (len(self.count11) > 10):
            self.count2 = self.count21[-9:]
        else:
            self.count2 = self.count21

        self.vel4.append(msg.angular.z)
        if (len(self.vel41) > 10):
            self.vel4 = self.vel41[-9:]
        else:
            self.vel4 = self.vel41

    def update_plot(self, frame):
        self.line1.set_data(self.count1, self.vel1)
        self.line2.set_data(self.count1, self.vel2)
        # self.line3.set_data(self.count2, self.vel3)
        # self.line4.set_data(self.count2, self.vel4)
        return self.line1, self.line2


rospy.init_node('odom_visualizer')
vis = VelocityAnalysis()
sub = rospy.Subscriber('/odom', Odometry, vis.odom_callback)
sub2 = rospy.Subscriber('/cmd_vel', Twist, vis.cmd_callback)
ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init, blit=True)
plt.show(block=True)
