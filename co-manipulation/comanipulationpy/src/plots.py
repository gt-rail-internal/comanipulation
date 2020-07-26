#!/usr/bin/env python
# coding: utf-8

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



if __name__ == "__main__":
    data_straight = np.loadtxt("trajectories/straight.txt", delimiter=",")
    data_legible = np.loadtxt("trajectories/legibility.txt", delimiter=",")
    data_optimal = np.loadtxt("trajectories/optimal.txt", delimiter=",")
    data_visible = np.loadtxt("trajectories/visibility.txt", delimiter=",")
    data_dist = np.loadtxt("trajectories/dist.txt", delimiter=",")
    data_vel = np.loadtxt("trajectories/vel.txt", delimiter=",")


    x_straight = data_straight[:, 0]
    y_straight = data_straight[:, 1]
    z_straight = data_straight[:, 2]
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.plot(x_straight, y_straight, zs=z_straight)


    x_legible = data_legible[:, 0]
    y_legible = data_legible[:, 1]
    z_legible = data_legible[:, 2]
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.plot(x_legible, y_legible, zs=z_legible)


    x_optimal = data_optimal[:, 0]
    y_optimal = data_optimal[:, 1]
    z_optimal = data_optimal[:, 2]
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.plot(x_optimal, y_optimal, zs=z_optimal)


    x_visible = data_visible[:, 0]
    y_visible = data_visible[:, 1]
    z_visible = data_visible[:, 2]
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.plot(x_visible, y_visible, zs=z_visible)


    x_dist = data_dist[:, 0]
    y_dist = data_dist[:, 1]
    z_dist = data_dist[:, 2]
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.plot(x_dist, y_dist, zs=z_dist)


    x_vel = data_vel[:, 0]
    y_vel = data_vel[:, 1]
    z_vel = data_vel[:, 2]
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.plot(x_vel, y_vel, zs=z_vel)


    fig = plt.figure()
    ax = Axes3D(fig)
    ax.plot(x_straight, y_straight, zs=z_straight, label="straight")
    ax.plot(x_visible, y_visible, zs=z_visible, label="visible")
    ax.scatter([x_straight[0], x_visible[0]], [y_straight[0], y_visible[0]], zs=[z_straight[0], z_visible[0]], color="green", label="Start")
    ax.scatter([x_straight[-1], x_visible[-1]], [y_straight[-1], y_visible[-1]], zs=[z_straight[-1], z_visible[-1]], color="red", label="Stop")

    ax.set_title("Visibility \n")
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0.)

    plt.savefig("plots/visibility.png", dpi=600, bbox_inches="tight")


    x_gaze = []
    for i in range(11):
        x_gaze.append(0.1 * i)

    fig = plt.figure()
    ax = Axes3D(fig)
    ax.plot(x_straight, y_straight, zs=z_straight, label="straight")
    ax.plot(x_visible, y_visible, zs=z_visible, label="visible")
    ax.plot(x_gaze, x_gaze, zs=x_gaze, label="gaze", linestyle="--")
    ax.scatter([0, 1], [0, 1], zs=[0, 1], c='green')
    ax.scatter([x_straight[0], x_visible[0]], [y_straight[0], y_visible[0]], zs=[z_straight[0], z_visible[0]], color="green", label="Start")
    ax.scatter([x_straight[-1], x_visible[-1]], [y_straight[-1], y_visible[-1]], zs=[z_straight[-1], z_visible[-1]], color="red", label="Stop")

    ax.set_title("Visibility \n")
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0.)

    plt.savefig("plots/visibility_gaze.png", dpi=600, bbox_inches="tight")


    fig = plt.figure()
    ax = Axes3D(fig)
    ax.plot(x_straight, y_straight, zs=z_straight, label="straight")
    ax.plot(x_legible, y_legible, zs=z_legible, label="legible")
    ax.scatter([x_straight[0], x_legible[0]], [y_straight[0], y_legible[0]], zs=[z_straight[0], z_legible[0]], color="green", label="Start")
    ax.scatter([x_straight[-1], x_legible[-1]], [y_straight[-1], y_legible[-1]], zs=[z_straight[-1], z_legible[-1]], color="red", label="Stop")

    ax.set_title("Legibility \n")
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0.)

    plt.savefig("plots/legibility.png", dpi=600, bbox_inches="tight")


    fig = plt.figure()
    ax = Axes3D(fig)
    ax.plot(x_straight, y_straight, zs=z_straight, label="straight")
    ax.plot(x_optimal, y_optimal, zs=z_optimal, label="optimal")
    ax.scatter([x_straight[0], x_optimal[0]], [y_straight[0], y_optimal[0]], zs=[z_straight[0], z_optimal[0]], color="green", label="Start")
    ax.scatter([x_straight[-1], x_optimal[-1]], [y_straight[-1], y_optimal[-1]], zs=[z_straight[-1], z_optimal[-1]], color="red", label="Stop")

    ax.set_title("Optimal \n")
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0.)

    plt.savefig("plots/optimal.png", dpi=600, bbox_inches="tight")


    fig = plt.figure()
    ax = Axes3D(fig)
    ax.plot(x_straight, y_straight, zs=z_straight, label="straight")
    ax.plot(x_dist, y_dist, zs=z_dist, label="distance")
    ax.scatter([x_straight[0], x_dist[0]], [y_straight[0], y_dist[0]], zs=[z_straight[0], z_dist[0]], color="green", label="Start")
    ax.scatter([x_straight[-1], x_dist[-1]], [y_straight[-1], y_dist[-1]], zs=[z_straight[-1], z_dist[-1]], color="red", label="Stop")

    ax.set_title("Distance \n")
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0.)

    plt.savefig("plots/dist.png", dpi=600, bbox_inches="tight")


    fig = plt.figure()
    ax = Axes3D(fig)
    ax.plot(x_straight, y_straight, zs=z_straight, label="straight")
    ax.plot(x_vel, y_vel, zs=z_vel, label="distance")
    ax.scatter([x_straight[0], x_vel[0]], [y_straight[0], y_vel[0]], zs=[z_straight[0], z_vel[0]], color="green", label="Start")
    ax.scatter([x_straight[-1], x_vel[-1]], [y_straight[-1], y_vel[-1]], zs=[z_straight[-1], z_vel[-1]], color="red", label="Stop")

    ax.set_title("Velocity \n")
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0.)

    plt.savefig("plots/vel.png", dpi=600, bbox_inches="tight")


def plot_trajectory(traj, traj_name, ref_traj, ref_traj_name, file_name, human_pos = None, human_dof = None, obj_pos = None):
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.plot(ref_traj[:, 0], ref_traj[:, 1], zs=ref_traj[:, 2], label=ref_traj_name)
    ax.plot(traj[:, 0], traj[:, 1], zs=traj[:, 2], label=traj_name)
    ax.scatter([ref_traj[0, 0], traj[0, 0]], [ref_traj[0, 1], traj[0, 1]], zs=[ref_traj[0, 2], traj[0, 2]], color="green", label="Start")
    ax.scatter([ref_traj[-1, 0], traj[-1, 0]], [ref_traj[-1, 1], traj[-1, 1]], zs=[ref_traj[-1, 2], traj[-1, 2]], color="red", label="Stop")

    if human_pos:
        ax.scatter(human_pos[0::3], human_pos[1::3], zs=human_pos[2::3], color="blue", label="Human Position")
        # ax.plot(human_pos[0:human_dof*3:3], human_pos[1:human_dof*3:3], zs=human_pos[2:human_dof*3 + 1:3], color="magenta", label="Human Start Position")
        # ax.plot(human_pos[len(human_pos) - human_dof*3::3], human_pos[len(human_pos) - human_dof*3 + 1::3], zs=human_pos[len(human_pos) - human_dof*3 + 2::3], color="pink", label="Human Stop Position")
    if obj_pos:
        ax.scatter(obj_pos[0], obj_pos[1], zs=obj_pos[2], color="red", label="Object")


    ax.set_title(traj_name)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0.)
    plt.show()

    # plt.savefig(file_name, dpi=600, bbox_inches="tight")

def plot_spheres(human_joints_pos, robot_joints_pos, human_center=None, robot_centers=None):
    fig = plt.figure()
    ax = Axes3D(fig)

    hum_x = human_joints_pos[::3]
    hum_y = human_joints_pos[1::3]
    hum_z = human_joints_pos[2::3]


    robot_x = robot_joints_pos[::3]
    robot_y = robot_joints_pos[1::3]
    robot_z = robot_joints_pos[2::3]

    ax.scatter(hum_x, hum_y, zs=hum_z, label="human", color="blue")
    ax.scatter(robot_x, robot_y, zs=robot_z, label="robot", color="red")
    if human_center:
        ax.scatter([i[0] for i in human_center], [i[1] for i in human_center], zs=[i[2] for i in human_center], label="human", color="green")
    if robot_centers:
        ax.scatter([i[0] for i in robot_centers], [i[1] for i in robot_centers], zs=[i[2] for i in robot_centers], label="robot", color="yellow")
    plt.show()