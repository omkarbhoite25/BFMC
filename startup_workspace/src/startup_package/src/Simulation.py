import matplotlib.pyplot as plt
import numpy as np
import math
import cv2
import rospy




def simulate(track, states, mpcs, modelcar, track_type, car=None, time=None, gps=None, bno=None):

    # set figure size
    plt.rcParams["figure.figsize"] = (10, 10)

    # decide the track type
    if track_type.upper() == "RACE":
        img = plt.imread("/home/alberto/Documents/BFMC_Simulator/startup_workspace/src/startup_package/src/tracks/Competition_image.jpg")
        extent = [0, 15, 0, 15]
    else:
        img = plt.imread("tracks/Test_image.jpg")
        extent = [0, 6, 0, 6]

    # get points
    waypoints_x, waypoints_y = track.get_waypoints()
    cx, cy, _, _, _ = track.get_spline_path_all()

    # plt.ion()
    for i, _ in enumerate(states):
        # start = rospy.get_time()
        # accumuate values
        x, y, v, t = [], [], [], []
        target_x, target_y = [], []
        for j in range(i+1):
            x.append(states[j].x)
            y.append(states[j].y)
            v.append(states[j].v)
            t.append(states[j].t)
            target_x.append(mpcs[j].cx[mpcs[j].target_ind])
            target_y.append(mpcs[j].cy[mpcs[j].target_ind])

        # for stopping simulation with the esc key.
        plt.cla()
        plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])

        plt.plot(waypoints_x, waypoints_y, 'or', markersize=3, label="waypoints")                                           # draw waypoints
        plt.plot(cx, cy, color="r", label="course")                                                                         # draw predicted spline path
        plot_car(states[i], modelcar, truckcolor='-m')                                                                      # draw car
        plt.plot(x, y, "-y", label="trajectory", linestyle='dashed')                                                        # plot car path
        plt.plot(mpcs[i].cx[mpcs[i].target_ind], mpcs[i].cy[mpcs[i].target_ind], "og", markersize=7, label="target")        # plot target index
        # plt.plot(target_x, target_y, "xg", label="target")                                                                # plot target index
        if mpcs[i].ox is not None:                                                                                          # draw mpc predicted path and reference tracjector
            plt.plot(mpcs[i].ox, mpcs[i].oy, "-b", linewidth=3, label="MPC")
        plt.plot(mpcs[i].xref[0, :], mpcs[i].xref[1, :], "-g", linewidth=3,  label="xref")

        # plot background
        plt.imshow(img, extent=extent)

        # other parameters
        plt.gca().invert_yaxis()
        plt.legend()
        plt.axis("equal")
        plt.grid(False)
        plt.title("Time[s]:" + str(round(states[i].t, 2)) + ", speed[m/s]:" + str(round(states[i].v, 2)))
        # car.stop(0.0)
        # print(states[i].yaw)
        if gps._ox:
            plt.plot(gps._ox, gps._oy, color="w", label="GPS position")
        plt.pause(0.2)
        # img = cv2.cvtColor(signdetector.detection(cam.getImage()), cv2.COLOR_BGR2RGB)
        # imshow.set_data(img)
        # plt.draw()
        # end = rospy.get_time()
        # print("took:",end-start)
        rospy.sleep(0.175)
    car.stop(0.0)

    # plt.subplots()
    # plt.plot([s.t for s in states], [s.v for s in states], "-r", label="speed")
    # plt.grid(True)
    # plt.xlabel("Time [s]")
    # plt.ylabel("Speed [ms]")
    # plt.legend()

    # plt.show()