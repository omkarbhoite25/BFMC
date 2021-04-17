import matplotlib.pyplot as plt
import numpy as np
import math
   

def plot_car(state, modelcar, truckcolor="-k",):
    x = state.x
    y = state.y
    yaw = state.yaw
    steer = state.d if state.d else 0.0

    outline = np.array([[-modelcar.BACKTOWHEEL, (modelcar.LENGTH - modelcar.BACKTOWHEEL),
                        (modelcar.LENGTH - modelcar.BACKTOWHEEL), -modelcar.BACKTOWHEEL, -modelcar.BACKTOWHEEL],
                        [modelcar.WIDTH / 2, modelcar.WIDTH / 2, - modelcar.WIDTH / 2, -modelcar.WIDTH / 2, modelcar.WIDTH / 2]
    ])
    fr_wheel = np.array([[modelcar.WHEEL_LEN, -modelcar.WHEEL_LEN, -modelcar.WHEEL_LEN, modelcar.WHEEL_LEN, modelcar.WHEEL_LEN],
                         [-modelcar.WHEEL_WIDTH - modelcar.TREAD, -modelcar.WHEEL_WIDTH - modelcar.TREAD,
                          modelcar.WHEEL_WIDTH - modelcar.TREAD, modelcar.WHEEL_WIDTH - modelcar.TREAD, -modelcar.WHEEL_WIDTH - modelcar.TREAD]
    ])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += modelcar.WB
    fl_wheel[0, :] += modelcar.WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(), np.array(outline[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(), np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(), np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(), np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(), np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "*")
   

def simulate(track, states, mpcs, modelcar, track_type):

    # set figure size
    plt.rcParams["figure.figsize"] = (10, 10)

    # decide the track type
    if track_type.upper() == "RACE":
        img = plt.imread("tracks/Competition_image.jpg")
        extent = [0, 15, 0, 15]
    else:
        img = plt.imread("tracks/Test_image.jpg")
        extent = [0, 6, 0, 6]

    # get points
    waypoints_x, waypoints_y = track.get_waypoints()
    cx, cy, _, _, _ = track.get_spline_path_all()

    for i, _ in enumerate(states):

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
        plt.pause(0.11)

    plt.subplots()
    plt.plot([s.t for s in states], [s.v for s in states], "-r", label="speed")
    plt.grid(True)
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [ms]")
    plt.legend()

    plt.show()