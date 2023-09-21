import numpy as np
from scipy.io import loadmat

import matplotlib.pyplot as plt

def get_trajectory_mat(path_to_file,dt,psi=0):
    g = 9.81
    sim_data = loadmat(path_to_file)
    x = sim_data["x"] - 0.4
    y = sim_data["y"] + 0.4
    z = sim_data["z"] + 0.30

    vx = sim_data["vx"]
    vy = sim_data["vy"]
    vz = sim_data["vz"]

    ax = sim_data["ax"]
    ay = sim_data["ay"]
    az = sim_data["az"]

    nbr_agents = np.size(x,0)
    Tsim = dt*(np.size(x,1))

    ref_full = {}
    for i in range(nbr_agents):
        ref_full[i] = np.vstack((x[i,:],y[i,:],z[i,:],vx[i,:],vy[i,:],vz[i,:])).transpose()
    
    v_ref = {}
    for i in range(nbr_agents):
        v_ref[i] = np.vstack((ax[i,:],ay[i,:],az[i,:])).transpose()
        
    tt = np.arange(0, Tsim, dt)

    thrust = {}
    phi = {}
    theta = {}
    for i in range(nbr_agents):
        thrust[i] =  np.sqrt(ax[i,:] ** 2 + ay[i,:] ** 2 + (az[i,:] + 9.81) ** 2)
        phi[i] = np.arcsin((ax[i,:] * np.sin(psi) - ay[i,:] * np.cos(psi)) / thrust[i])
        theta[i] = np.arctan((ax[i,:] * np.cos(psi) + ay[i,:] * np.sin(psi)) / (az[i,:] + g))

    ref = {}
    for i in range(nbr_agents):
        ref[i] = {
        "trajectory": ref_full[i],
        "time_step": tt,
        "thrust": thrust[i],
        "phi": phi[i],
        "theta": theta[i],
        "Nsim": tt.shape[0],
        "v_ref": v_ref[i]}

    return ref


def get_ref_setpoints_takeoff(psi, Tto, dt, ref):
    knot = [0, Tto]
    g = 9.81
    tt = np.arange(min(knot), max(knot), dt)

    print(tt)
    k_pass = 1
    dest = ref[0,0:3].reshape(-1,1)
    # nbr_step = int(Tto/dt)
    nbr_step = 2
    W12 = np.repeat(dest[:2],nbr_step,axis=1)
    W3 = np.repeat(dest[2],nbr_step,axis=0)
    # W3 = np.linspace(0.15,dest[2],nbr_step).reshape((1,nbr_step))
    W = np.vstack((W12,W3))
    print(W)
    ref_tmp = np.empty((0, 3))
    waypoint_time_stamps = np.linspace(min(knot), max(knot), W.shape[1] + 1)
    for i_tmp in range(waypoint_time_stamps.shape[0] - 1):
        cur = np.array(W[:, i_tmp])
        while dt * k_pass <= waypoint_time_stamps[i_tmp + 1]:
            ref_tmp = np.vstack((ref_tmp, cur))
            k_pass = k_pass + 1

    ref_full = np.block([
        [ref_tmp, ref_tmp * 0]
    ])
    v_ref = 0 * ref_tmp.transpose()
    # v_ref[2, :] = v_ref[2, :] - 0.075
    # v_ref[0, :] = v_ref[0, :] - 0.125
    ddx, ddy, ddz = v_ref[0, :], v_ref[1, :], v_ref[2, :]
    thrust = np.sqrt(ddx ** 2 + ddy ** 2 + (ddz + 9.81) ** 2)
    phi = np.arcsin((ddx * np.sin(psi) - ddy * np.cos(psi)) / thrust)
    theta = np.arctan((ddx * np.cos(psi) + ddy * np.sin(psi)) / (ddz + g))
    ref = {
        "trajectory": ref_full,
        "time_step": tt,
        "thrust": thrust,
        "phi": phi,
        "theta": theta,
        "Nsim": tt.shape[0],
        "v_ref": v_ref.transpose()}

    return ref


def get_ref_setpoints(psi, Tsim, dt, version=1):
    knot = [0, Tsim]
    g = 9.81
    tt = np.arange(min(knot), max(knot), dt)
    if version == 1:
        W = np.array([[-0.4],
                      [0.4],
                      [0.3]  # 3D test
                      ])
    elif version == 2:
        W = np.array([[0],
                      [0.4],
                      [0.3]  # 3D test
                      ])
    elif version == 3:
        W = np.array([[-0.4],
                      [0],
                      [0.3]  # 3D test
                      ])
    elif version == 4:
        W = np.array([[0.4],
                      [-0.4],
                      [0.3]  # 3D test
                      ])
    elif version == 5:
        W = np.array([[-0.2, -0.2, -0.2,-0.2,0.3,0.8,0.8,0.8,0.3,-0.2,-0.2,-0.2],
                      [0, 0, 0, 0, 0.25,0.5,0.5,0.5,0.25,0,0,0],
                      [0.15, 0.3,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.2,0.15]  # 3D test
                      ])
    elif version == 6:
        W = np.array([[-0.2, -0.2 , -0.2,-0.2,0.8, 0.8,0.8,0.8,0.8,-0.2,-0.2,-0.2],
                      [0,    0,      0,   0,  0.5, 0.5,0.5,0.5,0.5,0,0,0],
                      [0.2,  0.2,    0.2, 0.2,0.4, 0.4,0.4,0.4,0.4,0.2,0.2,0.2]  # 3D test
                      ])
    elif version == 11:
        W = np.array([[0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0.2, 0.7, 0.7, 0.7]  # 3D test
                    ])
    elif version == 12:
        W = np.array([[-0.5, -0.5, -0.5, -0.5],
                    [0, 0, 0, 0],
                    [0.2, 0.7, 0.7, 0.7]  # 3D test
                    ])
    elif version == 13:
        W = np.array([[0.5, 0.5, 0.5, 0.5],
                    [0, 0, 0, 0],
                    [0.2, 0.7, 0.7, 0.7]  # 3D test
                    ])
    k_pass = 1
    ref_tmp = np.empty((0, 3))
    waypoint_time_stamps = np.linspace(min(knot), max(knot), W.shape[1] + 1)
    for i_tmp in range(waypoint_time_stamps.shape[0] - 1):
        cur = np.array(W[:, i_tmp])
        while dt * k_pass <= waypoint_time_stamps[i_tmp + 1]:
            ref_tmp = np.vstack((ref_tmp, cur))
            k_pass = k_pass + 1

    ref_full = np.block([
        [ref_tmp, ref_tmp * 0]
    ])
    v_ref = 0 * ref_tmp.transpose()
    # v_ref[2, :] = v_ref[2, :] - 0.075
    # v_ref[0, :] = v_ref[0, :] - 0.125
    ddx, ddy, ddz = v_ref[0, :], v_ref[1, :], v_ref[2, :]
    thrust = np.sqrt(ddx ** 2 + ddy ** 2 + (ddz + 9.81) ** 2)
    phi = np.arcsin((ddx * np.sin(psi) - ddy * np.cos(psi)) / thrust)
    theta = np.arctan((ddx * np.cos(psi) + ddy * np.sin(psi)) / (ddz + g))
    ref = {
        "trajectory": ref_full,
        "time_step": tt,
        "thrust": thrust,
        "phi": phi,
        "theta": theta,
        "Nsim": tt.shape[0],
        "v_ref": v_ref.transpose()}

    return ref


if __name__ == "__main__":
    ptf = './Trajectory/traj4UAVs_Vincent3.mat'
    ref = get_trajectory_mat(ptf,dt=0.1)

    print(type(ref[0]["trajectory"]))
    print(np.size(ref[0]["trajectory"],0))
    print(np.size(ref[0]["trajectory"],1))
    print(np.size(ref[0]["v_ref"],0))
    print(np.size(ref[0]["v_ref"],1))
    print(np.size(ref[0]["time_step"],0))

    print(max(ref[0]["time_step"]))
    print(ref[0]["time_step"])

    fig1 = plt.figure()
    ax1 = fig1.add_subplot(1, 1, 1, projection="3d")
    ax1.plot(ref[0]["trajectory"][:, 0], ref[0]["trajectory"][:, 1], ref[0]["trajectory"][:, 2])
    ax1.plot(ref[1]["trajectory"][:, 0], ref[1]["trajectory"][:, 1], ref[1]["trajectory"][:, 2])
    ax1.plot(ref[2]["trajectory"][:, 0], ref[2]["trajectory"][:, 1], ref[2]["trajectory"][:, 2])
    ax1.plot(ref[3]["trajectory"][:, 0], ref[3]["trajectory"][:, 1], ref[3]["trajectory"][:, 2])


    full_ref_takeoff = get_ref_setpoints_takeoff(psi=0,Tto=10,dt=0.1,ref=ref[0]['trajectory'])
    ref_takeoff = {'Test': full_ref_takeoff["trajectory"]}
    vref_takeoff = {'Test': full_ref_takeoff["v_ref"]}

    fig = plt.figure()
    ax = fig.add_subplot()
    ax.plot(full_ref_takeoff['time_step'],ref_takeoff['Test'][:,0])
    ax.plot(full_ref_takeoff['time_step'],ref_takeoff['Test'][:,1])
    ax.plot(full_ref_takeoff['time_step'],ref_takeoff['Test'][:,2])

    plt.show()
