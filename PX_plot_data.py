import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def rms(X):
    y = np.sqrt(np.mean(X ** 2))
    return y

URI0 = 'radio://0/80/2M/E7E7E7E7E5'
URI1 = 'radio://0/80/2M/E7E7E7E7E7'
URI2 ='radio://0/80/2M/E7E7E7E7E8'
URI3 ='radio://0/110/2M/E7E7E7E7E9'

if __name__=='__main__':
    
    # id of simulation
    sim_id = 2000

    # list_of_bodies = ['DroneE7', 'DroneE8','DroneE9']
    # uris = [URI1,URI2,URI3]
    list_of_bodies = ['DroneE5', 'DroneE7','DroneE8','DroneE9']
    uris = [URI0,URI1,URI2,URI3]
    # list_of_bodies = ['DroneE7','DroneE9']
    # uris = [URI1,URI3]
    # list_of_bodies = ['DroneE9']
    # uris = [URI3]


    Ts = 0.1        # sampling time
    Tsim = 30       # Total simulation time
    # time_stamp=np.arange(start=0,step=Ts,stop=Tsim)
    # print(np.size(time_stamp,0))
    # print('Time stamp : ',len(time_stamp))
    load_data = np.load('./Data_Drone/PX_data_drone_{}.npy'.format(sim_id),allow_pickle=True).item()
    data = load_data['result']
    ref = load_data['parameter']
    nb = len(data)
    print("number of bodies = ",nb)
    xref = {}
    yref = {}
    zref = {}
    vxref = {}
    vyref = {}
    vzref = {}
    
    for i in uris:
        xref[i] = ref[i][:,0]
        yref[i] = ref[i][:,1]
        zref[i] = ref[i][:,2]
        vxref[i] = ref[i][:,3]
        vyref[i] = ref[i][:,4]
        vzref[i] = ref[i][:,5]


    x = {}
    y = {}
    z = {}
    vx = {}
    vy = {}
    vz = {}
    T = {}
    Roll = {}
    Pitch = {}
    Yaw = {}
    time = {}
    for i in list_of_bodies:
        x[i] = data[i][0::11]
        y[i] = data[i][1::11]
        z[i] = data[i][2::11]
        vx[i] = data[i][3::11]
        vy[i] = data[i][4::11]
        vz[i] = data[i][5::11]
        T[i] = data[i][6::11]
        Roll[i] = data[i][7::11]
        Pitch[i] = data[i][8::11]
        Yaw[i] = data[i][9::11]
        time[i] = data[i][10::11]

    # for i in range(len(list_of_bodies)):
    #     print(T[list_of_bodies[i]][len(T[list_of_bodies[i]])//2])
    #     print(z[list_of_bodies[i]][len(z[list_of_bodies[i]])//2]-zref[uris[i]][len(zref[uris[i]])//2])
    #     print(vz[list_of_bodies[i]][len(vz[list_of_bodies[i]])//2]-vzref[uris[i]][len(vzref[uris[i]])//2])
    #     print(x[list_of_bodies[i]][len(x[list_of_bodies[i]])//2]-xref[uris[i]][len(xref[uris[i]])//2])
    #     print(vx[list_of_bodies[i]][len(vx[list_of_bodies[i]])//2]-vxref[uris[i]][len(vxref[uris[i]])//2])
    #     print(y[list_of_bodies[i]][len(y[list_of_bodies[i]])//2]-yref[uris[i]][len(yref[uris[i]])//2])
    #     print(vy[list_of_bodies[i]][len(vy[list_of_bodies[i]])//2]-vyref[uris[i]][len(vyref[uris[i]])//2])

    # print('Timestamps : ',len(time[list_of_bodies[0]]))

    # Plot position in 3D plot:
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(projection='3d')


    for i in range(len(list_of_bodies)):
        ax1.plot(x[list_of_bodies[i]],y[list_of_bodies[i]],z[list_of_bodies[i]], '--',label='Trajectory')
        ax1.scatter(xref[uris[i]],yref[uris[i]],zref[uris[i]],label='Reference')

    
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('y (m)')
    ax1.set_zlabel('z (m)')
    ax1.legend()
    
    fig12 = plt.figure()
    for i in range(len(list_of_bodies)):  
        ax1 = fig12.add_subplot(nb,1,i+1)
        ax1.plot(time[list_of_bodies[0]],z[list_of_bodies[i]],label='Trajectory')
        ax1.plot(time[list_of_bodies[0]],zref[uris[i]],label='Reference')
        ax1.grid(True)
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel(f"z {list_of_bodies[i]} (m)")
        if (i==0):
            ax1.legend()
    fig13 = plt.figure()
    for i in range(len(list_of_bodies)):  
        ax1 = fig13.add_subplot(nb,1,i+1)
        # ax1.plot(time[list_of_bodies[i]],x[list_of_bodies[i]])
        # ax1.plot(time[list_of_bodies[i]],xref[uris[i]])
        ax1.plot(time[list_of_bodies[0]],x[list_of_bodies[i]],label='Trajectory')
        ax1.plot(time[list_of_bodies[0]],xref[uris[i]],label='Reference')
        ax1.grid(True)
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel(f"x {list_of_bodies[i]} (m)")
        if (i==0):
            ax1.legend()
    fig14 = plt.figure()
    for i in range(len(list_of_bodies)):  
        ax1 = fig14.add_subplot(nb,1,i+1)
        # ax1.plot(time[list_of_bodies[i]],y[list_of_bodies[i]],label='Trajectory')
        # ax1.plot(time[list_of_bodies[i]],yref[uris[i]])
        ax1.plot(time[list_of_bodies[0]],y[list_of_bodies[i]],label='Trajectory')
        ax1.plot(time[list_of_bodies[0]],yref[uris[i]],label='Reference')
        ax1.grid(True)
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel(f"y {list_of_bodies[i]} (m)")
        if (i==0):
            ax1.legend()
    fig21 = plt.figure()
    for i in range(len(list_of_bodies)):
        ax2 = fig21.add_subplot(nb,1,i+1)
        # ax2.plot(time[list_of_bodies[i]],vx[list_of_bodies[i]],label='Trajectory')
        # ax2.plot(time[list_of_bodies[i]],vxref[uris[i]])
        ax2.plot(time[list_of_bodies[0]],vx[list_of_bodies[i]],label='Trajectory')
        ax2.plot(time[list_of_bodies[0]],vxref[uris[i]],label='Reference')
        ax2.grid(True)
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel(f"vx {list_of_bodies[i]} (m/s)")
        if (i==0):
            ax2.legend()


    fig22 = plt.figure()
    for i in range(len(list_of_bodies)):  
        ax2 = fig22.add_subplot(nb,1,i+1)
        ax2.plot(time[list_of_bodies[0]],vy[list_of_bodies[i]],label='Trajectory')
        ax2.plot(time[list_of_bodies[0]],vyref[uris[i]],label='Reference')
        ax2.grid(True)
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel(f"vy {list_of_bodies[i]} (m/s)")
        if (i==0):
            ax2.legend()


    fig23 = plt.figure()
    for i in range(len(list_of_bodies)):  
        ax2 = fig23.add_subplot(nb,1,i+1)
        ax2.plot(time[list_of_bodies[0]],vz[list_of_bodies[i]],label='Trajectory')
        ax2.plot(time[list_of_bodies[0]],vzref[uris[i]],label='Reference')
        ax2.grid(True)
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel(f"vz {list_of_bodies[i]} (m/s)")
        if (i==0):
            ax2.legend()   

    fig3 = plt.figure()
    for i in range(len(list_of_bodies)):
        ax3 = fig3.add_subplot(nb,1,i+1)
        ax3.plot(time[list_of_bodies[0]],T[list_of_bodies[i]])
        ax3.grid(True)
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel(f"Thrust {list_of_bodies[i]} (m/s^{2})")

    fig32 = plt.figure()
    for i in range(len(list_of_bodies)):
        ax3 = fig32.add_subplot(nb,1,i+1)
        ax3.plot(time[list_of_bodies[0]],Roll[list_of_bodies[i]]*180/np.pi)
        ax3.grid(True)
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel(f"Roll {list_of_bodies[i]} (°)")

    fig33 = plt.figure()
    for i in range(len(list_of_bodies)):
        ax3 = fig33.add_subplot(nb,1,i+1)
        ax3.plot(time[list_of_bodies[0]],Pitch[list_of_bodies[i]]*180/np.pi)
        ax3.grid(True)
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel(f"Pitch {list_of_bodies[i]} (°)")
    
    fig34 = plt.figure()
    for i in range(len(list_of_bodies)):
        ax3 = fig34.add_subplot(nb,1,i+1)
        ax3.plot(time[list_of_bodies[0]],Yaw[list_of_bodies[i]])
        ax3.grid(True)
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel(f"Yaw {list_of_bodies[i]} (°)")
    plt.show()