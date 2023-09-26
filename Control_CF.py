import time
import math
from threading import Thread
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

import numpy as np
import control_packagecf as cfcontrol
import get_data_QTM as cfqtm
import save_plot

import Trajectory_generation as trajgen


import yaml

## Load parameters from config file
with open('Config_Crazyflie.yaml') as f:
    system_parameters = yaml.load(f,Loader=yaml.FullLoader)

g = 9.81
qtm_ip = system_parameters['qtm_ip']
Ts = system_parameters['Ts']
Tsim = system_parameters['Tsim']
m = system_parameters['mass']
uris = system_parameters['uris']
drone_bodies = system_parameters['drone_bodies']
T_coeff = system_parameters['T_coeff']
alpha = system_parameters['alpha']
v_land = 0.03 # m/s
T_land = 1.5 # second

## Load Trajectories for drones:

ptf = './Trajectory/traj4UAVs_Vincent3.mat'
full_ref = trajgen.get_trajectory_mat(path_to_file=ptf,dt=Ts)

ref = {}

vref = {}   
for i in range(len(drone_bodies)):
    ref[uris[i]] = full_ref[i]["trajectory"]

    vref[uris[i]] = full_ref[i]["v_ref"]

""" Takeoff of drone """
# Takeoff in 10sec to the initial position of the trajectory
# ((at least above the ground 15cm)
Tto = 10
# generate takeoff trajectory based on initial positions of drones
full_ref_to1 = trajgen.get_ref_setpoints_takeoff(psi=0,Tto=Tto,dt=Ts,ref=ref[uris[0]])
full_ref_to2 = trajgen.get_ref_setpoints_takeoff(psi=0,Tto=Tto,dt=Ts,ref=ref[uris[1]])
full_ref_to3 = trajgen.get_ref_setpoints_takeoff(psi=0,Tto=Tto,dt=Ts,ref=ref[uris[2]])
full_ref_to4 = trajgen.get_ref_setpoints_takeoff(psi=0,Tto=Tto,dt=Ts,ref=ref[uris[3]])

refto = {uris[0]: full_ref_to1["trajectory"],
       uris[1]: full_ref_to2["trajectory"],
      uris[2]: full_ref_to3["trajectory"],
      uris[3]: full_ref_to4["trajectory"]} 

vrefto = {uris[0]: full_ref_to1["v_ref"],
        uris[1]: full_ref_to2["v_ref"],
        uris[2]: full_ref_to3["v_ref"],
        uris[3]: full_ref_to4["v_ref"]}

""" References for landing point  
"""
ref_land = {}
ref_land[uris[0]] = np.array([[-0.0],[-0.0], [0.05],[0],[0],[0]])

vref_land = np.array([[0],[0],[0]])


## Parameters for controller LQR:
Kf = {}
for i in range(len(drone_bodies)):
    Kf[uris[i]]  =  -2.0 *  np.array([[2.5, 0, 0, 1.5, 0, 0],
                                    [0, 2.5, 0, 0, 1.5, 0],
                                    [0, 0, 2.5, 0, 0, 1.5]]) #gain matrix obtained from LQR

## Parameters for controller LQI
Klqi = {}
A = np.diag([4.64, 4.64, 4.64])
B = np.diag([4.52, 4.52, 4.52])
C = np.diag([-1.068, -1.068, -1.068])
Ki = np.hstack((A,B,C))

for i in range(len(drone_bodies)):
    Klqi[drone_bodies[i]] = Ki * -1.0



def wait_for_param_download(scf):
    while not scf.cf.param.is_updated:
        time.sleep(1.0)
    print('Parameters downloaded for', scf.cf.link_uri)

def unlock_safety_check(scf):
    scf.cf.commander.send_setpoint(0,0,0,0)

def apply_control(scf, controller_cf):
    scf.cf.commander.send_setpoint(controller_cf[0],controller_cf[1],
                        controller_cf[2],controller_cf[3])

def land(scf,currentThrust,currentz):
    T_land = currentz/v_land
    for i in np.arange(0,T_land,Ts):
        scf.cf.commander.send_setpoint(0,0,0,int(currentThrust*(1-0.1*i/T_land)))


if __name__ == '__main__':

    # id of simulation for recording(can be either number or string)
    simid = 1            


    data = cfqtm.StreamingData6DEULER(qtm_ip,drone_bodies)
    data.start()

    while(not data.finish_initial):
        cfqtm.accurate_delay(1000)
    
    cflib.crtp.init_drivers(enable_debug_driver=False)
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:

        swarm.reset_estimators()
        print('Estimators have been reset')
        swarm.parallel_safe(wait_for_param_download)
        swarm.sequential(unlock_safety_check)

        output_data = {}
        output_data = dict.fromkeys(drone_bodies, np.empty((0, 11)))
        controls = {}

        ################################################
        ##### Takeoff control loop LQR #################
        ################################################

        start = time.perf_counter()
        cnt = 0
        while cnt < len(refto[uris[0]]):
            cf_control = {}
            tic = time.perf_counter()
            for i in range(len(drone_bodies)):
                pos_tmp = np.array([data.feedback_QTM[drone_bodies[i]][0:6]]).T
                yaw_tmp = data.feedback_QTM[drone_bodies[i]][6] * np.pi/180
                v = cfcontrol.compute_control(vrefto[uris[i]][cnt, :].reshape(-1, 1), pos_tmp, refto[uris[i]][cnt].reshape(-1,1), Kf[uris[i]])
                cf_control[uris[i]] = [cfcontrol.get_cf_input(v, yaw_tmp, T_coeff=T_coeff[i],alpha=alpha[i], mass=m[i])]
                controls[drone_bodies[i]] = cfcontrol.get_real_input(v,yaw_tmp)
            swarm.parallel_safe(apply_control, args_dict=cf_control)
            # swarm.sequential(apply_control, args_dict=cf_control) 
            
            cfqtm.accurate_delay((Ts-(time.perf_counter()-tic))*1000)  # either using sleep() function or accurate_delay() function
            cnt=cnt+1

        

        ####################################
        ########## Control loop ############
        ####################################
        

        start = time.perf_counter()
        cnt = 0
        while cnt < len(ref[uris[0]]):
            cf_control = {}
            tic = time.perf_counter()
            for i in range(len(drone_bodies)):
                pos_tmp = np.array([data.feedback_QTM[drone_bodies[i]][0:6]]).T
                yaw_tmp = data.feedback_QTM[drone_bodies[i]][6] * np.pi/180
                v = cfcontrol.compute_control(vref[uris[i]][cnt, :].reshape(-1, 1), pos_tmp, ref[uris[i]][cnt].reshape(-1,1), Kf[uris[i]])
                cf_control[uris[i]] = [cfcontrol.get_cf_input(v, yaw_tmp, T_coeff=T_coeff[i],alpha=alpha[i], mass=m[i])]
                controls[drone_bodies[i]] = cfcontrol.get_real_input(v,yaw_tmp)
            swarm.parallel_safe(apply_control, args_dict=cf_control)
            # swarm.sequential(apply_control, args_dict=cf_control)
            
            output_data = save_plot.collect_data(output_data,drone_bodies,data.feedback_QTM,controls,time.perf_counter()-start)
            cfqtm.accurate_delay((Ts-(time.perf_counter()-tic))*1000)
            # time.sleep((Ts-(time.perf_counter()-tic)))   # either using sleep() function or accurate_delay() function
            cnt=cnt+1


        ####################################
        save_plot.save_data(data_destination="./Data_Drone/PX_data_drone_{id_file}.npy".format(id_file=simid),output_data=output_data,ref=ref)
        
        # Landing function : reduce the thrust gradually to zero
        print("Landing...")
        
        last_thrust = {}
        for i in range(len(drone_bodies)):
            pos_tmp = np.array([data.feedback_QTM[drone_bodies[i]][0:6]]).T
            yaw_tmp = data.feedback_QTM[drone_bodies[i]][6] * np.pi/180
            v = cfcontrol.compute_control(vref[uris[i]][cnt-1, :].reshape(-1, 1), pos_tmp, ref[uris[i]][cnt-1].reshape(-1,1), Kf[uris[i]])
            cf_control[uris[i]] = cfcontrol.get_cf_input(v, yaw_tmp, T_coeff=T_coeff[i],alpha=alpha[i], mass=m[i])
            last_thrust[uris[i]] = [cf_control[uris[i]][3],pos_tmp[2]]

        swarm.parallel_safe(land,last_thrust)


        print("Finish.")
        data.close()