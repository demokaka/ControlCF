import numpy as np
import matplotlib.pyplot as plt


"""
    Record the simlation  
"""

def rms(X):
    y = np.sqrt(np.mean(X ** 2))
    return y


def collect_data(output_data, list_of_bodies, current_info, controls, time):
    # current_info : {'DroneEi':[xi,yi,zi,vxi,vyi,vzi,yawi]} 
    # controls : control signal output
    # 

    
    for i in range(len(list_of_bodies)):
        output_data[list_of_bodies[i]] = np.append(output_data[list_of_bodies[i]],
                                        [np.hstack((current_info[list_of_bodies[i]][0:6], 
                                        np.array(controls[list_of_bodies[i]]).flatten(), 
                                        current_info[list_of_bodies[i]][6],time))])    
    return output_data

def save_data(data_destination, output_data,ref):
    data = {'result': output_data,'parameter':ref}
    np.save(data_destination, data)





