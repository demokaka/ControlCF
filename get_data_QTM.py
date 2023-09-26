import asyncio
import xml.etree.ElementTree as ET
import pkg_resources
import numpy as np
import math
import qtm
import threading
import time 

"""
This script provides the class StreamingData6DEULER(thread) which play the role
as a communication link between QTM software to the computer that recuperates the 
6DOF data of all the rigid bodies defined. 

"""

def create_body_index(xml_string):
    """ Extract a name to index dictionary from 6dof settings xml """
    xml = ET.fromstring(xml_string)
    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[body.text.strip()] = index

    return body_to_index

def accurate_delay(delay):
    ''' Function to provide accurate time delay in millisecond
    '''
    _ = time.perf_counter() + delay/1000
    while time.perf_counter() < _:
        pass

class StreamingData6DEULER(threading.Thread):
    """
    When initialize an instance of this class, it starts a new thread to capture
    the streaming data from QTM

    Usage:
        data = StreamingData6DEULER(ip_addr,drone_bodies)
        - ip_addr       : IP address of the QTM software computer. 
        - drone_bodies  : List of the rigid bodies that we want to capture

        Ex: 
        ip_addr = "192.168.1.146"
        drone_bodies = ['DroneE5','DroneE7','DroneE8','DroneE9']
        data = StreamingData6DEULER(ip_addr,drone_bodies)
        data.start() # start the thread 

    """
    def __init__(self, IP_addr, list_of_bodies):
        threading.Thread.__init__(self)
        self.IP = IP_addr
        self.list_of_bodies = list_of_bodies
        self.currentPos = {}
        self.currentVel = {}
        self.prePosition = {}
        self.connection = None
        self._stay_open = True
        self.nb_bodies = 0
        self.body_index = None
        self.time_stamp = []
        self.feedback_QTM = {}
        self.finish_initial = False

    def run(self):
        asyncio.run(self._life_cycle())

    def close(self):
        self._stay_open = False
        self.join()


    async def _connect(self):
        self.connection = await qtm.connect(self.IP)
        if self.connection is None:
            return
        xml_string = await self.connection.get_parameters(parameters=["6d"])
        self.body_index = create_body_index(xml_string)

    async def _life_cycle(self):
        await self.get_initial_position()
        await self.streamdata()
        while(self._stay_open):
            await asyncio.sleep(1)
        await self._close()

    async def _close(self):
        await self.connection.stream_frames_stop()
        self.connection.disconnect()

    async def get_initial_position(self):
        await self._connect()
        first_packet = await self.connection.get_current_frame(components=["6deuler"])
        info, bodies = first_packet.get_6d_euler()
        self.time_stamp = first_packet.timestamp

        self.nb_bodies = info.body_count
        for i in range(len(self.list_of_bodies)):
            wanted_body = self.list_of_bodies[i]
            if wanted_body is not None and wanted_body in self.body_index:
                wanted_index = self.body_index[wanted_body]
                position, rotation = bodies[wanted_index]
                if(not np.isnan(position.x)):
                    self.prePosition[wanted_body] = [position.x/1000,position.y/1000,position.z/1000,rotation.a3]
                else:
                    print("Some problem with the camera. Cannot detect all the drones. Calibration needed.")
                    return

        self.finish_initial = True

    async def streamdata(self):
        
        await self.connection.stream_frames(components=["6deuler"], on_packet=self.on_packet)
    
    def on_packet(self, packet):
        info, bodies = packet.get_6d_euler()
        for i in range(len(self.list_of_bodies)):
            wanted_body = self.list_of_bodies[i]
            if wanted_body is not None and wanted_body in self.body_index:
                wanted_index = self.body_index[wanted_body]
                position, rotation = bodies[wanted_index]
                if(not np.isnan(position.x)):
                    self.currentPos[wanted_body] = [position.x/1000,position.y/1000,position.z/1000,rotation.a3]
                    self.currentVel[wanted_body] =  1000000 * (np.array(self.currentPos[wanted_body][0:3])  - np.array(self.prePosition[wanted_body][0:3] ))/(packet.timestamp-self.time_stamp)

                    self.prePosition[wanted_body] = self.currentPos[wanted_body]
                    self.feedback_QTM[wanted_body] = self.currentPos[wanted_body][0:3] + self.currentVel[wanted_body].tolist() + [self.currentPos[wanted_body][3]]
                else:
                    pass
                    # print("Some problem with the camera. Cannot detect all the drones. Calibration needed.")
        self.time_stamp = packet.timestamp


def test_Thread_access(Tsim,t_sampling,data):
    """
    A simple function for testing functionality of the StreamingData6DEULER() class
    """
    nb_steps = int(Tsim / t_sampling)
    print("Number of steps : ",nb_steps)
    accurate_delay(500)
    tic = time.time()    
    for i in range(nb_steps+1):
        t_current = time.time()
        # print("Position = ",data.feedback_QTM['DroneE6'])
        # print("Position = ",data.feedback_QTM['DroneE8'])     
        # print("Step i = {} , Time : {}".format(i,t_current-tic))  
        accurate_delay((t_sampling - (time.time()-t_current))*1000)

    

 
if __name__=='__main__':

    
    # drone_bodies = ['DroneE5','DroneE7','DroneE8','DroneE9'] 
    drone_bodies = ['DroneE8']
    data = StreamingData6DEULER("192.168.1.146",drone_bodies)
    data.start()

    # Wait until being connected to the QTM software and receive initial information
    while(not data.finish_initial):
        accurate_delay(1000)

    accurate_delay(500) # Delay 500ms to wait for the connection and initialization of data stream of QTM
    # Test multithread:
    t = threading.Thread(target=test_Thread_access,args=(10,0.1,data))
    t.start()
    
    t.join()
    data.close()

