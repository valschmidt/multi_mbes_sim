#!/usr/bin/env python

import sys
import rospy
import BathyGrid
import MBES
from multiprocessing import Process, Lock, RawArray
from std_msgs.msg import String

last_depth_time = None
heading = None

# Variable to hold a BathyGrid object.
bathygrid = None

class MBES_Server():
    ''' A Multibeam Echosounder Simulation Server

    This class implements a controller for simulated MBES systems, which
    serve "pings" of data to requesting vessels from a bathymetric grid.

    Control commands from each vessel are published to /multi_mbes_sim_ctrl as strings.
    These may be one of:

    NEW: Creates a new MBES object in the server.
    ---------------------------------------------
    "NEW: <vesselid/index>, <Nbeams>, <MapOriginLat>, <MapOriginLong>, MBEStype"
        veseelid/index: An alpha numeric identifier for the vessel and index for the MBES.
                        Data will be published to this topic name. For example BEN/mbes01
        Nbeams:         The number of beams to simulate.
        MapOriginLat:   Latitude of the origin of the vessels' map origin.
        MapOriginLon:   Longitude of the origin of the vessels's map origin.
        MBEStype:       Determines type of MBES to simulate, which, in turn, determines
                        the set of commands to interact with it. A type of "DEFAULT"
                        will implement the basic command set below.

    START: Starts a MBES.
    --------------------
    "START: <topicid>"
    topicid:            Identical to, and must match vesselid/index.

    STOP: Stops a MBES
    ------------------
    "STOP: <topicid>"
    topicid:            Identical to, and must match vesselid/index.

    '''
    def __init__(self):
        global bathygrid
        self.MBESs = {}
        self.mbes_ctrl = rospy.Subscriber("/multi_mbes_sim_ctrl",String, self.mbes_ctrl_callback)

        # Variables for multi-processes.
        self.lock = None
        self.MBES_Processes = []

        # Will this work? Create a reference to the global accessible throughout the class?
        self.bathygrid = bathygrid

        # Communicating with MBES Processes.
        self.CMD = None

        print("Listening for new MBES simulation requests on /multi_mbes_sim_ctrl...")

    def mbes_ctrl_callback(self, data):
        '''Callback for MBES simulation commands.

        Command Format:  COMMAND, Key, Value, Key, Value,...

        Example: "Action, NEW_MBES, topicid, /BEN/01,Nbeams, 128, mapOriginLat, 43.0, mapOriginLon, -70, mbesType, basic"

        '''
        fields = data.data.split(',')
        # Remove spaces and make keys lower case.
        keys = [x.strip().lower() for x in fields[::2]]
        #keys = [x.strip().lower() for x in fields]

        # Remove spaces from commands that may have been added for readability.
        command = dict(zip(keys,[x.strip() for x in fields[1::2]]))
        print("Received new command: %s" % data.data)
        print([":".join([k,v]) for k,v in command.items()])
        if command['action'] == "NEW_MBES":

            print("Creating new MBES simulator.")
            while self.bathygrid is None:
                print("No Bathy Data")
                rospy.sleep(1)
                
            M = MBES.MBES(topicid = command['topicid'],
                    CMD = self.CMD, bathyData = self.bathygrid,
                     bathyDataShape = self.bathygrid.data.shape,
                     lock = self.lock,
                     Nbeams = command['nbeams'])

            # The Bathygrid is shared through a multiprocessing.Array() shared data object.
            # But we have to provide some metadata for it. All calculations for the sonar simulation
            # are done in Cartesian coordinates
            # in a local reference frame. So we need the bathy grid in that local reference frame.
            # We'll do the math in the vehicle's map reference frame. To get the bathy data in
            # the vehicle's map reference frame we'll call the setMapOrigin() method of the BathyGrid
            # object. This will set two vectors, BathyDataXXmap and BathyDataYYmap, which contain
            # the coordinates of the rows and columns of the bathy data in the vehicle's map
            # reference frame. Because these don't require a lot of memory, we set variablies
            # in the MBES object to hold these values. They'll be copied and available in the MBES class
            # when we launch the process.
            self.bathygrid.setMapOrigin(mapOriginLatitude = float(command['maporiginlat']),
                                   mapOriginLongitude = float(command['maporiginlon']))

            M.bathyDataX = self.bathygrid.BathyDataXXmap
            M.bathyDataY = self.bathygrid.BathyDataYYmap

            self.MBESs[command['topicid']] = M
            self.MBESs[command['topicid']].run()
            # TODO: ADD SERVICE CALL TO SETUP UDP BRIDGE FOR RETURN DATA TO TOPICID

        else:
            # Send the command to the specified simulator. 
            self.MBESs[command['topicid']].CMD = data.data

    def shutdownMBESServer(self):
        print("")
        print("Terminating MBES Simulation processes.")
        for topicid in self.MBESs:
            S.CMD = "topicid," + topicid + ",Action," + "EXIT"
        for p in self.MBES_Processes:
            #p.join()
            if p is not None:
                p.terminate()




if __name__ == '__main__':

    rospy.init_node('multi_mbes_sim')
    S = MBES_Server()
    S.lock = Lock()
    S.bathygrid = BathyGrid.BathyGrid(sys.argv[1])
    rospy.loginfo("Loaded Bathy grid %s" % sys.argv[1])
    rospy.loginfo("Grid size: %d,%d" % S.bathygrid.data.shape)
    #S.sharedGridData = RawArray('d',S.bathygrid.data.shape[0] * S.bathygrid.data.shape[1])
    rospy.on_shutdown(S.shutdownMBESServer)
    rospy.spin()
