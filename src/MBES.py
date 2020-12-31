#!/usr/bin/env python

import rospy
import re
import time
import numpy as np
from project11 import geodesic
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import Point32
from geographic_msgs.msg import GeoPointStamped
from marine_msgs.msg import NavEulerStamped
from copy import deepcopy
#from marine_msgs.msg import GeoPoseNavEulerStamped
import math

# beam angle of 120 degrees
tan_half_swath_angle = math.tan(math.radians(120/2.0))
beam_count = 20


class MBES:
    ''' A Multibeam Echo Sounder Simulator.

    Managed by the "MBES_Server" a MBES object simulates a multibeam sonar
    by returning "pings" of data from a gridded bathy surface when provided position
    and orientation of the vehicle.

    Eventually different types of sonars will be supported and specified
    with the "mbesType" arguement. The default implements a basic sonar
    with only standard commands start() and stop().

    When started and when postion and heading data have been recently received, ping data will
    be streamed at the maximum rate possible given the simulated two-way travel
    time of the sonar's outer beams, interpolating vehicle positions between updates.

    Vehicle pitch and roll are currently ignored.

    Commands between the MBES_Server and the MBES are facilitate by the CMD argument
    which is a multiprocessing.Array() character array (max size 128), of the form
    "topicid:COMMAND". All simulated MBES listen to this channel, hence the topicid
    identifier.
    '''

    def __init__(self, topicid, CMD, bathyData, bathyDataShape, lock, Nbeams=256, mbesType="default",  mapOrigin=None):
        # Identfier and data publisher.
        self.topicid = topicid
        
        #rospy.init_node("SIM_MBES",anonymous=True)

        # Sonar model parameters
        self.Nbeams = 256
        self.sonarType = mbesType
        self.ssp = None  # TODO

        # Vessel pose information.
        self.position = None
        self.orientation = None
        self.last_position = None
        self.last_orientation = None
        self.secondssincelastposition = None
        self.secondssincelastheading = None
        self.pingrateHz = 1.

        self.test_latitude = None

        # Sonar status parameters.
        self.CMD = None
        self.isRunning = False
        self.isPinging = False
        self.timeOut = 10.

        # Bathymetric Information
        self.bathyDataShape = bathyDataShape
        self.bathyData = bathyData
        self.bathyDataX = None
        self.bathyDataY = None


    def setupPublishersandSubscribers(self):

        # Data publiser
        self.ping_publisher = rospy.Publisher(self.topicid+'/mbes', PointCloud2, queue_size=10)
        self.depth_publisher = rospy.Publisher(self.topicid+'/depth', Float32, queue_size=5)

        # Position and heading subscribers.
        #positiontopic = re.sub('\/(.*)', '/position', self.topicid)
        #headingtopic = re.sub('\/(.*)', '/heading', self.topicid)
        positiontopic = self.topicid + "/position"
        headingtopic = self.topicid + "/heading"
        print("MBES(%s): Subscribing to: %s" % (self.topicid,positiontopic))
        print("MBES(%s): Subscribing to: %s" % (self.topicid,headingtopic))
        self.position_subscriber = rospy.Subscriber(positiontopic, GeoPointStamped, self.positionGeoPoint_callback)
        self.heading_subscriber = rospy.Subscriber(headingtopic, NavEulerStamped, self.headingNavEuler_callback)

    def tearDownPublishersandSubscribers(self):

        self.position_subscriber.unregister()
        self.heading_subscriber.unregister()

    def start(self):
        # TODO: ADD SERVICE CALL SETTING UP UDPBRDIGE TO LISTEN TO POSITION UPDATES
        self.CMD = "START"

    def stop(self):
        self.CMD = "END"

    def position_callback(self, data):
        '''GeoPoseNavEulerStamped (position and orientation in one)'''
        if self.position is not None:
            self.last_position = deepcopy(self.position)
            self.last_orientation = deepcopy(self.orientation)

        if data.position.latitude != 0 and data.position.longitude != 0:
            self.position = deepcopy(data.position)
            self.orientation = deepcopy(data.orientation)

    def headingNavEuler_callback(self, data):
        '''NavEulerStamped callback (orientation only)'''
        if self.orientation is not None:
            self.last_orientation = deepcopy(self.orientation)
        self.orientation = deepcopy(data.orientation)
        #print(self.last_orientation)

    def positionGeoPoint_callback(self, data):
        '''GeoPointStamped callback (position only)'''
        if self.position is not None:
            self.last_position = deepcopy(self.position)
        if data.position.latitude != 0 and data.position.longitude != 0:
            self.position = deepcopy(data)


    def ping(self):

        print "PING!"
        
        '''Sends a ping of data'''
        # print data
        depth = self.bathyData.getDepthAtLatLon(self.position.position.latitude, self.position.position.longitude)
        print(depth)
        #if self.position.header.stamp - self.last_position.header.stamp > rospy.Duration(0.25):
        if depth is not None and depth >= 0:
            self.depth_publisher.publish(depth)
            if self.orientation.heading is not None:
                swath_half_width = depth * tan_half_swath_angle;
                # print 'swath half width:',swath_half_width
                lon_rad = math.radians(self.position.position.longitude)
                lat_rad = math.radians(self.position.position.latitude)
                port_outer_beam_location = geodesic.direct(lon_rad, lat_rad, math.radians(self.orientation.heading - 90),
                                                            swath_half_width)
                starboard_outer_beam_location = geodesic.direct(lon_rad, lat_rad, math.radians(self.orientation.heading + 90),
                                                                swath_half_width)
                # print 'outer beam locations:',port_outer_beam_location,starboard_outer_beam_location
                port_outer_beam_location_xy = self.bathyData.getXY(math.degrees(port_outer_beam_location[1]),
                                                            math.degrees(port_outer_beam_location[0]))
                starboard_outer_beam_location_xy = self.bathyData.getXY(math.degrees(starboard_outer_beam_location[1]),
                                                                math.degrees(starboard_outer_beam_location[0]))
                dx = (starboard_outer_beam_location_xy[0] - port_outer_beam_location_xy[0]) / float(beam_count)
                dy = (starboard_outer_beam_location_xy[1] - port_outer_beam_location_xy[1]) / float(beam_count)

                dx_re_mbes = 2.0 * swath_half_width / float(beam_count)

                header = Header()
                header.frame_id = 'mbes'
                header.stamp = self.position.header.stamp
                
                fields = []
                fieldidx = 0
                for f in ['x','y','z']:
                    pf = PointField()
                    pf.name = f
                    pf.offset = fieldidx * 32
                    pf.datatype = 7
                    pf.count = 1
                    fieldidx += 1
                    fields.append(pf)
                    
                points = []
                for i in range(beam_count):
                    x = port_outer_beam_location_xy[0] + dx * i
                    y = port_outer_beam_location_xy[1] + dy * i
                    z = self.bathyData.getDepth(x, y)
                    # print 'depth:', z
                    if z is not None:
                        
                        y = -(-swath_half_width + i * dx_re_mbes)
                        z = -z
                        points.append([0, y, z])
                        
                pointCloud = pcl2.create_cloud(header,fields,points)
                self.ping_publisher.publish(pointCloud)

    def updateVehiclePose(self):
        '''Extrapolates the last position and orientation to the current time.'''
        
        #print("LAT: %s" % test_latitude)
        #print("heading")
        #print(self.last_orientation)
        #print("position")
        #print(self.last_position)
        if self.last_orientation == None or self.last_position == None:
            print("MBES_SIM(%s): Both position and orientation have not been received." % self.topicid)
            return
        now = rospy.Time.now()
        deltaT = now.to_sec() - self.last_position.header.stamp.to_sec()

        if deltaT < 0 or deltaT > 10:
            # TODO: Register an error here somehow. Time shouldn't go backward or jump.
            return
        p = self.position.position
        plast = self.last_position.position

        self.position.position.latitude = p.latitiude + deltaT * (p.latitiude -
                                                                          plast.latitude)
        self.position.position.longitude = p.longitude + deltaT * (p.latitude -
                                                                     plast.latitude)
        self.position.position.altitude = p.altitude + deltaT * (p.altitude -
                                                                    plast.altitude)
        self.position.position.header.stamp = now

        deltaT = now.to_sec() - self.last_orientation.header.stamp.to_sec()
        self.orientation.heading = self.orientation.heading + deltaT * (self.orientation.heading -
                                                               self.last_orientation.heading)
        self.orientation.roll = self.orientation.roll + deltaT * (self.orientation.roll -
                                                                  self.last_orientation.roll)
        self.orientation.pitch = self.orientation.pitch + deltaT * (self.orientation.pitch -
                                                                    self.last_orientation.pitch)
        self.orientation.header.stamp = now

    def doCommand(self,command):

        
        if self.sonarType == 'default' or self.sonarType == None or self.sonarType == 'basic':
            if command['action'] == "START_PINGING":
                print("MBES(%s): Received START_PINGING" % self.topicid)
                self.isPinging = True
            elif command['action'] == "STOP_PINGING":
                print("MBES(%s): Received STOP_PINGING" % self.topicid)
                self.isPinging = False
            # if self.exit.is_set():
            elif command['action'] == "EXIT":
                if self.isRunning:
                    print("MBES(%s): Received EXIT" % self.topicid)
                    self.isRunning = False

        if self.sonarType == 'kongsberg_Kctrl':
            pass

    def g_tickDynamic(self):
        '''A generator that determines how much time to sleep to ensure
         a constant rate of execution.

         Modified from
         https://stackoverflow.com/questions/8600161/executing-periodic-actions-in-python
         Modified here to check get the interval period from a class varialbe.
         '''
        t = time.time()
        period = 1./self.pingrateHz
        count = 0
        while True:
            count += 1
            timetoexecute = t + count * period
            timetosleeep = max(t + count * period - time.time(), 0)
            yield timetoexecute, timetosleeep

    def do_everyDynamic(self, f, *args):
        '''A method that executes a function, f, with arguments, *args, at a precise interval

        Modified from:
        https://stackoverflow.com/questions/8600161/executing-periodic-actions-in-python
        '''
        g = self.g_tickDynamic()
        ctr = 0
        print("MBES %s is running." % self.topicid)
        while self.isRunning:

            if self.isPinging:
                self.update_time, self.sleeptime = next(g)
                time.sleep(self.sleeptime)
                self.updateVehiclePose()
                f(*args)
                ctr += 1
            else:
                rospy.sleep(.1)

            # Check to see if we've received a command.
            if self.CMD is not None:
                try:
                    fields = self.CMD.split(",")
                    keys = [x.strip().lower() for x in fields[::2]]
                    command = dict(zip(keys, [x.strip() for x in fields[1::2]]))
                    if command['topicid'] == self.topicid:
                        self.doCommand(command)
                except:
                    print("MBES(%s) - Unknown Command: %s" % (self.topicid, self.CMD.value))
                    
                self.CMD = None
        



    def run(self):

        self.isRunning = True
        self.setupPublishersandSubscribers()
        
        ctr = 0
        print("MBES %s is running." % self.topicid)
        while not rospy.is_shutdown() and self.isRunning:
            
            # This will update the loop rate to the ping rate. 
            # as it is changed dynamically. 
            r = rospy.Rate(self.pingrateHz)
        
            # If we're pinging, ping!
            if self.isPinging:
                self.updateVehiclePose()
                self.ping()
                ctr += 1
 

            # Check to see if we've received a command, and execute it.
            if self.CMD is not None:
                try:
                    fields = self.CMD.split(",")
                    keys = [x.strip().lower() for x in fields[::2]]
                    command = dict(zip(keys, [x.strip() for x in fields[1::2]]))
                    if command['topicid'] == self.topicid:
                        self.doCommand(command)
                except:
                    print("MBES(%s) - Unknown Command: %s" % (self.topicid, self.CMD.value))
                    
                self.CMD = None
        
            
            r.sleep()
            
        self.tearDownPublishersandSubscribers()
