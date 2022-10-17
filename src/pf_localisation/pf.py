from geometry_msgs.msg import Pose, PoseArray, Quaternion
from . pf_base import PFLocaliserBase
import math
import rospy

from . util import rotateQuaternion, getHeading
import random

from time import time


#test push with pycharm

class PFLocaliser(PFLocaliserBase):

    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()

        # ----- Set motion model parameters
        # ALL NEED TO HAVE A THINK ABOUT THESE
        """
            self.ODOM_ROTATION_NOISE = ???? # Odometry model rotation noise
            self.ODOM_TRANSLATION_NOISE = ???? # Odometry model x axis (forward) noise
            self.ODOM_DRIFT_NOISE = ???? # Odometry model y axis (side-to-side) noise
        """

        self.ODOM_ROTATION_NOISE = 1 # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 1 # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 1 # Odometry model y axis (side-to-side) noise


        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict



    # NIAM + TOM
    def initialise_particle_cloud(self, initialpose):


        # return the dots
        # map is occupancy_map a grid of all locations


        # how do we spread it out on a map we don't know

        # for now set the noise/ standard deviation to 1
        noise = 1.3

        poseArray = PoseArray()

        temp = []

        for count in range(self.NUMBER_PREDICTED_READINGS):
            particle = Pose()
            particle.position.x = random.gauss(initialpose.pose.pose.position.x, noise)
            particle.position.y = random.gauss(initialpose.pose.pose.position.y, noise)

            particle.orientation = rotateQuaternion(initialpose.pose.pose.orientation, random.uniform(0, 2 * math.pi))

            temp.append(particle)

        poseArray.poses = temp

        return poseArray

        """
        Set particle cloud to initialpose plus noise
        

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        # pass
        # testPose = Pose()
        # testPose.position.x = 1
        # testPose.position.y = 1
        #
        # testPose2 = Pose()
        # testPose2.position.x = 10
        # testPose2.position.y = 10
        #
        # poses = []
        # poses.append(testPose)
        # poses.append(testPose2)
        #
        # self.particlecloud.poses = poses
        #
        # return self.particlecloud



    # JOSH
    def update_particle_cloud(self, scan):
    
    #samples w/ pose in self.particlecloud
    #get weight of each pose - self.sensor_model.get_weight()
    #resample w/ new poses (resampling wheel)
    #add noise to new samples
    #replace old cloud w/ new one.
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.

        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """

        return []


    # DOBRI
    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).

        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.

        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """


        result = Pose()

        return result


