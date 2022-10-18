from geometry_msgs.msg import Pose, PoseArray, Quaternion
from .pf_base import PFLocaliserBase
import math
import rospy

from .util import rotateQuaternion, getHeading
import random

from time import time


# test push with pycharm

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

        self.ODOM_ROTATION_NOISE = 0.05  # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0.05  # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0.05 # Odometry model y axis (side-to-side) noise

        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20  # Number of readings to predict

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

        #    # samples w/ pose in self.particlecloud
        #    # get weight of each pose - self.sensor_model.get_weight()
        #    # resample w/ new poses (resampling wheel)
        #    # add noise to new samples
        #    # replace old cloud w/ new one.
        #    """
        #    #Resampling wheel:
        #
        #
        # print(self.particlecloud)
        #    self.particlecloud.poses
        #    new_particle_cloud = []
        #
        #    #to get the individual poses weights
        #
        #    amount_of_poses = len(self.particlecloud.poses)
        #
        #    weights = []
        #    for i in range(amount_of_poses):
        #        pose = self.particlecloud.poses[i]
        #        weights.append(self.sensor_model.get_weight(pose))
        #
        #
        #    # self.sensor_model.get_weight(scan[i], pose[i]) compares scan to the pose
        #
        #
        #
        #    u = 0
        #    index = random.randint(1,amount_of_poses)
        #    while len(new_particle_cloud) < amount_of_poses:
        #        u = u + random.uniform(0,2*max(weights))
        #        while weights[index] < u: # while u > weight of current index
        #            u = u - weights[index]
        #            index = (index + 1)
        #        new_particle_cloud.append(self.particlecloud[index])
        #    self.particlecloud = new_particle_cloud
        #    print(self.particlecloud)
        #    """
        #
        # systematic resampling algorithm


        amount_of_poses = len(self.particlecloud.poses)
        commulutive_weights = []

        weights = []

        for i in range(amount_of_poses):
           pose = self.particlecloud.poses[i]
           weights.append(self.sensor_model.get_weight(scan, pose))

        commulutive_weights.append(weights[0])
        for x in range(1, self.NUMBER_PREDICTED_READINGS):
           commulutive_weights.append(commulutive_weights[x - 1] + weights[x])

        threshold = random.uniform(0, 1 / self.NUMBER_PREDICTED_READINGS)

        i = 0


        cloud_to_return = []


        for count in range(0, self.NUMBER_PREDICTED_READINGS):
           while threshold > commulutive_weights[i]:
               i = i + 1
           # add noise
           self.particlecloud.poses[i].position.x = random.gauss(self.particlecloud.poses[i].position.x, (
                       self.particlecloud.poses[i].position.x * self.ODOM_DRIFT_NOISE))
           self.particlecloud.poses[i].position.y = random.gauss(self.particlecloud.poses[i].position.y, (
                       self.particlecloud.poses[i].position.y * self.ODOM_TRANSLATION_NOISE))
           self.particlecloud.poses[i].orientation = rotateQuaternion(self.particlecloud.poses[i].orientation,
                                                                      math.radians(
                                                                          random.uniform(-self.ODOM_ROTATION_NOISE,
                                                                                         self.ODOM_ROTATION_NOISE)))

           cloud_to_return.append(self.particlecloud.poses[i])
           threshold = threshold + 1 / self.NUMBER_PREDICTED_READINGS

        self.particlecloud.poses = cloud_to_return

        #
        # # samples w/ pose in self.particlecloud
        # # get weight of each pose - self.sensor_model.get_weight()
        # # resample w/ new poses (resampling wheel)
        # # add noise to new samples
        # # replace old cloud w/ new one.
        #
        # # Resampling wheel:
        # print(self.particlecloud)
        # S2 = []
        # weights = self.sensor_model.get_weight(self.particlecloud)
        # M = len(self.particlecloud)
        # u = 0
        # index = random.randint(1, M)
        # while len(S2) < len(self.particlecloud):
        #     u = u + random.uniform(0, 2 * max(weights))
        #     while weights[index] < u:  # while u > weight of current index
        #         u = u - weights[index]
        #         index = (index + 1)
        #     S2.append(self.particlecloud[index])
        # self.particlecloud = S2
        # print(self.particlecloud)
        # """
        # This should use the supplied laser scan to update the current
        # particle cloud. i.e. self.particlecloud should be updated.
        # :Args:
        #     | scan (sensor_msgs.msg.LaserScan): laser scan to use for update
        #  """

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

        pass

        result = Pose()

        return result