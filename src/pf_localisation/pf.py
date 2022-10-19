from geometry_msgs.msg import Pose, PoseArray, Quaternion
from .pf_base import PFLocaliserBase
import math
import rospy
from tf.msg import tfMessage

from .util import rotateQuaternion, getHeading
import random

import time

import numpy.ma as ma

from geometry_msgs.msg import Twist


# test push with pycharm

class PFLocaliser(PFLocaliserBase):
    moved = False

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

        self.ODOM_ROTATION_NOISE = 5
        self.ODOM_TRANSLATION_NOISE = 0.04
        self.ODOM_DRIFT_NOISE = 0.04

        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 100  # Number of readings to predict

    # NIAM + TOM
    def initialise_particle_cloud(self, initialpose):

        # return the dots
        # map is occupancy_map a grid of all locations

        # how do we spread it out on a map we don't know

        # for now set the noise/ standard deviation to 1
        noise = 10

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

    # JOSH

    def listener(self):
        # waits until message to move is received
        rospy.wait_for_message("/cmd_vel", Twist, 100)

    def update_particle_cloud(self, scan):
        self.listener()

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
        new_particle_cloud = []
        #
        #    #to get the individual poses weights
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

        # cloud = PoseArray()
        # cloud = self.particlecloud
        # amount_of_poses = len(cloud.poses)
        # commulutive_weights = []
        #
        # weights = []
        #
        # new_particle_cloud = []
        #
        # for i in range(amount_of_poses):
        #     pose = cloud.poses[i]
        #     weights.append(self.sensor_model.get_weight(scan, pose))
        #
        # u = 0
        # index = random.randint(1,amount_of_poses)
        # while len(new_particle_cloud) < amount_of_poses:
        #    u = u + random.uniform(0,2*max(weights))
        #    while weights[index] < u: # while u > weight of current index
        #        u = u - weights[index]
        #        index = ((index + 1)) % self.NUMBER_PREDICTED_READINGS
        #    new_particle_cloud.append(cloud.poses[index])
        #
        # returning_particle_cloud = PoseArray()
        # returning_particle_cloud.poses = new_particle_cloud
        #
        # self.particlecloud = returning_particle_cloud
        # #    """
        # #
        # # systematic resampling algorithm

        cloud = PoseArray()
        cloud = self.particlecloud
        amount_of_poses = len(cloud.poses)
        commulutive_weights = []

        weights = []
        weight_sum = 0

        for i in range(amount_of_poses):
            pose = cloud.poses[i]
            weight = self.sensor_model.get_weight(scan, pose)
            weight_sum += weight
            weights.append(weight)

        commulutive_weights.append(weights[0] / weight_sum)
        # self.display(weights[0])
        for x in range(1, self.NUMBER_PREDICTED_READINGS):
            weight_by_sum = weights[x] / weight_sum
            self.display("weightbysum for x = " + str(x) + "\n" + str(weight_by_sum) + "\n")
            commulutive_weights.append(commulutive_weights[x - 1] + weight_by_sum)

        threshold = random.uniform(0, 1 / self.NUMBER_PREDICTED_READINGS)

        i = 0

        poses_to_return = []

        for count in range(0, self.NUMBER_PREDICTED_READINGS):
            while threshold > commulutive_weights[i]:
                i = i + 1
            # add noise
            noisy_pose = Pose()
            noisy_pose.position.x = random.gauss(cloud.poses[i].position.x,
                                                 (cloud.poses[i].position.x * self.ODOM_DRIFT_NOISE))
            noisy_pose.position.y = random.gauss(cloud.poses[i].position.y,
                                                 (cloud.poses[i].position.y * self.ODOM_TRANSLATION_NOISE))
            noisy_pose.orientation = rotateQuaternion(cloud.poses[i].orientation,
                                                      math.radians(random.uniform(-self.ODOM_ROTATION_NOISE,
                                                                                  self.ODOM_ROTATION_NOISE)))

            poses_to_return.append(noisy_pose)
            threshold = threshold + (1 / self.NUMBER_PREDICTED_READINGS)

        # self.display(poses_to_return)
        cloud_to_return = PoseArray()
        cloud_to_return.poses.extend(poses_to_return)

        self.particlecloud = cloud_to_return

    def display(self, message):
        rospy.loginfo(message)

        # DOBRI

    def estimate_pose(self):

        x = 0
        y = 0
        z = 0
        orx = 0
        ory = 0
        orz = 0
        orw = 0
        count = len(self.particlecloud.poses)

        for particle in self.particlecloud.poses:
            x += particle.position.x
            y += particle.position.y
            z += particle.position.z
            orx += particle.orientation.x
            ory += particle.orientation.y
            orz += particle.orientation.z
            orw += particle.orientation.w

        result = Pose()

        result.position.x = x / count
        result.position.y = y / count
        result.position.z = z / count

        result.orientation.x = orx / count
        result.orientation.y = ory / count
        result.orientation.z = orz / count
        result.orientation.w = orw / count

        return result
