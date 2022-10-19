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
        noise = 1

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

    def callback(self, data):
        self.display("calshshhshhshlback")
        PFLocaliser.moved = True


    def listener(self):
        rospy.wait_for_message("/cmd_vel", Twist, 10)
        # rospy.Subscriber("/cmd_vel", Twist, self.callback)
        self.display("in listener")


    def update_particle_cloud(self, scan):
        self.display("about to enter listener")
        self.listener()

        # self.display("Called particle filter")
        # #
        # self.display("\n\n\n\n\n\n\n\n\n\n\n\n\n")
        # self.display("INITIAL PARTICLE CLOUD")
        # self.display(self.particlecloud.poses)
        # self.display("\n\n\n\n\n\n\n\n\n\n\n\n\n")



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


        cloud = PoseArray()
        cloud = self.particlecloud
        amount_of_poses = len(cloud.poses)
        commulutive_weights = []

        weights = []


        for i in range(amount_of_poses):
            pose = cloud.poses[i]
            weights.append(self.sensor_model.get_weight(scan, pose))

        # self.display("WEIGHTS")
        # self.display(weights)

        commulutive_weights.append(weights[0])
        for x in range(1, self.NUMBER_PREDICTED_READINGS):
            commulutive_weights.append(commulutive_weights[x - 1] + weights[x])

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
                    math.radians(random.uniform(-self.ODOM_ROTATION_NOISE, self.ODOM_ROTATION_NOISE)))

            poses_to_return.append(noisy_pose)
            threshold = threshold + (1 / self.NUMBER_PREDICTED_READINGS)


        # self.display("\n\n\n\n\n\n\n\n\n\n\n\n\n")
        # self.display("CLOUD AFTER ALG")
        # self.display(cloud_to_return)
        # self.display("\n\n\n\n\n\n\n\n\n\n\n\n\n")

        # self.display(poses_to_return)
        cloud_to_return = PoseArray()
        cloud_to_return.poses.extend(poses_to_return)

        self.particlecloud = cloud_to_return


    def update_particle_cloud2(self, scan):

        ##---------------------------##
        ##---------PARAMETERs--------##
        ##---------------------------##
        #list of poses
        sum_weights = 0
        sum_count = 0
        cumulate_weight= []
        separate_weight= []

        cloud_pose = self.particlecloud.poses

        ##---------------------------##
        ##-----------WEIGHT----------##
        ##---------------------------##

        ##remove the invalid value.
        scan.ranges=ma.masked_invalid(scan.ranges).filled(scan.range_max)

        ##record the weight for each pose, in case to use it again
        pairs_partweight=[]
        #time1 = datetime.datetime.now()

        ##weight need scan_data and current_pose
        for i in cloud_pose:
            particle_weight = self.sensor_model.get_weight(scan, i)
            sum_weights+= particle_weight
            pairs_partweight.append([i,particle_weight])
        #time2 = datetime.datetime.now()
        ##calculate the time consume
        #print (time2 - time1 ).microseconds

        for pair in pairs_partweight:
            particle_weight = pair[1]
            weight_over_sum = particle_weight/sum_weights
            sum_count+= weight_over_sum
            cumulate_weight.extend([sum_count])

        #print cumulate_weight
        ##-----------------------##
        ##---------UPDATE--------##
        ##-----------------------##
        updated_particlecloud = PoseArray()
        for particle in cloud_pose:
            count = 0
            rand = random.uniform(0,1)
            for i in cumulate_weight:
                ##TODO:the repeat pose doesn't matters
                if rand <= i:
                    updated_particlecloud.poses.extend([cloud_pose[count]])
                    #print count
                    break
                count=count+1

        ##-----------------------##
        ##---------NOISE---------##
        ##-----------------------##

        updated_with_noise_cloud = PoseArray()

        for i in updated_particlecloud.poses:
            noise_pose = Pose()
            noise_pose.position.x = random.gauss(i.position.x,(i.position.x * self.ODOM_DRIFT_NOISE))
            noise_pose.position.y = random.gauss(i.position.y,(i.position.y * self.ODOM_TRANSLATION_NOISE))
            noise_pose.orientation = rotateQuaternion(i.orientation, math.radians(random.uniform(-self.ODOM_ROTATION_NOISE,self.ODOM_ROTATION_NOISE)))

            updated_with_noise_cloud.poses.extend([noise_pose])

        ##-----------------------##
        ##---------OUTPUT--------##
        ##-----------------------##

        self.particlecloud = updated_with_noise_cloud

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
