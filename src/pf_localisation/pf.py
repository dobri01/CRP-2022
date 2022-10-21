from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point
from .pf_base import PFLocaliserBase
import math
import rospy
from tf.msg import tfMessage

from .util import rotateQuaternion, getHeading
import random

import time

import numpy.ma as ma

from geometry_msgs.msg import Twist

from queue import PriorityQueue

"""
kflddkfls

"""

# Data processing
import pandas as pd
import numpy as np

from sklearn.cluster import KMeans
from sklearn.metrics import silhouette_score
from sklearn.cluster import AgglomerativeClustering


# #
# # np.random.bit_generator = np.super()._bit_generator
#
# # Visualization
# import matplotlib.pyplot as plt
# import seaborn as sns
# # Dataset
# from sklearn import datasets
# # Dimensionality reduction
# from sklearn.decomposition import PCA
# # Modeling
# from sklearn.cluster import KMeans
# import scipy.cluster.hierarchy as hier
# from sklearn.mixture import GaussianMixture
# # Number of clusters
# from sklearn.metrics import silhouette_score


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

        self.ODOM_ROTATION_NOISE = 45
        self.ODOM_TRANSLATION_NOISE = 0.004
        self.ODOM_DRIFT_NOISE = 0.004

        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 100  # Number of readings to predict

        self.MY_MAP_STATES = []

    # NIAM + TOM

    def initialise_particle_cloud3(self, initialpose):

        '''map data cloud'''
        # return the dots
        # map is occupancy_map a grid of all locations

        # how do we spread it out on a map we don't know

        # for now set the noise/ standard deviation to 1

        map = self.create_map_states()

        poseArray = PoseArray()

        temp = []

        self.display("called initiialise")
        for count in range(self.NUMBER_PREDICTED_READINGS):
            temp.append(self.pick_from_map())

        poseArray.poses = temp

        return poseArray

    def pick_from_map(self):
        particle = Pose()
        # pick random element from list
        position = random.choice(self.MY_MAP_STATES)
        # self.display(position.x)
        particle.position.x = position.x
        particle.position.y = position.y
        return particle

    def initialise_particle_cloud2(self, initialpose):

        '''particle cloud with the graph lines we must stay within'''

        # return the dots
        # map is occupancy_map a grid of all locations

        # how do we spread it out on a map we don't know

        # for now set the noise/ standard deviation to 1
        noise = 10

        poseArray = PoseArray()

        temp = []

        self.display("called initiialise")
        for count in range(self.NUMBER_PREDICTED_READINGS):
            particle = Pose()
            particle.position.x = random.gauss(initialpose.pose.pose.position.x, noise)
            while (-14 > particle.position.x) or (13 < particle.position.x):
                particle.position.x = random.gauss(initialpose.pose.pose.position.x, noise)

            particle.position.y = random.gauss(initialpose.pose.pose.position.y, noise)
            while ((particle.position.y < -0.958 * (particle.position.x + 16) + 2.766) or (
                    particle.position.y < 1.22 * (particle.position.x + 16) - 40) or (
                           particle.position.y > 0.84395 * (particle.position.x + 16) + 2.125) or (
                           particle.position.y > -1.0365 * (particle.position.x + 16) + 27.646)):
                particle.position.y = random.gauss(initialpose.pose.pose.position.y, noise)

            particle.orientation = rotateQuaternion(initialpose.pose.pose.orientation, random.uniform(0, 2 * math.pi))
            temp.append(particle)

        poseArray.poses = temp

        return poseArray

    def initialise_particle_cloud(self, initialpose):

        '''particle cloud we used initially'''

        self.MY_MAP_STATES = self.create_map_states()
        noise = 0.2

        poseArray = PoseArray()

        temp = []

        self.display("called initiialise")
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

    def create_map_states(self):
        """Create list of set of points that lie in the map"""
        index = 0
        map_states = []
        for h in range(0, self.sensor_model.map_height):
            for w in range(0, self.sensor_model.map_width):
                if (self.sensor_model.map_data[index] >= 0.0):
                    p = Point()
                    p.x = w * self.sensor_model.map_resolution + self.occupancy_map.info.origin.position.x
                    p.y = h * self.sensor_model.map_resolution + self.occupancy_map.info.origin.position.y
                    p.z = 0
                    map_states.append(p)
                index += 1

        return map_states

    def update_particle_cloud(self, scan):

        # 5th of them randomly around

        self.listener()

        cloud = PoseArray()
        cloud = self.particlecloud
        commulutive_weights = []

        weights = []
        weight_sum = 0

        weight_poses = []

        # create weight pose array and also add weight to the priority queue
        queue = PriorityQueue()
        for j in range(self.NUMBER_PREDICTED_READINGS):
            pose = cloud.poses[j]
            weight = self.sensor_model.get_weight(scan, pose)
            weight_pose = (weight, (pose.position.x, pose.position.y, pose.orientation))
            weight_poses.append(weight_pose)
            queue.put(weight_pose)
            weight_sum += weight

        number_of_weights_to_remove = int(0 * self.NUMBER_PREDICTED_READINGS)
        number_of_poses = self.NUMBER_PREDICTED_READINGS - number_of_weights_to_remove

        # get poses with smallest weights
        # and remove those poses from weight list
        # also makes new random poses to fill these gaps

        poses_to_return = []

        for k in range(number_of_weights_to_remove):
            combination = queue.get()
            weight_sum -= combination[0]
            weight_poses.remove(combination)
            part = self.pick_from_map()
            part.orientation.z = random.uniform(0, math.radians(360))
            part.orientation.w = random.uniform(0, math.radians(360))
            self.display(part)

            poses_to_return.append(part)

        # for i in range(self.NUMBER_PREDICTED_READINGS):
        #     pose = cloud.poses[i]
        #     weight = self.sensor_model.get_weight(scan, pose)
        #     weight_sum += weight
        #     weights.append(weight)

        commulutive_weights.append(weight_poses[0][0] / weight_sum)

        # remove a 5th of the least most likely positions
        for x in range(1, number_of_poses):
            weight_by_sum = weight_poses[x][0] / weight_sum
            commulutive_weights.append(commulutive_weights[x - 1] + weight_by_sum)

        threshold = random.uniform(0, 1 / number_of_poses)

        i = 0

        for count in range(0, number_of_poses):
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
            threshold = threshold + (1 / number_of_poses)

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

    def convert(self, lst):
        xs = []
        ys = []

        for particle in self.particlecloud.poses:
            xs.append(particle.position.x)
            ys.append(particle.position.y)

        return {'x values': xs, 'y values': ys}

    def biggest_number_index(self, lst):
        biggest_number_index = [0, -1]
        for i in range(len(lst)):
            if lst[i] > biggest_number_index[0]:
                biggest_number_index[0] = lst[i]
                biggest_number_index[1] = i

        return biggest_number_index

    def estimate_pose1(self):

        # convert particlecloud into dictionary
        dict = self.convert(self.particlecloud.poses)

        particle_dataframe = pd.DataFrame(data=dict)

        # self.display(particle_dataframe)

        # Create an empty dictionary for the Silhouette score
        s_score = {}
        # Loop through the number of clusters

        biggest = -1
        best_k = -1
        i = 2
        for i in range(2, 11):  # Note that the minimum number of clusters is 2
            # Fit kmeans clustering model for each cluster number
            kmeans = KMeans(n_clusters=i, random_state=0).fit(particle_dataframe)
            # Make prediction
            classes = kmeans.predict(particle_dataframe)
            # Calculate Silhouette score
            s_score[i] = (silhouette_score(particle_dataframe, classes))
            # Print the Silhouette score for each cluster number
            if float(s_score[i]) > biggest:
                biggest = float(s_score[i])
                best_k = i

            print(f'The silhouette score for {i} clusters is {s_score[i]:.3f}')

        print(best_k)

        # Kmeans model
        kmeans = KMeans(best_k, random_state=42)
        # Fit and predict on the data
        y_kmeans = kmeans.fit_predict(particle_dataframe)
        # Save the predictions as a column
        particle_dataframe['y_kmeans'] = y_kmeans
        # Check the distribution
        particle_dataframe['y_kmeans'].value_counts()

        # Hierachical clustering model
        # hc = AgglomerativeClustering(i)
        # # Fit and predict on the data
        # y_hc = hc.fit_predict(particle_dataframe)

        # self.display(y_hc)

        # Save the predictions as a column
        # particle_dataframe['y_kmeans'] = y_kmeans
        # Check the distribution
        # self.display(self.particlecloud.poses)
        # self.display("47832476294763924735635628562386582365283659238659238467")
        # self.display(particle_dataframe.info())

        # self.display(particle_dataframe[particle_dataframe.y_hc == 0])
        # self.display(particle_dataframe[particle_dataframe.y_hc == 1])

        products_list = particle_dataframe.values.tolist()
        self.display(products_list)
        clusters = 3

        totals = list(particle_dataframe['y_kmeans'].value_counts())

        biggest_number_index = self.biggest_number_index(totals)

        # self.display(biggest_number_index[1])

        # get particles in biggest cluster
        # self.display("sdk")
        # self.display(list(particle_dataframe[particle_dataframe.y_hc == biggest_number_index[1]]))

        # particles_to_average = [position for position in products_list if position[2] == biggest_number_index[1]]

        indexes_to_average = []

        # for p in range(self.NUMBER_PREDICTED_READINGS):
        #     if products_list[p][2] == biggest_number_index[1]:
        #         indexes_to_average.append(p)

        # retrieve each list of results from particlelist

        # store then with their index

        list_list = []

        # calculate standard deviation of plots and try that

        for w in range(best_k):
            list_list.append(([[], []], []))

        self.display(list_list)

        for x in range(self.NUMBER_PREDICTED_READINGS):
            list_list[int(products_list[x][2])][0][0].append(products_list[x][0])
            list_list[int(products_list[x][2])][0][1].append(products_list[x][1])

            list_list[int(products_list[x][2])][1].append(x)

        self.display(list_list)

        just_positions = []
        devs = []

        for f in range(len(list_list)):
            just_positions.append((list_list[f][0]))
            self.display("\n" + str(just_positions[f]) + "\n")
            self.display(np.std(just_positions[f]))
            numpydev = np.array(just_positions[f][0])
            devs.append(np.std(numpydev))


        biggest = -1
        biggest_index = -1
        for m in range(len(devs)):
            if devs[m] > biggest:
                biggest_index = m
                biggest = devs[m]

        for z in range(self.NUMBER_PREDICTED_READINGS):
            if products_list[z][2] == biggest_index:
                indexes_to_average.append(z)












        # a = np.array([[1, 2], [3, 4]])
        # self.display(np.std(a))


        # self.display("\n" + str(dev) + "\n")










        # particles_to_average = [position for position in products_list if position[2] == biggest_number_index[1]]

        # x = 0
        # y = 0
        # z = 0
        # orx = 0
        # ory = 0
        # orz = 0
        # orw = 0
        # count = len(particles_to_average)
        #
        # for particle in particles_to_average:
        #     x += particle.position.x
        #     y += particle.position.y
        #     z += particle.position.z
        #     orx += particle.orientation.x
        #     ory += particle.orientation.y
        #     orz += particle.orientation.z
        #     orw += particle.orientation.w
        #
        # result = Pose()
        #
        # result.position.x = x / count
        # result.position.y = y / count
        # result.position.z = z / count
        #
        # result.orientation.x = orx / count
        # result.orientation.y = ory / count
        # result.orientation.z = orz / count
        # result.orientation.w = orw / count

        x = 0
        y = 0
        z = 0
        orx = 0
        ory = 0
        orz = 0
        orw = 0
        count = len(indexes_to_average)

        for index in indexes_to_average:
            x += self.particlecloud.poses[index].position.x
            y += self.particlecloud.poses[index].position.y
            x += self.particlecloud.poses[index].position.z
            orx += self.particlecloud.poses[index].orientation.x
            ory += self.particlecloud.poses[index].orientation.y
            orz += self.particlecloud.poses[index].orientation.z
            orw += self.particlecloud.poses[index].orientation.w

            self.display(index)

        result = Pose()

        result.position.x = x / count
        result.position.y = y / count
        result.position.z = z / count

        result.orientation.x = orx / count
        result.orientation.y = ory / count
        result.orientation.z = orz / count
        result.orientation.w = orw / count

        return result
