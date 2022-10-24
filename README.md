# Variables

## Sensor noise variables:
```python
self.ODOM_ROTATION_NOISE = 15
self.ODOM_TRANSLATION_NOISE = 0.004
self.ODOM_DRIFT_NOISE = 0.004
``` 
## Initialising particle cloud variables
```python
self.NUMBER_PREDICTED_READINGS = 200
noise = 0.2 #how spread out is the gaussian cloud
fraction_to_be_random = 0.5 #particles initialised at random locations %
```
## Update particle cloud
```python
updated_particles = 0.4 #percentage of particles to get added
removed_particles = 40 #particles to get removed 
```

# Other functions
``` python
#The basic Gaussian distribution initialisation
def initialise_particle_cloud_ours(self, initialpose)
#Spreads particles all around a predefined map 
def initialise_particle_cloud_graph(self, initialpose)
#Spread particles randomly all around the map
def initialise_particle_cloud_map(self, initialpose):

#Basic pose estimation based on medium of the positions
def estimate_pose_medium(self):
#Initial pose estimation based on clustering
def estimate_pose_cluster(self):
```

# Performance analysis and decisions 
We will test performance based on the accuracy of the localisation after a predefined robot movement of about 10s. The blue dot represents the actual robot position and with red the particles.

As a baseline we'll use the AMCL localisation node in it's default settings:

## AMCL localisation

![[Pasted image 20221024004739.png|Correct Pose Estimation - Initialisation |500]]
![[Pasted image 20221024004524.png|Correct Pose Estimation - After Basic Movement |500]]

![[Pasted image 20221024004941.png|Incorrect Pose Estimation - Initialisation |500]]
![[Pasted image 20221024005225.png|Incorrect Pose Estimation - Basic Movement |500]]

![[Pasted image 20221024005704.png|Incorrect Pose Estimation - Prolonged movement | 500]]

## Our project
### Last version - specified parameters

![[Pasted image 20221024012547.png|Correct Pose Estimation - Initialisation |500]]

![[Pasted image 20221024012716.png|Correct Pose Estimation - Basic Movement |500]]

![[Pasted image 20221024012921.png|Incorrect Pose Estimation - Initialisation |500]]

![[Pasted image 20221024012957.png|Incorrect Pose Estimation - Basic movement |500]]

After just a bit more movement the second cluster disappears and the localisation is accurate. Same localisation can be achieved if we wait in the same spot for a few seconds because of the adaptive particle cloud.

To get to this point and have good performance even with an incorrect pose estimation we had to implement **clustering** and **adaptive particles updating** which randomly adds new particles on top of the old ones and then removes outliers. 

## Design choices

### Parameters - Odometry sensor noise

 We found the sensor noise variables very important in the performance of our algorithm. If we have a very accurate input, as shown in the beginning of the document, it performs well but if we have a very inaccurate input given the following parameters as an example the algorithm can't approximate the position of the robot at all.
```python
self.ODOM_ROTATION_NOISE = 30
self.ODOM_TRANSLATION_NOISE = 0.4
self.ODOM_DRIFT_NOISE = 0.4
```

![[Pasted image 20221024015424.png|Different Parameters, Basic Movement, Correct Pose Estimation|500]]

If we try again but with more realistic sensor input noise we get a better result.
```python
self.ODOM_ROTATION_NOISE = 5
self.ODOM_TRANSLATION_NOISE = 0.05
self.ODOM_DRIFT_NOISE = 0.05
```
Because of the limited movement and the symmetric map the particles don't manage to focus enough and the particle updating has a high chance of dropping the right cluster.

![[Pasted image 20221024020354.png|A bit more noise, Basic Movement, Correct Pose Estimation|500]]

To fix that we can adjust the particle updating variables and reduce the noise in the initial pose estimation. Also because of the high noise our clustering function doesn't have enough density to work properly so we switched back to **estimate_pose_medium**. We also could have increased the number of particles but the performance is very poor so we settled on 200 particles.
```python
updated_particles = 0.1 #percentage of particles to get added
removed_particles = 10 #particles to get removed 
fraction_to_be_random = 0.3 #particles initialised at random locations %
```
![[Pasted image 20221024022519.png|Same noise, Basic Movement, Correct Pose Estimation, no clustering|500]]

### Initialising particle cloud methods
In the beginning of the project we had a simple gaussian initialisation **initialise_particle_cloud_ours** around the Estimate Point which worked great when the point was accurate but quickly degraded if it wasn't. 
![[Pasted image 20221024024715.png|Incorrect Pose Estimation - Initialisation, initialise_particle_cloud_ours|500]]

Because of the new update function, the really focused initialisation is not as detrimental in a wrong estimate scenario as new particles get generated at random locations, so the result is similar to our first experiment.

![[Pasted image 20221024024738.png|Incorrect Pose Estimation - Basic Movement, initialise_particle_cloud_ours |500]]

To tackle the kidnapped robot problem we looked at multiple ways of initialising the particles. We wrote **initialise_particle_cloud_graph** in order to spread the particles evenly inside the map but that proved ineffective as the particles never managed to focus properly. To do it we first tried to do it specifically for this map using a bit of math but then we did it properly using a function that recognises if the particle is inside the map or not (**initialise_particle_cloud_map**).

![[Pasted image 20221024030910.png|Spread out particles initialisation|500]]
![[Pasted image 20221024031109.png|Spread out particles initialisation - Basic Movement|500]]

Because of that we settled for something in the middle, an initialisation around the pose estimate, with a bit of random noise randomly distributed. In the latest function we can set how noisy the gaussian distribution is and how many random particles around it we want. Because we are adding random particles on top we settled for a low noise and 50% random particles to find the robot if the initialisation is not correct. We could adjust the random percentage but it will not change anything as the updating of the particles will either focus on them or disregard them.

