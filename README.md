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
updated_particles = 0.45 #percentage of particles to get added
removed_particles = 30 #particles to get removed 
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

<figure>
   <p align="center">
     <img src="https://github.com/dobri01/CRP-2022/blob/a4aecd1e4ad8da4a54a0af3bd11622e1f13f0511/images/Pasted%20image%2020221024004739.png" style="width:50%" />
     </br>
    <em> Correct Pose Estimation - Initialisation </em>
   </p>
</figure>

<figure>
   <p align="center">
     <img src="https://github.com/dobri01/CRP-2022/blob/a4aecd1e4ad8da4a54a0af3bd11622e1f13f0511/images/Pasted%20image%2020221024004524.png" style="width:50%" />
     </br>
    <em> Correct Pose Estimation - After Basic Movement </em>
   </p>
</figure>

<figure>
   <p align="center">
     <img src="https://github.com/dobri01/CRP-2022/blob/a4aecd1e4ad8da4a54a0af3bd11622e1f13f0511/images/Pasted%20image%2020221024004941.png" style="width:50%" />
     </br>
    <em> Incorrect Pose Estimation - Initialisation </em>
   </p>
</figure>

<figure>
   <p align="center">
     <img src="https://github.com/dobri01/CRP-2022/blob/a4aecd1e4ad8da4a54a0af3bd11622e1f13f0511/images/Pasted%20image%2020221024005225.png" style="width:50%" />
     </br>
    <em> Incorrect Pose Estimation - Basic Movement </em>
   </p>
</figure>


<figure>
   <p align="center">
     <img src="https://github.com/dobri01/CRP-2022/blob/a4aecd1e4ad8da4a54a0af3bd11622e1f13f0511/images/Pasted%20image%2020221024005704.png" style="width:50%" />
     </br>
    <em> Incorrect Pose Estimation - Prolonged movement </em>
   </p>
</figure>

<figure>
   <p align="center">
     <img src="https://github.com/dobri01/CRP-2022/blob/b6df5163d4519bb73d6f5427f688a928d6b0aa15/images/Pasted%20image%2020221024110956.png" style="width:50%" />
     </br>
    <em> Correct Pose Estimation - Initialisation - Kidnaped Robot </em>
   </p>
</figure>

<figure>
   <p align="center">
     <img src="https://github.com/dobri01/CRP-2022/blob/b6df5163d4519bb73d6f5427f688a928d6b0aa15/images/Pasted%20image%2020221024111300.png" style="width:50%" />
     </br>
    <em> Correct Pose Estimation - Basic Movement - Kidnaped Robot </em>
   </p>
</figure>

After the robot is kidnaped from the **blue location** and put into the **purple location** the particle cloud is reinitialised and after a bit of movement 2 clusters with possible positions are selected. After a bit of extra movement the correct cluster is selected and the initialisation is correct.

## Our project
### Last version - specified parameters

<figure>
   <p align="center">
     <img src="https://github.com/dobri01/CRP-2022/blob/a4aecd1e4ad8da4a54a0af3bd11622e1f13f0511/images/Pasted%20image%2020221024012547.png" style="width:50%" />
     </br>
    <em> Correct Pose Estimation - Initialisation </em>
   </p>
</figure>

<figure>
   <p align="center">
     <img src="https://github.com/dobri01/CRP-2022/blob/a4aecd1e4ad8da4a54a0af3bd11622e1f13f0511/images/Pasted%20image%2020221024012716.png" style="width:50%" />
     </br>
    <em> Correct Pose Estimation - Basic Movement </em>
   </p>
</figure>

<figure>
   <p align="center">
     <img src="https://github.com/dobri01/CRP-2022/blob/a4aecd1e4ad8da4a54a0af3bd11622e1f13f0511/images/Pasted%20image%2020221024012921.png" style="width:50%" />
     </br>
    <em> Incorrect Pose Estimation - Initialisation </em>
   </p>
</figure>

<figure>
   <p align="center">
     <img src="https://github.com/dobri01/CRP-2022/blob/a4aecd1e4ad8da4a54a0af3bd11622e1f13f0511/images/Pasted%20image%2020221024012957.png" style="width:50%" />
     </br>
    <em> Incorrect Pose Estimation - Basic movement </em>
   </p>
</figure>

<figure>
   <p align="center">
     <img src="https://github.com/dobri01/CRP-2022/blob/b6df5163d4519bb73d6f5427f688a928d6b0aa15/images/Pasted%20image%2020221024110319.png" style="width:50%" />
     </br>
    <em> Correct Pose Estimation - Basic Movement - Kidnaped to a different location </em>
   </p>
</figure>


After just a bit more movement the second cluster disappears and the localisation is accurate. Same localisation can be achieved if we wait in the same spot for a few seconds because of the adaptive particle cloud.

To get to this point and have good performance even with an incorrect pose estimation we had to implement **clustering** and **adaptive particles updating** which randomly adds new particles on top of the old ones, then resamples which will reinforce any correct randomly selected position. The number of particles in the cloud will then be slowly decreased by randomly removing particles in the entire cloud, before reaching a threshold, adding more random samples and resampling again. This process goes on continuously. 
If we kidnap the robot to the purple location after finishing the basic movement, the localisation is quickly recovered after a bit of movement as shown in the simulation. (blue spot)
Our algorithm just after kidnapping doesn't reinitialise the particle cloud and is reliant on the **adaptive particles updating**.

## Design choices

### Parameters - Odometry sensor noise

 We found the sensor noise variables very important in the performance of our algorithm. If we have a very accurate input, as shown in the beginning of the document, it performs well but if we have a very inaccurate input given the following parameters as an example the algorithm can't approximate the position of the robot at all.
```python
self.ODOM_ROTATION_NOISE = 30
self.ODOM_TRANSLATION_NOISE = 0.4
self.ODOM_DRIFT_NOISE = 0.4
```

<figure>
   <p align="center">
     <img src="https://github.com/dobri01/CRP-2022/blob/a4aecd1e4ad8da4a54a0af3bd11622e1f13f0511/images/Pasted%20image%2020221024015424.png" style="width:50%" />
     </br>
    <em> Different Parameters, Basic Movement, Correct Pose Estimation </em>
   </p>
</figure>

If we try again but with more realistic sensor input noise we get a better result.
```python
self.ODOM_ROTATION_NOISE = 5
self.ODOM_TRANSLATION_NOISE = 0.05
self.ODOM_DRIFT_NOISE = 0.05
```
Because of the limited movement and the symmetric map the particles don't manage to focus enough and the particle updating has a high chance of dropping the right cluster.

<figure>
   <p align="center">
     <img src="https://github.com/dobri01/CRP-2022/blob/a4aecd1e4ad8da4a54a0af3bd11622e1f13f0511/images/Pasted%20image%2020221024020354.png" style="width:50%" />
     </br>
    <em> A bit more noise, Basic Movement, Correct Pose Estimation </em>
   </p>
</figure>


To fix that we can adjust the particle updating variables and reduce the noise in the initial pose estimation. Also because of the high noise our clustering function doesn't have enough density to work properly so we switched back to **estimate_pose_medium**. We also could have increased the number of particles but the performance is very poor so we settled on 200 particles.
```python
updated_particles = 0.1 #percentage of particles to get added
removed_particles = 10 #particles to get removed 
fraction_to_be_random = 0.3 #particles initialised at random locations %
```

<figure>
   <p align="center">
     <img src="https://github.com/dobri01/CRP-2022/blob/a4aecd1e4ad8da4a54a0af3bd11622e1f13f0511/images/Pasted%20image%2020221024022519.png" style="width:50%" />
     </br>
    <em> Same noise, Basic Movement, Correct Pose Estimation, no clustering </em>
   </p>
</figure>


### Initialising particle cloud methods
In the beginning of the project we had a simple gaussian initialisation **initialise_particle_cloud_ours** around the Estimate Point which worked great when the point was accurate but quickly degraded if it wasn't. 

<figure>
   <p align="center">
     <img src="https://github.com/dobri01/CRP-2022/blob/a4aecd1e4ad8da4a54a0af3bd11622e1f13f0511/images/Pasted%20image%2020221024024715.png" style="width:50%" />
     </br>
    <em> Incorrect Pose Estimation - Initialisation, initialise_particle_cloud_ours </em>
   </p>
</figure>


Because of the new update function, the really focused initialisation is not as detrimental in a wrong estimate scenario as new particles get generated at random locations, so the result is similar to our first experiment.

<figure>
   <p align="center">
     <img src="https://github.com/dobri01/CRP-2022/blob/a4aecd1e4ad8da4a54a0af3bd11622e1f13f0511/images/Pasted%20image%2020221024024738.png" style="width:50%" />
     </br>
    <em> Incorrect Pose Estimation - Basic Movement, initialise_particle_cloud_ours </em>
   </p>
</figure>

To tackle the kidnapped robot problem we looked at multiple ways of initialising the particles. We wrote **initialise_particle_cloud_graph** in order to spread the particles evenly inside the map but that proved ineffective as the particles never managed to focus properly. To do it we first tried to do it specifically for this map using a bit of math but then we did it properly using a function that recognises if the particle is inside the map or not (**initialise_particle_cloud_map**).

<figure>
   <p align="center">
     <img src="https://github.com/dobri01/CRP-2022/blob/a4aecd1e4ad8da4a54a0af3bd11622e1f13f0511/images/Pasted%20image%2020221024030910.png" style="width:50%" />
     </br>
    <em> Spread out particles initialisation </em>
   </p>
</figure>

<figure>
   <p align="center">
     <img src="https://github.com/dobri01/CRP-2022/blob/a4aecd1e4ad8da4a54a0af3bd11622e1f13f0511/images/Pasted%20image%2020221024031109.png" style="width:50%" />
     </br>
    <em> Spread out particles initialisation - Basic Movement </em>
   </p>
</figure>

Because of that we settled for something in the middle, an initialisation around the pose estimate, with a bit of random noise randomly distributed. In the latest function we can set how noisy the gaussian distribution is and how many random particles around it we want. Because we are adding random particles on top we settled for a low noise and 50% random particles to find the robot if the initialisation is not correct. We could adjust the random percentage but it will not change anything as the updating of the particles will either focus on them or disregard them.

