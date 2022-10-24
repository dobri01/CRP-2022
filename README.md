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
#Spreads particles all around the map
def initialise_particle_cloud_graph(self, initialpose)
#Basic pose estimation based on medium of the positions
def estimate_pose_medium(self):
#Initial pose estimation based on clustering
def estimate_pose_cluster(self):

```

# Performance
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
