<h1 align="center">Sparkle</h1>

<h3 align="center">ROS package enabling multi-vehicle simulation at large-scale</h3>

<h4 align="center">
<img src="https://github.com/jmscslgroup/sparkle/blob/master/sparkle_new.png" alt="Strym Logo" align="center" width=600/>
</h4>


# Components of Sparkle API

## `layout` class
`layout` class is a baseclass for creating a simulation. `layout` constructor takes following argument:

* X: an array of x-coordinate of vehicles wrt world frame
* Y: an array of y-coordinate of vehicles wrt world frame
* Yaw: an array of yaw-angle of vehicles wrt world frame
* n_vehicles: nmber of vehicles to spawn in the simulation
* max_update_rate: maximum update for the physics engine
* time_step: time step of the physics engine for the simulation
* update_rate: Update rate at which each vehicle publishes its state
* log_time: How long to record the data from simulation using `rosbag record`
* description: A description of simulation for housekeeping purposes

### Attributes of `layout` class

* package_path: Package path where the Sparkle package is
* callflag: A dictionary to keep track for calls to various functions and utilities
* bagdir: A directory where bagfiles will be stored
* gzstatsfile: File path of simulation statistics file

## `lane` class
`lane` class is a subclass of `layout` class that creates a platoon of vehicles in a straightline separated by equal distance in the beginning. Following parameters are important for `lane` class:

* n_vehicles: The number of vehicles to simulate
* vehicle_spacing: Initial spacing between vehicles
* include_laser: A bool array specifying whether to include front mounted laser on each car or not

