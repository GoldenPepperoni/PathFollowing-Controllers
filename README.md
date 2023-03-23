# Repository for Path following controllers for Final Year Individual Project 2023

This repository is the culmination of path following algorithms for a fixed-wing small UAV (2.5Kg), simulated in [PyFlyt](https://github.com/jjshoots/PyFlyt)


Key features:
- The fixed wing UAV was modelled and simulated as part of [PyFlyt](https://github.com/jjshoots/PyFlyt), more information on the physics and flight model can be found there.
- Path planning was done by generating a Dubin's path for a set of waypoints.
- Carrot chasing and [Non-linear Guidance Law](http://acl.mit.edu/papers/gnc_park_deyst_how.pdf) path following algorithm was implemented to follow the generated path.
- Lower level controls (Elev, Ail, Rud, Thr) were performed by a LQR controller with control references from the path following algorithms.
- Linearised state space matrices of the longitudinal and lateral dynamics used for the LQR controller were obatained from XFLR5 stability analysis.
- Gif generator to visualise simulation in real time
- Plot generator to provide illustration of trajectory, control traces and zero-pole plots.
- Comparison tool for obtaining performance statistics between algorithms, such as tracking error (IAE, ISE and ITSE) and 3D spatial trajectory comparison

## Installation

1. Git clone this repository to a directory of your choice. (Creates a folder named `/PathFollowing-Controllers/`)

```sh
git clone https://github.com/GoldenPepperoni/PathFollowing-Controllers.git
```

2. Go to repository folder

```sh
cd ./PathFollowing-Controllers/
```

3. Install packages, and any other dependencies (such as PyFlyt)

```sh
pip3 install -e ./
```

## Usage

### Quickstart

Each script in `/PathFollowing-Controllers/pathfollowingcontrollers/algorithms/` can be run individually

An example to simulate a Figure 8 flight path guided by the Carrot Chasing algorithm:
```sh
cd ./pathfollowingcontrollers/algorithms/CarrotChasing/
python3 8.py
```



### Spawn the UAV away from the origin (With a different orientation)

This functionality provides the user an option to spawn the UAV away from the origin. This is useful to study how the UAV converges onto the path. Especially useful for comparisons.

By default, the UAV is spawned at [0.0, 0.0, 10.0] for [X, Y, Z] coordinates respectively. To change the spawn location, add a `spawn_pos` keyword argument to the environment creation:

```py
# Create and initialise dubins path env
envs = gymnasium.make("PyFlyt/Fixedwing-CCDubinsPath-v0", spawn_pos=np.array([[10.0, 10.0, 10.0]]),...
```

By default, the UAV is spawned at [0.0, 0.0, 0.0] for rotations around [X, Y, Z] axes respectively. To change the spawn orientation, add a `spawn_orn` keyword argument to the environment creation:

```py
# Create and initialise dubins path env
envs = gymnasium.make("PyFlyt/Fixedwing-CCDubinsPath-v0", spawn_orn=np.array([[0.0, 0.0, 1.0]]),...
```



### Watch simulation in (almost) real time 

By default, rendering is disabled, which allows the simulation to run as fast as possible.

To enable rendering, change `render_mode=None` to `render_mode="human"` in the script:

```py
# Create and initialise dubins path env
envs = gymnasium.make("PyFlyt/Fixedwing-CCDubinsPath-v0", render_mode="human",...
```



### Custom waypoints

To let the UAV follow a custom waypoint, simply edit the `custom_targets` and `custom_yaw_targets` variables in the script.

```py
# Create a custom "S" path
custom_targets = [[50, 100, 10], [50, 200, 10], [50, 300, 10], [50, 400, 10], [50, 500, 10]] # Coordinates of the waypoints
custom_yaw_targets = [0, np.pi, 0, np.pi, 0] # Orientation of the UAV at the waypoints in the XY plane (-pi to pi)

```
3D plot:
<p align="center">
    <img src="/readme_assets/CustomS.png" width="500px"/>
</p>



### Plotting 
By default, plots will not be generated when the scripts were ran. Plots are saved to `/PathFollowing-Controllers/plots/` folder

To enable plots, edit the following in the script:

```py
# Make plots?
makePlots = True
```

A total of 6 plots will be generated:
1. Poles and Zeros plot for the Longitudinal State Matrix
2. Poles and Zeros plot for the Lateral State Matrix
3. Desired vs actual horizontal UAV trajectory (XY plane)
4. Desired vs actual vertical UAV trajectory (XZ plane)
5. 3D interactive plot of the UAV trajectory
6. Control traces [Elev, Ail, Rud, Thr]

### Gif maker
***Please note that enabling gif making will slow down the simulation due to image capture in every frame (Does not affect physics simulation)***

By default, gif making is disabled. To enable, edit the following in the script:
```py
# Make gif?
makePlots = True
```

and enable rendering:

```py
# Create and initialise dubins path env
envs = gymnasium.make("PyFlyt/Fixedwing-CCDubinsPath-v0", render_mode="human",...
```

A GUI will appear that shows the UAV performing in (almost) real time. 

Red sphere represents the Virtual Target Point/Carrot, and green spheres are waypoints.

Example gif of a Figure 8 path guided by the Carrot Chasing algorithm:
<p align="center">
    <img src="/readme_assets/LQR_CC_8.gif" width="500px"/>
</p>



### Comparison Tool

The comparison tool takes in individual `/PathFollowing-Controllers/pathfollowingcontrollers/algorithms/` scripts as command line arguments. There are no limit on the number of algorithms to be compared at a time, designed to be scalable for future algorithm implementations. 

Here is an example on how to use this tool to compare between 2 algorithms, `NLGL/8.py` and `CarrotChasing/8.py`:

```sh
cd ./pathfollowingcontrollers/PF_utils/
python3 compare.py /NLGL/8.py /CarrotChasing/8.py
```

The resulting artifacts:
COMING SOON!!!

























