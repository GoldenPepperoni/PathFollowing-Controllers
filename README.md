# Repository for Path following controllers for Final Year Individual Project 2023

This repository is the culmination of path following algorithms for a fixed-wing small UAV (2.5Kg), simulated in [PyFlyt](https://github.com/jjshoots/PyFlyt)


Key features:
- The fixed wing UAV was modelled and simulated as part of [PyFlyt](https://github.com/jjshoots/PyFlyt), more information on the physics and flight model can be found there.
- Path planning was done by generating a Dubin's path for a set of waypoints.
- Carrot chasing path following algorithm was implemented to follow the generated path. (NLGL coming soon!)
- Lower level controls (Elev, Ail, Rud, Thr) were performed by a LQR controller with references from the path following algorithms.
- Linearised state space matrices of the longitudinal and lateral dynamics used for the LQR controller were obatained from XFLR5 stability analysis.

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

```sh
cd ./pathfollowingcontrollers/algorithms/
python3 LQR_CC_8.py
```

### Watch simulation in (almost) real time 

By default, rendering is disabled, which allows the simulation to run as fast as possible.

To enable rendering, change `render_mode=None` to `render_mode="human"` in the script:

```py
# Create and initialise dubins path env
envs = gymnasium.make("PyFlyt/Fixedwing-DubinsPath-v0", render_mode="human",...
```

### Plotting 
By default, plots will be generated when the scripts were ran. Plots are saved to `/PathFollowing-Controllers/plots/` folder

To disable plots, edit the following in the script:

```py
# Make plots?
makePlots = False
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
envs = gymnasium.make("PyFlyt/Fixedwing-DubinsPath-v0", render_mode="human",...
```

A GUI will appear that shows the UAV performing in (almost) real time. 

Red sphere represents the Virtual Target Point/Carrot, and green spheres are waypoints.

Example gif:
<p align="center">
    <img src="/readme_assets/LQR_CC_rand.gif" width="500px"/>
</p>




























