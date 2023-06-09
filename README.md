# Soft Robots Design Optimization Toolkit using SOFA Framework for Simulation
[![SOFA](https://img.shields.io/badge/SOFA-on_github-orange.svg)](https://github.com/SofaDefrost/sofa) 

This software toolkit contains components for exploring parametric design of any SOFA scene.
We provide a unified framework for implementing a single parametric Sofa scene and use it both for multi-objective design optimization and model-based control. 

This toolkit is provided with the example of the optimization of a soft finger parametric design.
This example illustrates how to couple heuristic search and automatic mesh generation for efficiently exploring the soft finger geometry. We also introduce scripts for automatically generating the molds necessary for manufacturing a given design.


![Alt text](/images/Intro_Toolbox.png)

More examples will be available in the future.

# Table of Contents
1. [Installation](#installation)
2. [Quick Start](#quickstart)
3. [Examples](#examples)
      1. [Design Optimization of a Sensorized Finger](#sensorizedfinger)          


# Installation <a name="installation"></a>

## Python requirements
Python3 is needed to make use of toolbox.
The required basic python libraries can be installed with the following bash command:
```bash
pip install pathlib importlib numpy logging 
```

Additionally, the provided example expects an installation of the following libraries:
* [Gmsh](https://gmsh.info/) for automatic mesh generation.
* [Optuna](https://github.com/optuna/optuna) implementing algorithms for heuristic-based optimization.

Both this libraries can be installed using the following bash command:
```bash
pip install gmsh==4.11.1 optuna==2.10.0
```

## SOFA and mandatory plugins
This toolbox was tested with the SOFA v22.12 installation. 
The following plugins are mandatory:
* [SofaPython3](https://github.com/sofa-framework/SofaPython3)
* [SoftRobots](https://github.com/SofaDefrost/SoftRobots)

Please refer to the following [tutorial](https://www.sofa-framework.org/community/doc/getting-started/build/linux/) for a complete overview of the installation process.



# Quick Start <a name="quickstart"></a>

## Soft Robot Modeling for Design Optimization
A soft robot model is described through a set of different scripts:
* A parametric SOFA scene called after the model name. This scene should also reimplement a Controller class inheriting from BaseFitnessEvaluationController to describe the soft robot's control strategy for evaluating a given parametric design.
* A Config class inheriting from BaseConfig describing the design variables and optimization objectives.

## User Interface
In this section we introduce the main commands for using the toolbox with the SensorFinger example. For testing these commands, first open a command prompt in the project directory, then type the command provided bellow. A list of all available commands can be read in the main.py file.

### Testing a baseline SOFA scene
For running a parametric scene without optimization, the following command is available:
```bash
python3 main.py -n SensorFinger -rp 0 -sd -so ba 
```
- -n, --name: name of the soft robot.
- -rp, --reduced_problem: reference to a reduced configuration of a soft robot. For a same soft robot, several reduced configurations can be implemented considering different design variables or optimization objectives.
- -sd, --simulate_design: call to the simulation script in the SOFA GUI
- -so, --simulation_option: simulation option. For baseline simulation, we have to specify the option "ba" [Optional, default=ba]

### Sensitivity Analysis 
Running a sensitivity analysis for measuring the local impact of each design variable on the optimization objectives is performed through:
```bash
python3 main.py -n SensorFinger -rp 0 -sa -nsa 2
```
- -sa, --sensitivity_analysis: call to the sensitivity analysis script
- -nsa, --n_samples_per_param: Number of point to sample for each design variable [Optional, default=2]

### Design Optimization
Design optimization of a parametric design is launched using:
```bash
python3 main.py -n SensorFinger -rp 0 -o -ni 100
```
- -o, --optimization: call to the design optimization script
- -ni, --n_iter: Number of design optimization iterations [Optional, default=10]

#### Multithreading Feature
To start a multithreaded design optimization, simply run the design optimizaiton command in several different terminals. 
The number of design optimization iterations "ni" provided is then the number of iterations for each process.

### Simulate a Design obtained through Optimization
For selecting and visualizing any design encountered during design optimization in the SOFA GUI, consider the following command: 
```bash
python3 main.py -n SensorFinger -rp 0 -sd -so fo
```
- -so, --simulation_option: simulation option. For choosing a specific design encountered during design optimization, we have to specify the option "fo" [Optional, default=ba]

Once launched, a command prompt will ask you the id of the design to simulate.


# Examples <a name="examples"></a>

## Design Optimization of a Sensorized Finger <a name="sensorizedfinger"></a> 
The Sensorized Finger is a cable actuated soft finger with pneumatic chambers located at the joints. This chambers are used as sensors. The measurement of their volume change enables finding the Sensorized Finger actuation state through inverse modeling. The robot parameterization as well as our results are described in this [article](https://arxiv.org/pdf/2304.07260.pdf). We also provide scripts for automatic mold generation for manufacturing any optimized robot.

![Alt text](/images/SensorizedFingerOverview.png)


# Citing
If you use the project in your work, please consider citing it with:
```bibtex
@misc{navarro2023open,
      title={An Open Source Design Optimization Toolbox Evaluated on a Soft Finger}, 
      author={Stefan Escaida Navarro and Tanguy Navez and Olivier Goury and Luis Molina and Christian Duriez},
      year={2023},
      eprint={2304.07260},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```
