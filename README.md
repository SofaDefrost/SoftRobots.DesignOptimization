# Design Optimization Toolkit using SOFA Framework for Simulation
[![Documentation](https://img.shields.io/badge/doc-on_website-blue.svg)](TODO)
[![SOFA](https://img.shields.io/badge/SOFA-on_github-orange.svg)](https://github.com/SofaDefrost/sofa) 

This software toolkit contains components for exploring parametric design of any SOFA scene.

We provide a unified framework for implementing a single parametric Sofa scene and use it both for multi-objective design optimization and model-based control. 

Please refer to the following documentation for installation and more details.

## Soft Finger Geometry Parametric Design by Scripting 
This toolkit is provided with the example of the optimization of a soft finger parametric design.

This example illustrates how to couple heuristic search and automatic mesh generation for efficiently exploring the soft finger geometry. We also introduce scripts for automatically generating the molds necessary for manufacturing a given design.

A dedicated tutorial could be found at this adress: *in comming*

## Requirements
This toolbox was tested with Gmsh 4.9.3 [1] and SOFA v22.06 [2] including the SoftRobots-plugin (v22.06) [3] for Linux. 
The toolbox also requires installing optuna 2.8.0 [4] and pandas [5] for some specific visualization.

[1] [https://gmsh.info/]

[2] [https://github.com/sofa-framework/sofa]

[3] [https://github.com/SofaDefrost/SoftRobots]

[4] [https://github.com/optuna/optuna] 

[5] [https://github.com/pandas-dev/pandas]

You can use "pip" to install the packages [1], [4] and [5] with the specified version, e.g. ```$pip install gmsh==4.9.3"```.

## Get Started

In this section we introduce some commands to use the toolbox with the SensorFinger settings. To use these commands, first open a command prompt in the project directory, then type the command provided bellow.

#### How to choose a problem
The choice of the model and problem to optimise is perform through the arguments "-n *model_name* -rp *id_reduced_problem*". For the same model, several reduced configurations can be implemented considering different designs or objectives. Such a reduced configuration is chosen via the "-rp" argument. For the rest of this tutorial we will work with the reduced configuration 0 of the SensorFinger model.

#### Simulate the Baseline Design

```python3 main.py -n SensorFinger -sd -ba```

#### Sensitivity Analysis 
The following command aims to measure the local impact of each design variable on the optimization objectives.

```python3 main.py -n SensorFinger -rp 0 -sa -nsa 2 -p```

The integer provided to the argument "nsa" is the number of point to sample for each design variables. 
The argument "p" indicates that we want to plot the results.

#### Design Optimization

```python3 main.py -n SensorFinger -rp 0 -o -ni 100 -p```

The integer provided to the argument "ni" is the number of design optimization iterations.
The argument "p" indicates that we want to plot the results.

##### Multithreading Feature
To start a multithreaded design optimization, simply run the design optimizaiton command in several different terminals. 
The number of design optimization iterations "ni" provided is then the number of iterations for each process.

#### Simulate a Design obtained through Optimization
For selecting and visualizing any design encountered during design optimization in SOFA GUI, consider the two following commands: 
* ```python3 main.py -n SensorFinger -rp 0 -sd -be``` for selecting one of the best designs. A command prompt will ask you which design on the Paretto front you want to visualize. 

* ```python3 main.py -n SensorFinger -rp 0 -sd -fo``` for selecting any design. A command prompt will ask you the design id.


## Author
[Team DEFROST (INRIA/CRISTAL), Lille](https://team.inria.fr/defrost/)

## Citing
If you use the project in your work, please consider citing it with:

@misc{navarro2023open,
      title={An Open Source Design Optimization Toolbox Evaluated on a Soft Finger}, 
      author={Stefan Escaida Navarro and Tanguy Navez and Olivier Goury and Luis Molina and Christian Duriez},
      year={2023},
      eprint={2304.07260},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
