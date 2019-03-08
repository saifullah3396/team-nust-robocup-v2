# vrep-naoqi-simulation
=======================================================

This respository contains the software for running simulations on vrep 
by providing an interface between vrep opensource simulator and naoqi 
(Aldebaran's Nao software). Using this, you can easily run any naoqi
based code or script on the simulated robot in vrep (:

## Getting Started

### Prerequisites

* Updated version of naoqi-simulator-sdk. 
* V-REP_PRO_EDU_V3_4_0_Linux which can be downloaded from: http://www.coppeliarobotics.com/previousversions.html

### Installing
* For CMake configuration and scripts, set the following environment variables:
```
echo 'export PATH_TO_SIM_DIR=/path/to/simulator-sdk' >> ~/.bashrc 
echo 'export PATH_TO_VREP_DIR=/path/to/vrep' >> ~/.bashrc 
```

* Replace the file naoqi-simulator-sdk/etc/hal/hal.xml with dependencies/hal.xml

## Deployment

### Simulator Startup

For running the code, a simulated naoqi robot must be running. For that use:
```
./scripts/simulated_nao.sh
```

For dynamic simulations in vrep, there are two versions of nao provided in models and can be used with 
 separate scripts. 
 
* The simple_nao_model.ttm is the modified version of 
 vrep provided nao model. Modifications include addition of some sensors
 and fixes to work with the interface. You can use it for simultions
 and it is compatible with all of the available physics engine in vrep.
 However, this model greatly simplified and does not depict the inertial 
 properties of an actual nao (real life robot). This model can be run by:
 
 ```
./scripts/run_simulation.sh <scenes>/simple_nao.ttt
```

* The other model named v50_nao_model_vrep.ttm is a modified version of the 
 same model to have the inertial properties of an actual V50 Nao robot. 
 In addition, this model also has the foot collision mesh similar to an 
 actual Nao instead of a box mesh as provided in the vrep model. 
 However, the use of this model is not compatible with bullet, or ode 
 engines. For this model, only Vortex engine can be used. This model can be used
 by the script:
 
```
./scripts/run_simulation.sh <scenes>/v50_dynamic.ttt
```

Once the scene is loaded, and played, use the following script to start the interface:
```
./scripts/setup_interface.sh
```

## Authors
* <A href="mailto:saifullah3396@gmail.com">Saifullah</A>

## License
BSD

## Acknowledgments
* We acknowledge the adaption of code from the ros-naoqi interface respository
written by Konstantinos Chatzilygeroudis.
* See details here: https://github.com/costashatz/nao_gazebo
