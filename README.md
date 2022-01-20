# High-Speed Scooping

## 1. Overview
This repository contains the software implementation of **High-Speed Scooping** using a [direct-drive gripper](https://github.com/HKUST-RML/ddh_hardware). It can be applied to rapidly picking thin objects off from a hard surface, which would be quite challenging with a straightforward approach aiming at directly obtaining a pinch grasp.
<!-- This repository contains the implementation of **High-Speed Scooping**, which refers to the task of picking up thin objects rapidly by interacting with the environment through a direct-drive gripper. Our scooping technique ensures a pinch grasp configuration can be obtained to pick up the object securely, which addresses the limitation of [**Smack and Snatch**](https://www.youtube.com/watch?v=xnHtb0XP3U4&ab_channel=ManipulationLab) that is unstable for grasping relatively thin objects, for example, plastic cards. -->

### *High-Speed Scooping*
<p align = "center">
<img src="media/scoop_domino.gif" width="400"> 
<img src="media/scoop_card.gif" width="400"> 
</p>

### *Direct Pinch Grasping*
<p align = "center">
<img src="media/snatch_domino.gif" width="400"> 
<img src="media/snatch_card.gif" width="400"> 
</p>

## 2. Timeline of High-Speed Scooping
<p align = "center">
<img src="media/hss_process.jpg" width="800"> 
</p>

High-Speed Scooping is executed as follows:
- **t = 0s**: The pose of the gripper and digits is initialized above the object such that a lower angle of attack is attained for the thumb by tilting the gripper.
- **t = 0.75s**: The arm accelerates towards the surface where the height is unknown.
- **t = 1.15s**: The BLDC motors detect the collision through the digit's linkages and the program triggers the deceleration to stop the arm hitting to the surface. Both digits are commanded to close simultaneously with increased stiffness (proportional gain of the motor's position control loop).
- **t = 1.20s**: The finger presses on the object while the thumb slides along the surface and penetrates below the object.
- **t = 1.27s**: The arm reaches zero velocity and accelerates upward. Meanwhile, the stiffness of the fingers increases for the second time to secure the grasp.
- **t = 1.48s**: The arm lifts up from the surface to complete the task. 

<!-- The process of High-Speed Scooping consists of three steps as follows: 
1. Initialize the pre-scooping pose of the gripper and fingers above the object such that the gripper is tilted to attain a lower angle of attack.
2. Accelerate the gripper towards the surface where the height is unknown.
3. Detect the fingers' collision with the BLDC motors and trigger the deceleration to stop the gripper hitting to the surface. Meanwhile, the fingers are commanded to close with increased stiffness (position gain) to scoop up the object during the process of the thumb tip slides along the surface and penetrates below the object. -->


## 3. Prerequisites
### 3.1 Hardware
- [**Universal Robot UR10**](https://www.universal-robots.com/products/ur10-robot/): Industrial Robot Arm 
- [**Direct-Drive Hand (DDH)**](https://github.com/HKUST-RML/ddh_hardware): BLDC-motor-actuated gripper reproduced from the paper: [Direct Drive Hands](http://www.roboticsproceedings.org/rss15/p53.pdf)

### 3.2 Software
Our software is implemented with **python3** and tested on **Ubuntu 16.04**.

To install python3 and dependencies on Linux:
```
sudo apt install python3 python3-pip
sudo pip3 install urx odrive jupyter
```
**Note:** Our software requires a minor modification to the `urx` library for getting UR10 tool speed with the function `get_tcp_speed()`. To do this, replace the original urx scripts with [ansonmak/python-urx](https://github.com/ansonmak/python-urx/tree/master/urx). The path to the original urx package: 
- For local environment: `/usr/local/lib/python3.x/dist-packages/urx`
- For conda environment: `~/anaconda3/envs/<environment-name>/lib/python3.x/site-packages/urx`

## 4. Run High-Speed Scooping
### 4.1 Run with real robot
1. Start a Jupyter Notebook server in terminal `jupyter notebook`.
2. Run `scooping_test.ipynb` through the Jupyter Notebook web interface.

### 4.2 Changing execution parameters
The parameters of High-Speed Scooping can be specified in `config/hss.yaml`. The parameters are as follows:
- **Object Dimension**
    - ***object_length***: object length in the scooping direction (<img src="https://render.githubusercontent.com/render/math?math=mm">)
    - ***object_thickness***: object thickness (<img src="https://render.githubusercontent.com/render/math?math=mm">)
- **Pre-scooping Parameters**
    - ***gripper_tilt***: tilting angle of the gripper ( <img src="https://render.githubusercontent.com/render/math?math=\^\circ"> )
    - ***gripper_height***: initial height of gripper in world frame (<img src="https://render.githubusercontent.com/render/math?math=m">)
    - ***contact_distance***: distance from gripper frame to surface when the fingers are in contact (<img src="https://render.githubusercontent.com/render/math?math=mm">)
    - ***finger_prescoop_position***: dimensionless F position on the object from the scooping edge
    - ***thumb_prescoop_position***: dimensionless prescoop position away from the scooping edge
    - ***gripper_center***: gripper center line dimensionless position on the object from the scooping edge
    - ***finger_stiffness***: position gain of finger before scoop
    - ***thumb_stiffness***: position gain of thumb before scoop
    - ***init_vel***: velocity of tcp when initializing gripper pose (<img src="https://render.githubusercontent.com/render/math?math=m/s">)
    - ***init_acc***: acceleration of tcp when initializing gripper pose (<img src="https://render.githubusercontent.com/render/math?math=m/s^2">)
- **Smack and Scoop Parameters**
    - ***smack_vel***: velocity of tcp when approaching to the surface (<img src="https://render.githubusercontent.com/render/math?math=m/s">)
    - ***smack_acc***: acceleration of tcp when approaching to the surface (<img src="https://render.githubusercontent.com/render/math?math=m/s^2">)
    - ***slow_dist***: distance of gripper slowing down after collision (<img src="https://render.githubusercontent.com/render/math?math=m">)
    - ***lift_vel***: velocity of tcp when lifting the object up from the surface (<img src="https://render.githubusercontent.com/render/math?math=m/s">)
    - ***lift_dist***: distance of gripper lifted off from the surface (<img src="https://render.githubusercontent.com/render/math?math=m">)
    - ***finger_scoop_stiffness***: position gain of finger after collision
    - ***thumb_scoop_stiffness***: position gain of thumb after collision

<!-- ## 4. Background -->


## Maintenance
For any technical issues, please contact Ka Hei Mak (khmakac@connect.ust.hk)
