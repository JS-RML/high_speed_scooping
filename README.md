# High-Speed Scooping

## 1. Overview
This repository contains the implementation of **High-Speed Scooping**, which refers to the task of picking up thin objects rapidly by interacting with the environment through a direct-driven gripper. A stable pinch grasp configuration can be obtained by the scooping technique, which addresses the limitation of [**Smack and Snatch**](https://www.youtube.com/watch?v=xnHtb0XP3U4&ab_channel=ManipulationLab) that is unstable for grasping relatively thin objects, for example, plastic cards.

### *High-Speed Scooping*
<p align = "center">
<img src="media/scoop_domino.gif" width="400"> 
<img src="media/scoop_card.gif" width="400"> 
</p>

### *Smack and Snatch*
<p align = "center">
<img src="media/snatch_domino.gif" width="400"> 
<img src="media/snatch_card.gif" width="400"> 
</p>

## 2. Prerequisites
### 2.1 Hardware
- [**Universal Robot UR10**](https://www.universal-robots.com/products/ur10-robot/): Industrial Robot Arm 
- [**Direct-Drive Hand (DDH)**](https://github.com/HKUST-RML/ddh_hardware): Custom BLDC-actuated gripper reproduced from the paper: [Direct Drive Hands](http://www.roboticsproceedings.org/rss15/p53.pdf)

### 2.2 Software
The codes are implemented with **python3**.

Install python3 and dependencies:
```
sudo apt install python3 python3-pip
sudo pip3 install urx odrive jupyter
```

## 3. Run High-Speed Scooping
1. Start a Jupyter Notebook server in terminal `jupyter notebook`.
2. Run `scooping_test.ipynb` through the Jupyter Notebook web interface.