## Project: Kinematics Pick & Place

The goal of this project is to write code to perform Forward and Inverse Kinematic for the Kuka KR210:

* Setup environment properly
* Perform Kinematic Analysis of the robot and derive equations for individual joint angles.
* Implement Kinematic code in `IK_server.py`

[//]: # (Image References)

[image1]: ./misc_images/fk_1.png
[image2]: ./misc_images/fk_2.png
[image3]: ./misc_images/fk_3.png
[image4]: ./misc_images/fk_4.png
[image5]: ./misc_images/fk_5.png
[image6]: ./misc_images/fk_6.png
[image7]: ./misc_images/fk_7.png
[image8]: ./misc_images/dh-transform.png
[image9]: ./misc_images/dh-transform-matrix.png
[image10]: ./misc_images/dh-transform-0-ee.png
[image11]: ./misc_images/dh_parameters.png
[image12]: ./misc_images/homogeneous_trans.png
[image13]: ./misc_images/r_corr.png
[image14]: ./misc_images/wc.png
[image15]: ./misc_images/theta1.gif
[image16]: ./misc_images/theta2_theta3.png
[image17]: ./misc_images/r3_6.png
[image18]: ./misc_images/extrinsic_rotation.png
[image19]: ./misc_images/alpha.png
[image20]: ./misc_images/beta.png
[image21]: ./misc_images/gamma.png
[image22]: ./misc_images/kuka.png

---

![alt text][image22]


### One time Gazebo setup step:
Check the version of gazebo installed on your system using a terminal:
```sh
$ gazebo --version
```
To run projects from this repository you need version 7.7.0+
If your gazebo version is not 7.7.0+, perform the update as follows:
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```

Once again check if the correct version was installed:
```sh
$ gazebo --version
```
### For the rest of this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly

If you do not have an active ROS workspace, you can create one by:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the **src** directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Kinematics-Project.git
```

Now from a terminal window:

```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ sudo chmod +x target_spawn.py
$ sudo chmod +x IK_server.py
$ sudo chmod +x safe_spawner.sh
```
Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```

Add following to your .bashrc file
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models

source ~/catkin_ws/devel/setup.bash
```

For demo mode make sure the **demo** flag is set to _"true"_ in `inverse_kinematics.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch

In addition, you can also control the spawn location of the target object in the shelf. To do this, modify the **spawn_location** argument in `target_description.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch. 0-9 are valid values for spawn_location with 0 being random mode.

You can launch the project by
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh
```

If you are running in demo mode, this is all you need. To run your own Inverse Kinematics code change the **demo** flag described above to _"false"_ and run your code (once the project has successfully loaded) by:
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ rosrun kuka_arm IK_server.py
```
Once Gazebo and rviz are up and running, make sure you see following in the gazebo world:

	- Robot
	
	- Shelf
	
	- Blue cylindrical target in one of the shelves
	
	- Dropbox right next to the robot
	

If any of these items are missing, report as an issue.

Once all these items are confirmed, open rviz window, hit Next button.

To view the complete demo keep hitting Next after previous action is completed successfully. 

Since debugging is enabled, you should be able to see diagnostic output on various terminals that have popped up.

The demo ends when the robot arm reaches at the top of the drop location. 

There is no loopback implemented yet, so you need to close all the terminal windows in order to restart.

In case the demo fails, close all three terminal windows and rerun the script.



### Kinematic Analysis

#### Step 1. Sketch the robot arm in its zero configuration

1. Label joints from 1 to n
![alt text][image1]

2. Define joint axes
![alt text][image2]

3. Label links from 0 to n
![alt text][image3]

4. Define common normals and reference frame origins
![alt text][image4]

5. Add gripper frame: it's an extra frame, represents the point on the end effector that we actually care about. It differs from frame 6 only by a translation in z6 direction.
![alt text][image5]

#### Step 2. Derive DH parameters

![alt text][image11]

Definitions of the four DH parameters:

* **α** (twist angle): the angle between **Zi-1** and **Zi** measured about **Xi-1** in a right hand sense.
* **a** (link length): the distance from **Zi-1** to **Zi** measred along **Xi-1** where **Xi-1** is perpendicular to both.
* **d** (link offset): the signed distance from **Xi-1** to **Xi** measured along **Zi**.
* **θ** (joint angle): the angle between **Xi-1** and **Xi** measured about **Zi** in a right hand sense.

I first identified the location of each non-zero link lengths **a** and the link offsets **d**:

![alt text][image6]

I obtained the values for **a** and **d** from the URDF file at `/kuka_arm/urdf/kr210.urdf.xacro`. It wasn't a straight forward task because:

1. The joint origin in the URDF file are not consistent with the frame origins created in accordance with the DH parameter convention, nor do they have the same orientation:
![alt text][image7]
2. Each joint is defined relative to its parent.

Then I obtained th twist angles **alpha** from observing the relationship of each **Zi-1** and **Zi** pair:

Z(i), Z(i+1) | α(i) 
--- | ---
Z(0) ll Z(1) | 0
Z(1) ⟂ Z(2)| -pi/2
Z(2) ll Z(3) | 0 
Z(3) ⟂ Z(4) | -pi/2
Z(4) ⟂ Z(5) | pi/2
Z(5) ⟂ Z(6) | -pi/2
Z(6) ll Z(G) | 0


Here is the DH parameter table:

Links | α(i-1) | a(i-1) | d(i-1) | θ(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | -pi/2 | 0.35 | 0 | q2 - pi/2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.50 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | q7

#### 2. Using the DH parameter table to create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

For each link there are four individual transforms, 2 rotations and 2 translations:

![alt text][image8]

In matrix form this transform is:

![alt text][image9]

In addition to individual transforms, I determined the transfromation between `base_link` and the `end_effector` by pre-multiplying(intrinsic transformations) all individual transforms together:

![alt text][image10]

Then I determined a complete homogeneous transfrom between `base_link` and the `gripper_link` using the end-effector pose (position + rotation):

![alt text][image12]

where Px, Py, Pz represent the position of end-effector w.r.t. base_link and RT represent the rotation part. I constructed RT using the Roll-Pitch-Yaw angles of the end-effector (given by the simulator).

The URDF model does not follow the DH convention, I used additional rotations - a rotation about z-axis of 180° followed by a rotation about the y-axis of -90° - in the calculation to compensate for the difference:

![alt text][image13]


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The last three joints of the robot are revolute and their joint axes intersect at a single point at `joint_5` - it's a **spherical wrist** with `joint_5` being its **wrist center**.

I kinematically decoupled the IK problem into two steps: **Inverse Position** and **Inverse Orientation**.

##### Inverse Position
The goal of this step is to find the first 3 joint angles using the end effector's position in Cartesian coordinates. 

The **spherical wrist** involving `joints 4,5,6`, the position of the **wrist center** is governed by the first three joints. I used the complete transformation matrix derived above to find the position of the **wrist center**:

![alt text][image14]

where Px, Py, Pz represent the position of end-effector w.r.t. base_link and d represents the displacement between **wrist center** and **gripper** along the z-axis, which is **dG** in the graph below and the values are defined in the URDF file.

Once I have the **wrist center** position, I used trigonometry to find the values for the first three joint angles. **theta1** is straightforward by looking from above to the robotic arm:

![alt text][image15]

The following diagram depicts **theta2** and **theta3**:

![alt text][image16]

From the DH parameters I calculated the distance between each joint and then used Cosine Laws to calculate **theta2** and **theta3**.

##### Inverse Orientation
The goal is to find the values of the final three joint angles.

Using the values of the first joint angles obtained above, I calculated **R0_3** via the application of homogeneous transformations up to the WC. Then I find the rotation matrix between joint 3 and joint 6:

![alt text][image17]

where **R0_6** is the homogeneous RPY rotation matrix calculated above from the `base_link` to `gripper_link`. 

**R3_6** is the rotation matrix of the extrinsic X-Y-Z rotation sequence account for the end gripper from the wrist center::

![alt text][image18]

where **R_XYZ** is **R3_6**, and **alpha**, **beta**, **gamma** is **theta4**, **theta5**, **theta6**.

Here are the formulas I used to calculate the last three angles:

![alt text][image19]

![alt text][image20]

![alt text][image21]


### Project Implementation

I implemented Forward Kinematics in lines 31 to 94 in `IK_server.py`, and Inverse Kinematics in lines 117 to 149.

To speed up the Inverse Kinematics calulation, I pre-caluclated some values in the Cosine Laws formula for angle a and b. I've also limited the cos values to be between -1 and 1 to avoid complex numbers. I did this in lines 131 to 141. 

In the inverse orientation step, I transposed the matrix **R3_6** instead of invert it. I did this because inverting a matrix is complex and can be numerically unstable, and I could do this because the rotation matrices are orthogonal and its tranpose is the same as its inverse.

I've limited the angles to be under the limits indicated in the URDF file. 

Before doing the real implementation in `IK_server.py`, I tested my implementation with `IK_debug.py`. Here are the test results:

##### Test Case 1
```
Total run time to calculate joint angles from pose is 7.7479 seconds

Wrist error for x position is: 0.00000046
Wrist error for y position is: 0.00000032
Wrist error for z position is: 0.00000545
Overall wrist offset is: 0.00000548 units

Theta 1 error is: 0.00093770
Theta 2 error is: 0.00178633
Theta 3 error is: 0.06990386
Theta 4 error is: 0.06376055
Theta 5 error is: 0.04193932
Theta 6 error is: 0.08872932

**These theta errors may not be a correct representation of your code, due to the fact            
that the arm can have muliple positions. It is best to add your forward kinmeatics to            
confirm whether your code is working or not**
 

End effector error for x position is: 0.01352825
End effector error for y position is: 0.01030426
End effector error for z position is: 0.10665275
Overall end effector offset is: 0.10800000 units 
```
##### Test Case 2

```
Total run time to calculate joint angles from pose is 8.0994 seconds

Wrist error for x position is: 0.00002426
Wrist error for y position is: 0.00000562
Wrist error for z position is: 0.00006521
Overall wrist offset is: 0.00006980 units

Theta 1 error is: 3.14309971
Theta 2 error is: 0.27927962
Theta 3 error is: 1.94030206
Theta 4 error is: 3.11289370
Theta 5 error is: 0.03462632
Theta 6 error is: 6.20633831

**These theta errors may not be a correct representation of your code, due to the fact            
that the arm can have muliple positions. It is best to add your forward kinmeatics to            
confirm whether your code is working or not**
 

End effector error for x position is: 0.05348588
End effector error for y position is: 0.05381796
End effector error for z position is: 0.07685628
Overall end effector offset is: 0.10800000 units 

```

##### Test Case 3

```
Total run time to calculate joint angles from pose is 8.6296 seconds

Wrist error for x position is: 0.00000503
Wrist error for y position is: 0.00000512
Wrist error for z position is: 0.00000585
Overall wrist offset is: 0.00000926 units

Theta 1 error is: 0.00136747
Theta 2 error is: 0.00329800
Theta 3 error is: 0.07536755
Theta 4 error is: 6.29703249
Theta 5 error is: 0.04694006
Theta 6 error is: 6.34109408

**These theta errors may not be a correct representation of your code, due to the fact            
that the arm can have muliple positions. It is best to add your forward kinmeatics to            
confirm whether your code is working or not**
 

End effector error for x position is: 0.08352294
End effector error for y position is: 0.01287626
End effector error for z position is: 0.06724671
Overall end effector offset is: 0.10800000 units 

```

### Future Works

* Use NumPy instead of SymPy to speed up calculation
