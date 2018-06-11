# Estimation Project #

The purpose of the estimation project is to implement a realistic estimator to make our quadcopter, equiped with noisy sensors, follow a real path.


## Project Outline ##

 - [Step 1: Sensor Noise](#step-1-sensor-noise)
 - [Step 2: Attitude Estimation](#step-2-attitude-estimation)
 - [Step 3: Prediction Step](#step-3-prediction-step)
 - [Step 4: Magnetometer Update](#step-4-magnetometer-update)
 - [Step 5: Closed Loop + GPS Update](#step-5-closed-loop--gps-update)
 - [Step 6: Adding Your Controller](#step-6-adding-your-controller)



### Step 1: Sensor Noise ###
#### Set the sensors noise ####
For the sake of this first step, a bunch of GPS and Accelerometers datas were provided to us. The aim of this part was to reacalculate the standard deviation from this datas recorded over 10 seconds at a frequency of 10Hz for the GPS and a lot more for the accelerometer. As the standard deviation follows a gaussian distribution, the calculated standard deviation was suppose to capture ~68% of the sensor mesurements. The STDs calculated we added it to the config/6_Sensornoise.txt file, linked the MeasuredStdDev_GPSPosXY and MeasuredStdDev_AccelXY values.

![noise example](images/noise.gif)


### Step 2: Attitude Estimation ###
#### Implement the non linear complementary filter for accelerometer and gyroscope sensors ####
To estimate the attitude we use a complementary filter. To do so and to make our attitude estimate as accurate as possible we need to use to mesurmeent of attitude, one whose come from the accelerometer and the other from the gyros. Neither of this mesurements alone can provide a reliable and responsive attitude estimate but taking together they do. There are two ways to implement the complementary filter. The linear approach which allows only smalls angle when for example the drone is close from hovering. The non-linear approach is trickier and this is the one we implemented in the project. This approach allows the drone to work for any attitude. As we get the rate in the body frame we first need to translate them to the inertial frame. We could use Eulers angle as a good representation of world frame angle but we chose the quaternions which are more robusts. We used the methods FromEuler123_RPY and .IntegrateBodyRate which make it easier to work with this attitude representation. Nevertheless the transformation matrix can be found in "Representing attitude: Euler angles, unit quaternions, and rotation vectors." from John Diebel

What does FromEuler123_RPY function do:

![euler_to_quaternions](images/euler_to_quaternions.PNG)

How to get Pitch and roll back from quaternions:

![quaternions_to_euler](images/quaternions_to_euler.PNG)

Using this predicated estimate we can then uptade the estimated attitude:

![attitude_update_from_IMU](images/attitude_update_from_IMU.PNG)

![attitude_estimation](images/attitude_estimation2.gif)


### Step 3: Prediction Step ###

#### Implement the prediction step of the filter ####

The prediction step consists in implementing the transition function and then to take the Jacobian to solve the non-linearity and update the covariance matrix. We are working with 3 fuctions: PredictState which update the first member of the transition function, GetRbgPrime which participate in building the Jacobian matrix, PredictState to predict the state forward using the output of GetRbgPrime to build the gPrime which is the Jacobian matrix itself.

![predict_state](images/predict_state.gif)

### Step 4: Magnetometer Update ###

Up until now we've only used the accelerometer and gyro for our state estimation.  In this step, you will be adding the information from the magnetometer to improve your filter's performance in estimating the vehicle's heading.

1. Run scenario `10_MagUpdate`.  This scenario uses a realistic IMU, but the magnetometer update hasn’t been implemented yet. As a result, you will notice that the estimate yaw is drifting away from the real value (and the estimated standard deviation is also increasing).  Note that in this case the plot is showing you the estimated yaw error (`quad.est.e.yaw`), which is drifting away from zero as the simulation runs.  You should also see the estimated standard deviation of that state (white boundary) is also increasing.

2. Tune the parameter `QYawStd` (`QuadEstimatorEKF.txt`) for the QuadEstimatorEKF so that it approximately captures the magnitude of the drift, as demonstrated here:

![mag drift](images/mag-drift.png)

3. Implement magnetometer update in the function `UpdateFromMag()`.  Once completed, you should see a resulting plot similar to this one:

![mag good](images/mag-good-solution.png)

***Success criteria:*** *Your goal is to both have an estimated standard deviation that accurately captures the error and maintain an error of less than 0.1rad in heading for at least 10 seconds of the simulation.*

**Hint: after implementing the magnetometer update, you may have to once again tune the parameter `QYawStd` to better balance between the long term drift and short-time noise from the magnetometer.**

**Hint: see section 7.3.2 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj) for a refresher on the magnetometer update.**


### Step 5: Closed Loop + GPS Update ###

1. Run scenario `11_GPSUpdate`.  At the moment this scenario is using both an ideal estimator and and ideal IMU.  Even with these ideal elements, watch the position and velocity errors (bottom right). As you see they are drifting away, since GPS update is not yet implemented.

2. Let's change to using your estimator by setting `Quad.UseIdealEstimator` to 0 in `config/11_GPSUpdate.txt`.  Rerun the scenario to get an idea of how well your estimator work with an ideal IMU.

3. Now repeat with realistic IMU by commenting out these lines in `config/11_GPSUpdate.txt`:
```
#SimIMU.AccelStd = 0,0,0
#SimIMU.GyroStd = 0,0,0
```

4. Tune the process noise model in `QuadEstimatorEKF.txt` to try to approximately capture the error you see with the estimated uncertainty (standard deviation) of the filter.

5. Implement the EKF GPS Update in the function `UpdateFromGPS()`.

6. Now once again re-run the simulation.  Your objective is to complete the entire simulation cycle with estimated position error of < 1m (you’ll see a green box over the bottom graph if you succeed).  You may want to try experimenting with the GPS update parameters to try and get better performance.

***Success criteria:*** *Your objective is to complete the entire simulation cycle with estimated position error of < 1m.*

**Hint: see section 7.3.1 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj) for a refresher on the GPS update.**

At this point, congratulations on having a working estimator!

### Step 6: Adding Your Controller ###

Up to this point, we have been working with a controller that has been relaxed to work with an estimated state instead of a real state.  So now, you will see how well your controller performs and de-tune your controller accordingly.

1. Replace `QuadController.cpp` with the controller you wrote in the last project.

2. Replace `QuadControlParams.txt` with the control parameters you came up with in the last project.

3. Run scenario `11_GPSUpdate`. If your controller crashes immediately do not panic. Flying from an estimated state (even with ideal sensors) is very different from flying with ideal pose. You may need to de-tune your controller. Decrease the position and velocity gains (we’ve seen about 30% detuning being effective) to stabilize it.  Your goal is to once again complete the entire simulation cycle with an estimated position error of < 1m.

**Hint: you may find it easiest to do your de-tuning as a 2 step process by reverting to ideal sensors and de-tuning under those conditions first.**

***Success criteria:*** *Your objective is to complete the entire simulation cycle with estimated position error of < 1m.*


## Tips and Tricks ##

 - When it comes to transposing matrices, `.transposeInPlace()` is the function you want to use to transpose a matrix

 - The [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj) document contains a helpful mathematical breakdown of the core elements on your estimator

## Submission ##

For this project, you will need to submit:

 - a completed estimator that meets the performance criteria for each of the steps by submitting:
   - `QuadEstimatorEKF.cpp`
   - `config/QuadEstimatorEKF.txt`

 - a re-tuned controller that, in conjunction with your tuned estimator, is capable of meeting the criteria laid out in Step 6 by submitting:
   - `QuadController.cpp`
   - `config/QuadControlParams.txt`

 - a write up addressing all the points of the rubric

## Authors ##

Thanks to Fotokite for the initial development of the project code and simulator.
