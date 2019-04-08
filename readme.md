**Building a Controller Project**

The goals / steps of this project are the following:

* Build a controller in C++
* Tune the parameters 

## [Rubric](https://review.udacity.com/#!/rubrics/1643/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
## Writeup / README


### Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf.  

You're reading it!

---

## Implemented Controller

### Implemented body rate control in C++

It is implemented in function BodyRateControl, lines 107 to 136.

```cpp
 V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  V3F momentCmd;
 V3F err = pqrCmd - pqr;
  V3F inertia;
  inertia.x = Ixx;
  inertia.y = Iyy;
  inertia.z = Izz;
  momentCmd = inertia * kpPQR * err;
  return momentCmd;
}
```



### Implement roll pitch control in C++
It is implemented in function RollPitchControl, lines 139 to 183.

```cpp
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrustAcc = collThrustCmd / mass;
  float R13;
  float R23;
 
  if (thrustAcc< 0)
  {
	  R13 = 0;
	  R23 = 0;
  }
  else
  {
	  R13 = -CONSTRAIN(accelCmd[0] / thrustAcc, -maxTiltAngle, maxTiltAngle);
	  R23 = -CONSTRAIN(accelCmd[1] / thrustAcc, -maxTiltAngle, maxTiltAngle);
  }
  pqrCmd.x = (1 / R(2, 2))*(-R(1, 0) * kpBank*(R(0, 2) - R13) + R(0, 0) * kpBank*(R(1, 2) - R23));
  pqrCmd.y = (1 / R(2, 2))*(-R(1, 1) * kpBank*(R(0, 2) - R13) + R(0, 1) * kpBank*(R(1, 2) - R23));
  return pqrCmd;
}

```

### Implement altitude controller in C++
It is implemented in function AltitudeControl, lines 175 to 225


```cpp
float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;
  float posErr;
  posErr = posZCmd - posZ;
  velZCmd += kpPosZ * posErr;
  integratedAltitudeError += posErr * dt;
  velZCmd = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);
  float velErr;
  velErr = velZCmd - velZ;
  thrust = (9.81f - kpVelZ * velErr - KiPosZ * integratedAltitudeError - accelZCmd ) / R(2, 2) * mass;
  return thrust;
}
```

### Implement lateral position control in C++
It is implemented in function LateralPositionControl, lines 228 to 273.

```cpp
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;
  V3F accelCmd = accelCmdFF;
  V3F posErr = posCmd - pos;
  velCmd += kpPosXY * posErr;
  if (velCmd.mag() > maxSpeedXY)
  {
	  velCmd = velCmd * maxSpeedXY / velCmd.mag();
  }
  V3F velErr = velCmd - vel;
  accelCmd += kpVelXY * velErr;
  if (accelCmd.mag() > maxAccelXY)
  {
	  accelCmd = accelCmd * maxAccelXY / accelCmd.mag();
  }
  return accelCmd;
}
```

### Implement yaw control in C++
It is implemented in function YawControl, lines 276 to 308

```cpp
float QuadControl::YawControl(float yawCmd, float yaw)
{
  float yawRateCmd=0;
  float err= yawCmd - yaw;
  err = fmodf(err, F_PI*2.f);
  if (err > F_PI)
  {
	  err -= 2.f * F_PI;
  }
  else if (err < -F_PI)
  {
	  err += 2.f * F_PI;
  }
  yawRateCmd = err * kpYaw;
  return yawRateCmd;
}
``` 
### Implement calculating the motor commands given commanded thrust and moments in C++
It is implemented in function GenerateMotorCommands, lines 56 to 105.

```cpp
VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  cmd.desiredThrustsN[0] = collThrustCmd / 4.f; // front left
  cmd.desiredThrustsN[1] = collThrustCmd / 4.f; // front right
  cmd.desiredThrustsN[2] = collThrustCmd / 4.f; // rear left
  cmd.desiredThrustsN[3] = collThrustCmd / 4.f; // rear right

  cmd.desiredThrustsN[0] += xMomentThrustCmd; // front left
  cmd.desiredThrustsN[1] -= xMomentThrustCmd; // front right
  cmd.desiredThrustsN[2] += xMomentThrustCmd; // rear left
  cmd.desiredThrustsN[3] -= xMomentThrustCmd; // rear right

  float yMomentThrustCmd = momentCmd.y / (4.0f * sqrtf(2.0f) * L);  
  cmd.desiredThrustsN[0] += yMomentThrustCmd; // front left
  cmd.desiredThrustsN[1] += yMomentThrustCmd; // front right
  cmd.desiredThrustsN[2] -= yMomentThrustCmd; // rear left
  cmd.desiredThrustsN[3] -= yMomentThrustCmd; // rear right

  float zMomentThrustCmd = momentCmd.z / (4.0f * kappa) ; 
  cmd.desiredThrustsN[0] -= zMomentThrustCmd; // front left
  cmd.desiredThrustsN[1] += zMomentThrustCmd; // front right
  cmd.desiredThrustsN[2] += zMomentThrustCmd; // rear left
  cmd.desiredThrustsN[3] -= zMomentThrustCmd; // rear right

  return cmd;
}

```

---
## Flight Evaluation
### Your C++ controller is successfully able to fly the provided test trajectory and visually passes inspection of the scenarios leading up to the test trajectory.
Every scenario works stable and performs the required task.

```
Simulation #7 (../config/1_Intro.txt)
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
Simulation #8 (../config/2_AttitudeControl.txt)
Simulation #9 (../config/2_AttitudeControl.txt)
PASS: ABS(Quad.Roll) was less than 0.025000 for at least 0.750000 seconds
PASS: ABS(Quad.Omega.X) was less than 2.500000 for at least 0.750000 seconds
Simulation #10 (../config/3_PositionControl.txt)
Simulation #11 (../config/3_PositionControl.txt)
PASS: ABS(Quad1.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Yaw) was less than 0.100000 for at least 1.000000 seconds
Simulation #12 (../config/4_Nonidealities.txt)
Simulation #13 (../config/4_Nonidealities.txt)
PASS: ABS(Quad1.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad2.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad3.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
Simulation #14 (../config/5_TrajectoryFollow.txt)
Simulation #15 (../config/5_TrajectoryFollow.txt)
PASS: ABS(Quad2.PosFollowErr) was less than 0.250000 for at least 3.000000 seconds

```


