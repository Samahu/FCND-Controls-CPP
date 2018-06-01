#  Implementation details

## Body Rate Control ##

The controller should be a proportional controller on body rates to commanded moments. This has been implememented by take the diff between commanded body rates against current body rates then multipy the difference by the kpPQR gains.
Moments of inertia are taken into account when calculating the commanded moments

```
V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
    V3F momentCmd;

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    auto pqrError = pqrCmd - pqr;
    auto u_bar = kpPQR * pqrError;
    momentCmd.x = Ixx * u_bar.x;
    momentCmd.y = Iyy * u_bar.y;
    momentCmd.z = Izz * u_bar.z;

    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return momentCmd;
}
```

## Roll Pitch Control ##
I first convert thrust value into accerlation then use the acceleration to compute the target tilt angles for roll and pitch. I make sure I don't exceed maxTiltAngle.
Then using current tilt angles supplied through ```Mat3x3F R = attitude.RotationMatrix_IwrtB();``` I compute the target rates b_x_c and b_y_c. Then 

```
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
    V3F pqrCmd;
    Mat3x3F R = attitude.RotationMatrix_IwrtB();

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    auto b_x_c_target = 0.0f;
    auto b_y_c_target = 0.0f;

    if (collThrustCmd > 0)
    {
        auto c = collThrustCmd / mass;  // convert to acceleration
        b_x_c_target = -CONSTRAIN(accelCmd.x / c, -maxTiltAngle, maxTiltAngle);
        b_y_c_target = -CONSTRAIN(accelCmd.y / c, -maxTiltAngle, maxTiltAngle);
    }

    auto b_x_c = kpBank * (b_x_c_target - R(0, 2));
    auto b_y_c = kpBank * (b_y_c_target - R(1, 2));

    pqrCmd.x = ( R(1, 0) * b_x_c - R(0, 0) * b_y_c ) / R(2, 2);
    pqrCmd.y = ( R(1, 1) * b_x_c - R(0, 1) * b_y_c ) / R(2, 2);
    pqrCmd.z = 0;

    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return pqrCmd;
}
```

## Altitude Controller ##

Both the down position and the down velocity are used to compute the drone commanded accerelation. 
The produced accerlation is converted into a thrust using the drone's mass.
The altitude controller also implements an integrator  ```integratedAltitudeError += dt * positionError;```

```
float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
    Mat3x3F R = attitude.RotationMatrix_IwrtB();
    float thrust = 0;

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    auto positionError = posZCmd - posZ;
    velZCmd = CONSTRAIN(velZCmd + kpPosZ * positionError, -maxAscentRate, maxDescentRate);
    integratedAltitudeError += dt * positionError;
    auto u_bar = kpVelZ * (velZCmd - velZ) + KiPosZ * integratedAltitudeError + accelZCmd;
    thrust = - mass * (u_bar - CONST_GRAVITY) / R(2, 2);

    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return thrust;
}
```


## Lateral Position Control ##

Both the local position and velocity are used to generate a commanded local acceleration.
They are also thresholded by the maxSpeedXY and maxAccelXY constants.

```
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
    // make sure we don't have any incoming z-component
    accelCmdFF.z = 0;
    velCmd.z = 0;
    posCmd.z = pos.z;

    // we initialize the returned desired acceleration to the feed-forward value.
    // Make sure to _add_, not simply replace, the result of your controller
    // to this variable
    V3F accelCmd = accelCmdFF;

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    velCmd += kpPosXY * (posCmd - pos);
    auto magVelCmd = velCmd.mag();
    if (magVelCmd > maxSpeedXY)
        velCmd *= maxSpeedXY / magVelCmd;

    accelCmd += kpVelXY * (velCmd - vel);
    auto magAccelCmd = accelCmd.mag();
    if (magAccelCmd > maxAccelXY)
        magAccelCmd *= maxAccelXY / magAccelCmd;

    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return accelCmd;
}
```


## Yaw Control ##

A proportional heading controller is implemented as follows:

```
// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
    float yawRateCmd=0;
    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    auto yawError = (yawCmd - yaw);
    yawError = fmodf(yawError, 2.0f * F_PI);

    if (yawError > F_PI)
        yawError = yawError - 2.0f * F_PI;
    else if (yawError < - F_PI)
        yawError = yawError + 2.0f * F_PI;

    yawRateCmd = kpYaw * yawError;

    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return yawRateCmd;
}
```


## Calculating Motor Commands ##

The thrust and moments has been converted to 4 appropriate desired thrust forces for the moments. 
The dimensions of the drone are properly accounted for when calculating thrust from moments.

```
VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    auto c_bar = collThrustCmd;
    auto p_bar = 2.0f * momentCmd.x / L / sqrt(2);
    auto q_bar = 2.0f * momentCmd.y / L / sqrt(2);
    auto r_bar = momentCmd.z / kappa;

    auto omega_1 = 0.25f * (c_bar + p_bar + q_bar - r_bar);
    auto omega_2 = 0.25f * (c_bar - p_bar + q_bar + r_bar);
    auto omega_3 = 0.25f * (c_bar + p_bar - q_bar + r_bar);
    auto omega_4 = 0.25f * (c_bar - p_bar - r_bar - q_bar);

    cmd.desiredThrustsN[0] = omega_1; // front left
    cmd.desiredThrustsN[1] = omega_2; // front right
    cmd.desiredThrustsN[2] = omega_3; // rear left
    cmd.desiredThrustsN[3] = omega_4; // rear right

    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return cmd;
}
```

# Tuning #

The following parameters were modified such that all 4 different scenarios would pass.

## Position control gains ##
kpPosXY = 2.5       // original 1  
kpPosZ = 2.5         // original 1  
KiPosZ = 50           // original 20  

## Velocity control gains ##
kpVelXY = 11         // original 4  (increasing this from 8 to 11 helped get scenario 5 working)
kpVelZ = 8           // original 4  

## Angle control gains ##
kpBank = 12         // original 5  
kpYaw = 3             // original 1  

## Angle rate gains ##
kpPQR = 75, 75, 15  // original 23, 23, 5  

