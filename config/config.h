#ifndef CONFIG_H
#define CONFIG_H

// Robot Settings Geometery
#define robotSetting_RobotCenterFromKicker 0.073
#define robotSetting_RobotRadius 0.09
#define robotSetting_RobotHeight 0.13
#define robotSetting_BottomHeight 0.02
#define robotSetting_KickerZ 0.005
#define robotSetting_KickerThickness 0.005
#define robotSetting_KickerWidth 0.08
#define robotSetting_KickerHeight 0.04
#define robotSetting_WheelRadius 0.0325
#define robotSetting_WheelThickness 0.005
#define robotSetting_Wheel1Angle 60
#define robotSetting_Wheel2Angle 135
#define robotSetting_Wheel3Angle 225
#define robotSetting_Wheel4Angle 300

// Robot Settings Physics
#define robotSetting_BodyMass 2
#define robotSetting_WheelMass 0.2
#define robotSetting_KickerMass 0.02
#define robotSetting_KickerDampFactor 0.2
#define robotSetting_RollerTorqueFactor 0.06
#define robotSetting_RollerPerpendicularTorqueFactor 0.005
#define robotSetting_Kicker_Friction 0.8
#define robotSetting_WheelTangentFriction 0.8
#define robotSetting_WheelPerpendicularFriction 0.05
#define robotSetting_Wheel_Motor_FMax 0.2

// Ball Settings G
#define ballSetting_BallRadius 0.0215

// Ball Settings P
#define ballSetting_BallMass 0.043
#define ballSetting_BallFriction 0.05
#define ballSetting_BallSlip 1
#define ballSetting_BallBounce 0.5
#define ballSetting_BallBounceVel 0.1
#define ballSetting_BallLinearDamp 0.004
#define ballSetting_BallAngularDamp 0.004

// Field Settings
#define fieldSetting_Field_Line_Width 0.010
#define fieldSetting_Field_Length 12.000
#define fieldSetting_Field_Width 9.000
#define fieldSetting_Field_Rad 0.500
#define fieldSetting_Field_Free_Kick 0.700
#define fieldSetting_Field_Penalty_Width 2.40
#define fieldSetting_Field_Penalty_Depth 1.20
#define fieldSetting_Field_Penalty_Point 1.20
#define fieldSetting_Field_Margin 0.3
#define fieldSetting_Field_Referee_Margin 0.4
#define fieldSetting_Wall_Thickness 0.050
#define fieldSetting_Goal_Thickness 0.020
#define fieldSetting_Goal_Depth 0.200
#define fieldSetting_Goal_Width 1.200
#define fieldSetting_Goal_Height 0.160

// Simulator Settings
#define worldSetting_Gravity 9.8
#define worldSetting_DeltaTime 0.016666667
#define worldSetting_ResetTurnOver true
#define worldSetting_observation_port 1234
#define worldSetting_commands_port 20011

#define MAX_ROBOT_COUNT 16
#define TEAM_COUNT 2 // 1->JustBlue [-|-] 2->Blue&Yellow 
#define Robots_Count 8

#endif // CONFIG_H