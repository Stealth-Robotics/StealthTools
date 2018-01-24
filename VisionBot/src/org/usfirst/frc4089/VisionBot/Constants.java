package org.usfirst.frc4089.VisionBot;

public class Constants {
  // Hardware Constants
  public static final int kGyroZ = 2;
  public static final int kFastButton = 1;
  public static final int kSlowButton = 2;

  // assorted variables
  public static final double collectorSpeed = .5;
  public static final double ellevatorSpeedUp = 1;
  public static final double ellevatorSpeedDown = -.75;
  public static final double shooterSpeed = 1;
  public static final double climber1Speed = 1;
  public static final double climber2Speed = 1;
  public static final double rightSpeed = 1;
  public static final double leftSpeed = .9;
  public static final double curve = 0;// -0.2;

  // wheel info
  public static final double kWheelDiam = 6.0; // in
  public static final double kWheelCirc = kWheelDiam * Math.PI; // in
  public static final double kEncoderPulsePerRev = 4096;

  // Speed
  public static final double kNormalSpeed = 0.7;
  public static final double kSlowSpeed = 0.3;

  // gyro
  public static final double kGyroGain = Math.PI/180;

  // Motor IDs
  public static final int kRightMotor1SpeedControl = 1; // normally 6
  public static final int kRightMotor2SpeedControl = 2;
  public static final int kLeftMotor1SpeedControl = 4;
  public static final int kLeftMotor2SpeedControl = 3;
  public static final int climbMotor1SpeedControl = 8; // normally 8
  public static final int climbMotor2SpeedControl = 4;
  public static final int ShooterMotorSpeedControl = 10;
  public static final int ellevatorMotorSpeedControl = 9; // normally 9
  public static final int agitatorMotorSpeedControl = 5;
  public static final int collectorMotorControl = 11;

  // servos
  public static final int shooterDoorPort = 1;
  public static final int RBarrelBlockerPort = 2;
  public static final int LBarrelBlockerPort = 3;

  // autoconfig
  public final static double kTolerance = 0.1;
  public final static double kP = .88;// -1.0 / 5.0;
  public final static double kPtr = .95; // kP for turn right
  public final static double kPrl = .95; // kP for turn left
  public final static double kI = -1.0 / 5.0;
  public final static double kD = -1.0 / 5.0;
  public final static double defaultAutoSpeed = .75;
  public final static double AutoGearDistS1 = 92; // IN
  public final static double AutoGearAngS2 = 30; // deg
  public final static double AutoGearDistS3 = 60; // IN
  public final static double AutoGearAngRadS2 = 2; // feet of radius
  public final static double AutoDriveFWTimed = 2.25; // sec
  public final static double autoFWSpdRight = 0.5;
  public final static double autoFWSpdLeft = 0.5872;// 0.671;

}