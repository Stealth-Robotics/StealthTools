package org.usfirst.frc4444.TerrifiedCameliansLittleSistem;

public class Constants {
  // Hardware Constants
  public static final int kGyroZ = 2;
  public static final int kFastButton = 1;
  public static final int kSlowButton = 2;

  // assorted variables
  public static final double kAutoSpeed = 0.5;
  public static final double collectorSpeed = .5;
  public static final double ellevatorSpeedUp = 1;
  public static final double ellevatorSpeedDown = -.75;
  public static final double shooterSpeed = 1;
  public static final double climber1Speed = 1;
  public static final double climber2Speed = 1;
  public static final double rightSpeed = 1;
  public static final double leftSpeed = .9;
  public static final double curve = 0;// -0.2;

  public static final double kTicksPerRev = 280;
  public static final double kTicksPErRevSpeeed = 2.826;

  // Wheels
  public static double kDriveWheelDiameterInches = 3.419;
  public static double kTrackWidthInches = 26.655;
  public static double kTrackScrubFactor = 0.924;
  
  /**
   * set to zero to skip waiting for confirmation, set to nonzero to wait
   * and report to DS if action fails.
   */
  public static final int kTimeoutMs = 20;
  
  /**
   * Base trajectory period to add to each individual 
   * trajectory point's unique duration.  This can be set
   * to any value within [0,255]ms.
   */
  public static final int kBaseTrajPeriodMs = 0;
  /**
   * Motor deadband, set to 1%.
   */
  public static final double kNeutralDeadband  = 0.01;
  
  // wheel info
  public static final double kWheelDiam = 4.0; // in
  public static final double kWheelCirc = kWheelDiam * Math.PI; // in
  public static final double kEncoderPulsePerRev = 220;

  // Speed
  public static final double kNormalSpeed = 0.8;
  public static final double kSlowSpeed = 0.6;    // Path following constants
  public static double kMinLookAhead = 12.0; // inches
  public static double kMinLookAheadSpeed = 9.0; // inches per second
  public static double kMaxLookAhead = 24.0; // inches
  public static double kMaxLookAheadSpeed = 120.0; // inches per second
  public static double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
  public static double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;

  public static double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain *
                                                   // our speed
                                                   // in inches per sec
  public static double kSegmentCompletionTolerance = 0.1; // inches
  public static double kPathFollowingMaxAccel = 144.0; // inches per second^2
  public static double kPathFollowingMaxVel = 144.0; // inches per second
  public static double kPathFollowingProfileKp = 5.00;
  public static double kPathFollowingProfileKi = 0.03;
  public static double kPathFollowingProfileKv = 0.02;
  public static double kPathFollowingProfileKffv = 1.0;
  public static double kPathFollowingProfileKffa = 0.05;
  public static double kPathFollowingGoalPosTolerance = 0.75;
  public static double kPathFollowingGoalVelTolerance = 12.0;
  public static double kPathStopSteeringDistance = 9.0;
  
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