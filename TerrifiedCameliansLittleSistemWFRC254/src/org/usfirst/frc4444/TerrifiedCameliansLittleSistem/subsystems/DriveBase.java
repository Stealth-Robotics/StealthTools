//----------------------------------------------------------------------------
//
//  $Workfile: DriveBase.java$
//
//  $Revision: X$
//
//  Project:    Stealth Libraries
//
//                            Copyright (c) 2018
//                               Cedarcrest High School
//                            All Rights Reserved
//
//  Modification History:
//  $Log:
//  $
//
//----------------------------------------------------------------------------
package org.usfirst.frc4444.TerrifiedCameliansLittleSistem.subsystems;

import org.usfirst.frc4444.TerrifiedCameliansLittleSistem.*;
import org.usfirst.frc4444.TerrifiedCameliansLittleSistem.commands.*;
import org.usfirst.frc4444.TerrifiedCameliansLittleSistem.control.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import stealth_libraries.*;

//----------------------------------------------------------------------------
//Class Declarations
//----------------------------------------------------------------------------
//
//Class Name: DriveBase
//
//Purpose:
//  Handle getting us from here to there
//
//----------------------------------------------------------------------------
public class DriveBase extends Subsystem {
  //----------------------------------------------------------------------------
  //  Class Constants 
  //----------------------------------------------------------------------------
  static final double kPgain = 0.03; /* percent throttle per degree of error */
  static final double kDgain = 0.0004; /* percent throttle per angular velocity dps */
  static final double kMaxCorrectionRatio = 0.20; /* cap corrective turning throttle to 30 percent of forward throttle */
  static final double kSpeedGain = 0.05; // The ramp for the speed
  
  // The robot drivetrain's various states.
  public enum DriveControlState {
      OPEN_LOOP, // open loop voltage control
      VELOCITY_SETPOINT, // velocity PID control
      PATH_FOLLOWING, // used for autonomous driving
      AIM_TO_GOAL, // turn to face the boiler
      TURN_TO_HEADING, // turn in place
      DRIVE_TOWARDS_GOAL_COARSE_ALIGN, // turn to face the boiler, then DRIVE_TOWARDS_GOAL_COARSE_ALIGN
      DRIVE_TOWARDS_GOAL_APPROACH // drive forwards until we are at optimal shooting distance
  }
    
  //----------------------------------------------------------------------------
  //  Class Attributes 
  //----------------------------------------------------------------------------
  double mTargetAngle = 0;
  double mCurrentAngle = 0.0;
  double mActualSpeed = 0.0;
  
  double mLeftEncoderPrevDistance = 0;
  double mRightEncoderPrevDistance = 0;

  StopWatch mDisplay = new StopWatch(500);

  private Path mCurrentPath = null;
  private DriveControlState mDriveControlState;
  private PathFollower mPathFollower;
  private RobotState mRobotState = RobotState.getInstance();

  
  
  
  //--------------------------------------------------------------------
  // Purpose:
  //     Init the command
  //
  // Notes:
  //     None.
  //--------------------------------------------------------------------   
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new UserDrive());
    mDriveControlState = DriveControlState.OPEN_LOOP;
  }

  //--------------------------------------------------------------------
  // Purpose:
  //     Calls from every tick
  //
  // Notes:
  //     None.
  //--------------------------------------------------------------------  
  @Override
  public void periodic() {
    // Do Nothing
  }

  //--------------------------------------------------------------------
  // Purpose:
  //     Sets the target angle 
  //
  // Notes:
  //     Usually used in auto when we want to reset things
  //--------------------------------------------------------------------  
  public void SetTargetAngle(double targetAngle)
  {
    mTargetAngle = targetAngle;
  }
  
  //--------------------------------------------------------------------
  // Purpose:
  //     Drive using the joystick 
  //
  // Notes:
  //     none
  //--------------------------------------------------------------------  
  public void Drive(Joystick driveJoystick) {
    double timestamp = Timer.getFPGATimestamp();    
    final double left_distance = getLeftDistanceInches();
    final double right_distance = getRightDistanceInches();
    final Rotation2d gyro_angle = getGyroAngle();
    final Twist2d odometry_velocity = mRobotState.generateOdometryFromSensors(left_distance - 
        mLeftEncoderPrevDistance,
        right_distance - mRightEncoderPrevDistance, gyro_angle);
    final Twist2d predicted_velocity = Kinematics.forwardKinematics(getLeftVelocityInchesPerSec(),
        getRightVelocityInchesPerSec());
    mRobotState.addObservations(timestamp, odometry_velocity, predicted_velocity);
    mLeftEncoderPrevDistance = left_distance;
    mRightEncoderPrevDistance = right_distance;

    if(DriveControlState.OPEN_LOOP == mDriveControlState)
    {
      double y = driveJoystick.getRawAxis(1)*-1;
      double x = driveJoystick.getRawAxis(0)*-1;
  
      // Adjust for speed, check if the fast button is pushed
      if (true == driveJoystick.getRawButton(Constants.kFastButton)) {
        // Do Nothing
      } else {
        // Is the slow button pushed
        if (true == driveJoystick.getRawButton(Constants.kSlowButton)) {
          y *= Constants.kSlowSpeed;
          x *= Constants.kSlowSpeed;
        } else {
          y *= Constants.kNormalSpeed;
          x *= Constants.kNormalSpeed;
        }
      }
      Drive(y, x);
    }
    
    if(DriveControlState.PATH_FOLLOWING == mDriveControlState)
    {
      if (mPathFollower != null) {
          updatePathFollower(timestamp);
//          mCSVWriter.add(mPathFollower.getDebug());
      }
    }

  }

  //--------------------------------------------------------------------
  // Purpose:
  //     Drive the robot using throttle and turn 
  //
  // Notes:
  //     Usually used in auto when we want to reset things
  //--------------------------------------------------------------------  
  public void Drive(double speed, double turn) {
    PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
    double [] xyz_dps = new double [3];
    RobotMap.pigeonIMU.getRawGyro(xyz_dps);
    RobotMap.pigeonIMU.getFusedHeading(fusionStatus);
    
    mCurrentAngle = fusionStatus.heading;
    double currentAngularRate = xyz_dps[2];
    double turnThrottle = turn;
    
    // IF we are turning, turn off the gyro
    if (Math.abs(turn) > 0.2) {
      RawDrive(speed*2, turn*2);
      mTargetAngle = mCurrentAngle;
    } else {
      if (Math.abs(speed) > 0.1) {
        double angleError = (mTargetAngle - mCurrentAngle);
        /* very simple Proportional and Derivative (PD) loop with a cap,
         * replace with favorite close loop strategy or leverage future Talon <=> Pigeon features. */
        turnThrottle = angleError * kPgain - (currentAngularRate) * kDgain;
        /* the max correction is the forward throttle times a scalar,
         * This can be done a number of ways but basically only apply small turning correction when we are moving slow
         * and larger correction the faster we move.  Otherwise you may need stiffer pgain at higher velocities. */
        double maxThrot = StealthMath.MaxCorrection(speed, kMaxCorrectionRatio);
        turnThrottle =  StealthMath.Cap(turnThrottle, maxThrot);
        
        RobotMap.netTable.putNumber("gyroRate", currentAngularRate);
        RobotMap.netTable.putNumber("angleError", angleError);
        RobotMap.netTable.putNumber("turnAdj", turnThrottle);

        RawDrive(speed,turnThrottle);
      }
      else
      {
        RawDrive(0, 0);
        mTargetAngle = mCurrentAngle;
      }
    }
  }

  protected void RawDriveTest(double speed, double turn) {
    RobotMap.leftMotor1SpeedControler.set(speed);
    RobotMap.rightMotor1SpeedControler.set(speed);

    RobotMap.netTable.putNumber("lMotor", RobotMap.leftMotor1SpeedControler.getMotorOutputVoltage());
    RobotMap.netTable.putNumber("rMotor", RobotMap.rightMotor1SpeedControler.getMotorOutputVoltage());
    RobotMap.netTable.putNumber("lEncoder", RobotMap.leftEncoder.GetInches());
    RobotMap.netTable.putNumber("rEncoder", RobotMap.rightEncoder.GetInches());

    if(true == mDisplay.isExpired())
    {
      mDisplay.reset();
      System.out.format("%6d %6d %6d %6d\n", 
        RobotMap.leftMotor1SpeedControler.getSelectedSensorPosition(0),
        RobotMap.rightMotor1SpeedControler.getSelectedSensorPosition(0),
        RobotMap.leftMotor1SpeedControler.getSelectedSensorVelocity(0),
        RobotMap.rightMotor1SpeedControler.getSelectedSensorVelocity(0));
    }
    
    if(Robot.oi.joystick1.getRawButton(5))
    {
      RobotMap.leftMotor1SpeedControler.setSelectedSensorPosition(0, 0, 20);
      RobotMap.rightMotor1SpeedControler.setSelectedSensorPosition(0, 0, 20);
    }
  }
  
  protected void RawDrive(double speed, double turn) {
    
    // Ramp the speed
    if(mActualSpeed != speed)
    {
      if(mActualSpeed<speed)
      {
        mActualSpeed = Math.min(mActualSpeed+kSpeedGain, speed);
      }
      else
      {
        mActualSpeed = Math.max(mActualSpeed-kSpeedGain, speed);
      }
    }
    
    double targetSpeedL = (mActualSpeed + turn) * 4530;
    double targetSpeedR = (mActualSpeed - turn) * 4560;
    RobotMap.leftMotor1SpeedControler.set(ControlMode.Velocity, targetSpeedL);
    RobotMap.rightMotor1SpeedControler.set(ControlMode.Velocity, targetSpeedR);

    RobotMap.netTable.putNumber("lMotor", RobotMap.leftMotor1SpeedControler.getMotorOutputVoltage());
    RobotMap.netTable.putNumber("rMotor", RobotMap.rightMotor1SpeedControler.getMotorOutputVoltage());
    RobotMap.netTable.putNumber("lEncoder", RobotMap.leftEncoder.GetInches());
    RobotMap.netTable.putNumber("rEncoder", RobotMap.rightEncoder.GetInches());

    if(true == mDisplay.isExpired())
    {
      mDisplay.reset();
      System.out.format("%6f %6f %6f %6f %6f %6f %6d %6d\n",
          speed,turn, targetSpeedL,targetSpeedR,
        getLeftDistanceInches(),
        getRightDistanceInches(),
        RobotMap.leftMotor1SpeedControler.getSelectedSensorVelocity(0),
        RobotMap.rightMotor1SpeedControler.getSelectedSensorVelocity(0));
    }
    
    if(Robot.oi.joystick1.getRawButton(5))
    {
      RobotMap.leftMotor1SpeedControler.setSelectedSensorPosition(0, 0, 20);
      RobotMap.rightMotor1SpeedControler.setSelectedSensorPosition(0, 0, 20);
    }
    
    /*
    Navigation.getInstance().setNewPose(
        RobotMap.leftEncoder.GetInches(), 
        RobotMap.rightEncoder.GetInches(),
        mCurrentAngle);
     */   
  }

  public void ClearEncoders() {
    RobotMap.leftEncoder.Clear();
    RobotMap.rightEncoder.Clear();
  }
  
  
  /**
   * Configures the drivebase to drive a path. Used for autonomous driving
   * 
   * @see Path
   */
  public synchronized void setWantDrivePath(Path path, boolean reversed) {
      if (mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
          configureTalonsForSpeedControl();
          RobotState.getInstance().resetDistanceDriven();
          mPathFollower = new PathFollower(path, reversed,
                  new PathFollower.Parameters(
                          new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead,
                                  Constants.kMinLookAheadSpeed, Constants.kMaxLookAheadSpeed),
                          Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
                          Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
                          Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
                          Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel,
                          Constants.kPathFollowingGoalPosTolerance, Constants.kPathFollowingGoalVelTolerance,
                          Constants.kPathStopSteeringDistance));
          mDriveControlState = DriveControlState.PATH_FOLLOWING;
          mCurrentPath = path;
      } else {
          setVelocitySetpoint(0, 0);
      }
  }

  public synchronized boolean isDoneWithPath() {
      if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
          return mPathFollower.isFinished();
      } else {
          System.out.println("Robot is not in path following mode");
          return true;
      }
  }

  public synchronized void forceDoneWithPath() {
      if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
          mPathFollower.forceFinish();
      } else {
          System.out.println("Robot is not in path following mode");
      }
  }
  
  /**
   * Called periodically when the robot is in path following mode. Updates the path follower with the robots latest
   * pose, distance driven, and velocity, the updates the wheel velocity setpoints.
   */
  private void updatePathFollower(double timestamp) {
      RigidTransform2d robot_pose = mRobotState.getLatestFieldToVehicle().getValue();
      
      if(true == mDisplay.isExpired())
      {
        mDisplay.reset();
        System.out.format("%6.2f %6.2f\n",
            mRobotState.getLatestFieldToVehicle().getValue().getTranslation().x(),
            mRobotState.getLatestFieldToVehicle().getValue().getTranslation().y());
      }
      
      
      Twist2d command = mPathFollower.update(timestamp, robot_pose,
              RobotState.getInstance().getDistanceDriven(), RobotState.getInstance().getPredictedVelocity().dx);
      if (!mPathFollower.isFinished()) {
          Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
          updateVelocitySetpoint(setpoint.left, setpoint.right);
      } else {
          updateVelocitySetpoint(0, 0);
      }
  }

  /**
   * Adjust Velocity setpoint (if already in velocity mode)
   * 
   * @param left_inches_per_sec
   * @param right_inches_per_sec
   */
  private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
//      final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
      RobotMap.leftMotor1SpeedControler.set(inchesPerSecondToRpm(left_inches_per_sec));
      RobotMap.rightMotor1SpeedControler.set(inchesPerSecondToRpm(right_inches_per_sec));
/*
      if(true == mDisplay.isExpired())
      {
        mDisplay.reset();
        System.out.format("%6.2f %6.2f  %6d %6d %6d %6d\n",
            left_inches_per_sec,
            right_inches_per_sec,
          RobotMap.leftMotor1SpeedControler.getSelectedSensorPosition(0),
          RobotMap.rightMotor1SpeedControler.getSelectedSensorPosition(0),
          RobotMap.leftMotor1SpeedControler.getSelectedSensorVelocity(0),
          RobotMap.rightMotor1SpeedControler.getSelectedSensorVelocity(0));
      }
 */     
  }
  
  
  private static double inchesPerSecondToRpm(double inches_per_second) {
    return inchesToRotations(inches_per_second) * 60;
}
  
  private static double inchesToRotations(double inches) {
    return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
}


  
  /**
   * Configures talons for velocity control
   */
  private void configureTalonsForSpeedControl() {
    /*
      RobotMap.leftMotor1SpeedControler.changeControlMode(CANTalon.TalonControlMode.Speed);
      RobotMap.leftMotor1SpeedControler.setNominalClosedLoopVoltage(12.0);
      RobotMap.leftMotor1SpeedControler.setProfile(kHighGearVelocityControlSlot);
      RobotMap.leftMotor1SpeedControler.configNominalOutputVoltage(Constants.kDriveHighGearNominalOutput,
              -Constants.kDriveHighGearNominalOutput);
      mRightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
      mRightMaster.setNominalClosedLoopVoltage(12.0);
      mRightMaster.setProfile(kHighGearVelocityControlSlot);
      mRightMaster.configNominalOutputVoltage(Constants.kDriveHighGearNominalOutput,
              -Constants.kDriveHighGearNominalOutput);
    */
  }
  
  
  /**
   * Start up velocity mode. This sets the drive train in high gear as well.
   * 
   * @param left_inches_per_sec
   * @param right_inches_per_sec
   */
  public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
      configureTalonsForSpeedControl();
      mDriveControlState = DriveControlState.VELOCITY_SETPOINT;
      updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
  }
  
  
  private static double rotationsToInches(double rotations) {
    return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
}

private static double rpmToInchesPerSecond(double rpm) {
    return rotationsToInches(rpm) / 60;
}

  
  public double getLeftDistanceInches() {
    return -1*RobotMap.leftEncoder.GetInches();
}

public double getRightDistanceInches() {
  return -1*RobotMap.rightEncoder.GetInches();
}

public double getLeftVelocityInchesPerSec() {
    return rpmToInchesPerSecond(RobotMap.leftMotor1SpeedControler.getSelectedSensorVelocity(0));
}

public double getRightVelocityInchesPerSec() {
    return rpmToInchesPerSecond(RobotMap.rightMotor1SpeedControler.getSelectedSensorVelocity(0));
}
  
public synchronized Rotation2d getGyroAngle() {
  return Rotation2d.fromDegrees(RobotMap.returnRawGyro());
}

public void ResetEncoders()
{
  RobotMap.leftMotor1SpeedControler.setSelectedSensorPosition(0, 0, 20);
  RobotMap.rightMotor1SpeedControler.setSelectedSensorPosition(0, 0, 20);
}
  
}
