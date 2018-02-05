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

  StopWatch mDisplay = new StopWatch(1000);
  DriveControlState mState = DriveControlState.OPEN_LOOP;
  
  public void SetAuto()
  {
    mState = DriveControlState.PATH_FOLLOWING;
  }
  
  public void SetTele()
  {
    mState = DriveControlState.OPEN_LOOP;
  }
  
  
  
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
    mLeftEncoderPrevDistance = left_distance;
    mRightEncoderPrevDistance = right_distance;

    double y = driveJoystick.getRawAxis(1);
    double x = driveJoystick.getRawAxis(0);

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
    
    if(DriveControlState.OPEN_LOOP == mState)
    {
      Drive(y, x);
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
        turnThrottle =  -1*StealthMath.Cap(turnThrottle, maxThrot);
        
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

  public void AutoDrive(double speedL, double speedR, double heading)
  {
    PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
    double [] xyz_dps = new double [3];
    RobotMap.pigeonIMU.getRawGyro(xyz_dps);
    RobotMap.pigeonIMU.getFusedHeading(fusionStatus);
    
    mCurrentAngle = fusionStatus.heading;

    if(heading>180)
    {
      heading -= 360.0;
    }
    
    
    double targetSpeedL = speedL*20;
    double targetSpeedR = speedR*20;
   
      double angle_difference = heading - mCurrentAngle;    // Make sure to bound this from -180 to 180, otherwise you will get super large values

      double turn = -200.0*angle_difference;

      targetSpeedL += turn;
      targetSpeedR -= turn;
    
      System.out.format("%6.2f %6.2f %6.2f\n",
          targetSpeedL,targetSpeedR,turn);
    
    RobotMap.leftMotor1SpeedControler.set(ControlMode.Velocity, targetSpeedL);
    RobotMap.rightMotor1SpeedControler.set(ControlMode.Velocity, targetSpeedR);
    
    RobotMap.netTable.putNumber("gyroError", turn);
    RobotMap.netTable.putNumber("lMotor", RobotMap.leftMotor1SpeedControler.getMotorOutputVoltage());
    RobotMap.netTable.putNumber("rMotor", RobotMap.rightMotor1SpeedControler.getMotorOutputVoltage());
    RobotMap.netTable.putNumber("lEncoder", 
        RobotMap.leftMotor1SpeedControler.getSelectedSensorPosition(0));
    RobotMap.netTable.putNumber("rEncoder", 
        RobotMap.rightMotor1SpeedControler.getSelectedSensorPosition(0));

  }
  
  
  int mCount = 0;
  double mSinCount = 0.0;
  
  protected void RawDrive(double speed, double turn) {
    mCount++;
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
    
    //mActualSpeed = Math.sin(mSinCount);
    //mSinCount+=.01;
        
    double targetSpeedL = (mActualSpeed + turn) * 1820;
    double targetSpeedR = (mActualSpeed - turn) * 2000;
    RobotMap.leftMotor1SpeedControler.set(ControlMode.Velocity, targetSpeedL);
    RobotMap.rightMotor1SpeedControler.set(ControlMode.Velocity, targetSpeedR);

    RobotMap.netTable.putNumber("lMotor", RobotMap.leftMotor1SpeedControler.getMotorOutputVoltage());
    RobotMap.netTable.putNumber("rMotor", RobotMap.rightMotor1SpeedControler.getMotorOutputVoltage());
    RobotMap.netTable.putNumber("lEncoder", RobotMap.leftEncoder.GetInches());
    RobotMap.netTable.putNumber("rEncoder", RobotMap.rightEncoder.GetInches());

    if(true == mDisplay.isExpired())
    {
      mDisplay.reset();
      double timestamp = Timer.getFPGATimestamp();    
      System.out.format("%f %d %6.2f %6.2f %6d %6d %6d %6d\n",
          timestamp,mCount,targetSpeedL,targetSpeedR,
        RobotMap.leftMotor1SpeedControler.getSelectedSensorPosition(0),
        RobotMap.leftMotor1SpeedControler.getSelectedSensorVelocity(0),
        RobotMap.rightMotor1SpeedControler.getSelectedSensorPosition(0),
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
