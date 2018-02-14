package org.usfirst.frc.team4089.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.command.TimedCommand;

import java.io.FileWriter;
import java.io.IOException;

import org.usfirst.frc.team4089.robot.*;
import org.usfirst.frc.team4089.robot.commands.*;

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
  public void DriveTest(Joystick driveJoystick) {
    double y = driveJoystick.getRawAxis(1);
    double x = driveJoystick.getRawAxis(2);
    
    x = stealth_libraries.StealthMath.DeadBand(x,0.25);
    y = stealth_libraries.StealthMath.DeadBand(y,0.25);
    
    
    //System.out.format("%6.2f %6.2f\n", y,x);
  }

  //--------------------------------------------------------------------
  // Purpose:
  //     Drive using the joystick 
  //
  // Notes:
  //     none
  //--------------------------------------------------------------------  
  public void Drive(Joystick driveJoystick) {
    double y = driveJoystick.getRawAxis(1)*-1;
    double x = driveJoystick.getRawAxis(0);

    x = stealth_libraries.StealthMath.DeadBand(x,0.25);
    y = stealth_libraries.StealthMath.DeadBand(y,0.25);
        
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
      RawDrive(speed*.7, turn*.7);
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
        
        RawDrive(speed,turnThrottle);
      }
      else
      {
        RawDrive(0, 0);
        mTargetAngle = mCurrentAngle;
      }
    }
  }

  public void ClearCurrentAngle()
  {
    mCurrentAngle = 0.0;
    RobotMap.pigeonIMU.setFusedHeading(0, 20);
  }
  
  
  public void AutoDrive(double speedL, 
          double speedR, 
          double heading,
          FileWriter logFile)
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
    
    double targetSpeedL = speedL*32;
    double targetSpeedR = speedR*32;
   
      double angle_difference = heading - mCurrentAngle;    // Make sure to bound this from -180 to 180, otherwise you will get super large values

      double turn = -40.0*angle_difference;

      targetSpeedL += turn;
      targetSpeedR -= turn;
    
      System.out.format("%6.2f %6.2f %6.2f\n",
          targetSpeedL,targetSpeedR,turn);
    
    RobotMap.leftMotor1SpeedControler.set(ControlMode.Velocity, targetSpeedL);
    RobotMap.rightMotor1SpeedControler.set(ControlMode.Velocity, targetSpeedR);

    try {
      logFile.write(
          Timer.getFPGATimestamp() + ", " +
          speedL + ", " +
          speedR + ", " +
          heading + ", " +
          targetSpeedL + ", " +
          targetSpeedR + ", " +
          RobotMap.leftMotor1SpeedControler.getSelectedSensorVelocity(0) + ", " +
          RobotMap.rightMotor1SpeedControler.getSelectedSensorVelocity(0) + ", " +
          RobotMap.leftMotor1SpeedControler.getMotorOutputVoltage() + ", " +
          RobotMap.rightMotor1SpeedControler.getMotorOutputVoltage() +", " +
          angle_difference + "\n"
      );
    }
    catch(IOException e) {
        e.printStackTrace();
    }
    
    
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
    
public void ResetEncoders()
{
  RobotMap.leftMotor1SpeedControler.setSelectedSensorPosition(0, 0, 20);
  RobotMap.rightMotor1SpeedControler.setSelectedSensorPosition(0, 0, 20);
}
  
}
