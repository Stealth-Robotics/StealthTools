package org.usfirst.frc.team4089.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class RobotMap {
  public static final int kRightMotor1SpeedControl = 2; // normally 6
  public static final int kRightMotor2SpeedControl = 4;
  public static final int kLeftMotor1SpeedControl = 1;
  public static final int kLeftMotor2SpeedControl = 3;
  public static final int kTimeoutMs = 10;
  public static final int kPIDLoopIdx = 0;
  
  public static WPI_TalonSRX leftMotor1SpeedControler;
  public static WPI_TalonSRX leftMotor2SpeedControler;
  public static WPI_TalonSRX rightMotor1SpeedControler;
  public static WPI_TalonSRX rightMotor2SpeedControler;

  public static void init() {
    leftMotor1SpeedControler = new WPI_TalonSRX(kLeftMotor1SpeedControl);
    leftMotor2SpeedControler = new WPI_TalonSRX(kLeftMotor2SpeedControl);
    rightMotor1SpeedControler = new WPI_TalonSRX(kRightMotor1SpeedControl);
    rightMotor2SpeedControler = new WPI_TalonSRX(kRightMotor2SpeedControl);
    
    SetUpTalonForTele(leftMotor1SpeedControler);
    SetUpTalonForTele(rightMotor1SpeedControler);
    leftMotor2SpeedControler.set(ControlMode.Follower, kLeftMotor1SpeedControl);
    rightMotor2SpeedControler.set(ControlMode.Follower, kRightMotor1SpeedControl);
    rightMotor1SpeedControler.setInverted(true);
    rightMotor2SpeedControler.setInverted(true);    
  }
  
  static void SetUpTalonForTele(WPI_TalonSRX talon) {
    talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs);
    talon.setSensorPhase(true);

    /* set the peak and nominal outputs, 12V means full */
    talon.configNominalOutputForward(0, kTimeoutMs);
    talon.configNominalOutputReverse(0, kTimeoutMs);
    talon.configPeakOutputForward(1, kTimeoutMs);
    talon.configPeakOutputReverse(-1, kTimeoutMs);

    /* set closed loop gains in slot0 */
    talon.config_kF(kPIDLoopIdx, 0.34, kTimeoutMs);
    talon.config_kP(kPIDLoopIdx, 0.2, kTimeoutMs);
    talon.config_kI(kPIDLoopIdx, 0, kTimeoutMs);
    talon.config_kD(kPIDLoopIdx, 0, kTimeoutMs);

    talon.setSafetyEnabled(true);
    talon.setExpiration(kTimeoutMs);
  }
  
  
}

