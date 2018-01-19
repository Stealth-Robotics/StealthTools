// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc4089.VisionBot.subsystems;

import org.usfirst.frc4089.VisionBot.RobotMap;
import org.usfirst.frc4089.VisionBot.commands.*;
import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.Joystick;
import stealth_libraries.StealthMath;
import org.usfirst.frc4089.VisionBot.*;
import stealth_libraries.StopWatch;

/**
 *
 */
public class DriveBase extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	private static final double kAngleSetpoint = 0.0;
	private static final double kP = 0.008; // propotional turning constant
	private static final double kD = 0.0003; // propotional turning constant
	private static final double kI = 0.00001; // propotional turning constant
    
    private double mError = 0.0;
    private double mLastError = 0.0;
    private double mAcumError = 0.0;
    private static final double kMaxAcum = 100;
    private StopWatch debugDisplay = new StopWatch(200);
    
    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        setDefaultCommand(new UserDrive());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
    }

    @Override
    public void periodic() {
    }

    public void Drive(Joystick driveJoystick) {
    	double y = driveJoystick.getRawAxis(1);
    	double x = driveJoystick.getRawAxis(0);
    	
    	// Adjust for speed, check if the fast button is pushed
    	if(true == driveJoystick.getRawButton(Constants.kFastButton))
    	{
    		// Do Nothing
    	}
    	else
    	{
    		// Is the slow button pushed
        	if(true == driveJoystick.getRawButton(Constants.kSlowButton))
        	{
        		y *= Constants.kSlowSpeed;
        		x *= Constants.kSlowSpeed;
        	}
        	else
        	{
        		y *= Constants.kNormalSpeed;
        		x *= Constants.kNormalSpeed;
        	}
    	}
    	Drive(y,x);
    }	
    
    public void Drive(double y, double x) {

    	// IF we are turning, turn off the gyro
    	if(Math.abs(x)>0.2)
    	{
    		RawDrive(y,x);
    		mAcumError = 0;
    	}
    	else
    	{
        	double rate = RobotMap.reutrnGyro();
        	RobotMap.netTable.putNumber("gyroRate", rate);

        	mError = (kAngleSetpoint - rate);
        	mAcumError += mError;

        	mAcumError= StealthMath.Cap(mAcumError,kMaxAcum);        	
    		double turningValue = (mError * kP)+(mAcumError * kI) + (mLastError * kD);
    		turningValue=StealthMath.Cap(turningValue,1);
    		
    		mLastError = mError;

        	RobotMap.netTable.putNumber("error", mError);
        	RobotMap.netTable.putNumber("lastError", mLastError);
        	RobotMap.netTable.putNumber("acumError", mAcumError);    	
        	RobotMap.netTable.putNumber("speed", y);
        	RobotMap.netTable.putNumber("turning", turningValue);
    		    		
    		RawDrive(y,turningValue);
    	}
    }

    protected void RawDrive(double y, double x) {
    	double targetSpeedL = (y-x) * 4096 * 400.0 / -600; 
    	double targetSpeedR = (y+x) * 4096 * 400.0 / -600;
    	RobotMap.leftMotor1SpeedCpmtroler.set(ControlMode.Velocity, targetSpeedL);
    	RobotMap.rightMotor1SpeedCpmtroler.set(ControlMode.Velocity, targetSpeedR);
            	
    	RobotMap.netTable.putNumber("lMotor", RobotMap.leftMotor1SpeedCpmtroler.getMotorOutputVoltage());
    	RobotMap.netTable.putNumber("rMotor", RobotMap.rightMotor1SpeedCpmtroler.getMotorOutputVoltage());
    	RobotMap.netTable.putNumber("lEncoder", RobotMap.leftEncoder.GetInches());
    	RobotMap.netTable.putNumber("rEncoder", RobotMap.rightEncoder.GetInches());

    	Navigation.getInstance().setNewPose(RobotMap.leftEncoder.GetInches(),
    										RobotMap.rightEncoder.GetInches());
    	
    	if(true == debugDisplay.isExpired())
    	{
    		debugDisplay.reset();
    		System.out.format("%7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f\n", 
    				RobotMap.leftEncoder.GetInches(),
    				RobotMap.rightEncoder.GetInches(),
    				RobotMap.leftEncoder.GetVelocityInInches(),
					RobotMap.rightEncoder.GetVelocityInInches(),
					Navigation.getInstance().getX(),
					Navigation.getInstance().getY(),
					Navigation.getInstance().getThetaDeg());
    	}
    }

    public void ClearEncoders()
    {
		RobotMap.leftEncoder.Clear();
		RobotMap.rightEncoder.Clear();
    }
}

