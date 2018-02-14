package org.usfirst.frc.team4089.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import org.usfirst.frc.team4089.robot.commands.*;
import org.usfirst.frc.team4089.robot.subsystems.*;

public class Robot extends TimedRobot {

	public static OI oi;
	public static RobotMap mMap;
	Command mTestCommand;
	public static DriveBase mDriveBase;

	@Override
	public void robotInit() {
		oi = new OI();
		mMap = new RobotMap();
		mMap.init();
		mDriveBase = new DriveBase();
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
	  mDriveBase.ClearCurrentAngle();
    mTestCommand = new ScoreInSwitch();
    Scheduler.getInstance().add(mTestCommand);
    mDriveBase.SetAuto();
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
    mDriveBase.SetTele();
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    Scheduler.getInstance().run();
  }
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testInit() {
  }
}
