package org.usfirst.frc.team4089.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import org.usfirst.frc.team4089.robot.commands.*;

public class Robot extends TimedRobot {

	public static OI m_oi;
	public static RobotMap mMap;
	Command mTestCommand;

	@Override
	public void robotInit() {
		m_oi = new OI();
		mMap = new RobotMap();
		mMap.init();
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
    mTestCommand = new DrivetrainCharacterizer(4,
        DrivetrainCharacterizer.TestMode.QUASI_STATIC,
        0.005,
        0.7);
    Scheduler.getInstance().add(mTestCommand);
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
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
