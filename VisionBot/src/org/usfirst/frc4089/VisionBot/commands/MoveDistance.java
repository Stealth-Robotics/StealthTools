//----------------------------------------------------------------------------
//
//  $Workfile: MoveDistance.java$
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
package org.usfirst.frc4089.VisionBot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc4089.VisionBot.Constants;
import org.usfirst.frc4089.VisionBot.Robot;
import org.usfirst.frc4089.VisionBot.RobotMap;
import stealth_libraries.StopWatch;

//----------------------------------------------------------------------------
//Class Declarations
//----------------------------------------------------------------------------
//
//Class Name: MoveDistance
//
//Purpose:
//  Run a distance on the field
//
//----------------------------------------------------------------------------
public class MoveDistance extends Command {
  //----------------------------------------------------------------------------
  //  Class Attributes 
  //----------------------------------------------------------------------------
  double mDistance = 0.0;
  double mCurTarget = 0.0;
  StopWatch mForward = new StopWatch(5000);

  // --------------------------------------------------------------------
  // Purpose:
  // Constructor
  //
  // Notes:
  // None.
  // --------------------------------------------------------------------
  public MoveDistance() {
    mDistance = 12.0;
  }

  // --------------------------------------------------------------------
  // Purpose:
  // Constructor with distance
  //
  // Notes:
  // None.
  // --------------------------------------------------------------------
  public MoveDistance(int distance) {
    mDistance = distance;
  }

  // --------------------------------------------------------------------
  // Purpose:
  // Called just before this Command runs the first time
  //
  // Notes:
  // None.
  // --------------------------------------------------------------------
  @Override
  protected void initialize() {
    mForward.reset();
    mCurTarget = mDistance + RobotMap.leftEncoder.GetInches();
    Robot.driveBase.Drive(0,0);
  }

  // --------------------------------------------------------------------
  // Purpose:
  // Called repeatedly when this Command is scheduled to run
  //
  // Notes:
  // None.
  // --------------------------------------------------------------------
  @Override
  protected void execute() {
    if(mCurTarget > 0)
    {
      Robot.driveBase.Drive(Constants.kAutoSpeed, 0);
    }
    else
    {
      Robot.driveBase.Drive(-Constants.kAutoSpeed, 0);
    }
  }

  // --------------------------------------------------------------------
  // Purpose:
  // Make this return true when this Command no longer needs to run execute()
  //
  // Notes:
  // None.
  // --------------------------------------------------------------------
  @Override
  protected boolean isFinished() {
    double curLocation = RobotMap.leftEncoder.GetInches();
    if ((curLocation > mCurTarget) || (true == mForward.isExpired())) {
      Robot.driveBase.Drive(0, 0);
      return true;
    }

    return false;
  }

  // --------------------------------------------------------------------
  // Purpose:
  // Called once after isFinished returns true
  //
  // Notes:
  // None.
  // --------------------------------------------------------------------
  @Override
  protected void end() {
  }

  // --------------------------------------------------------------------
  // Purpose:
  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  //
  // Notes:
  // None.
  // --------------------------------------------------------------------
  @Override
  protected void interrupted() {
  }
}
