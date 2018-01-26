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
import org.usfirst.frc4089.VisionBot.motionprofiles.*;
import org.usfirst.frc4089.VisionBot.subsystems.*;

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
public class RunMotionProfile extends Command {
  //----------------------------------------------------------------------------
  //  Class Attributes 
  //----------------------------------------------------------------------------
  Path mPath = new Path();
  MotionDriveBase mBase = new MotionDriveBase();

  // --------------------------------------------------------------------
  // Purpose:
  // Constructor
  //
  // Notes:
  // None.
  // --------------------------------------------------------------------
  public RunMotionProfile() {
   System.out.println("RunMotionProfile"); 
  }

  // --------------------------------------------------------------------
  // Purpose:
  // Constructor with distance
  //
  // Notes:
  // None.
  // --------------------------------------------------------------------
  public RunMotionProfile(Path path) {
    mPath = path;
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
    System.out.println("initialize");
    mBase.initPaths(mPath.GetLeftPath(), mPath.GetRightPath());
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
    mBase.periodic();
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
    return mBase.isDone();
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
    mBase.stop();
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
    mBase.stop();
  }
}
