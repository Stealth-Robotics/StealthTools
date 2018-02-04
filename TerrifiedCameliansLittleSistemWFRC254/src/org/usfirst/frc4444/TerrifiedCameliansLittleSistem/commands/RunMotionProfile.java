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
package org.usfirst.frc4444.TerrifiedCameliansLittleSistem.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc4444.TerrifiedCameliansLittleSistem.control.*;

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
public class RunMotionProfile extends CommandGroup {
  //----------------------------------------------------------------------------
  //  Class Attributes 
  //----------------------------------------------------------------------------

  // --------------------------------------------------------------------
  // Purpose:
  // Constructor
  //
  // Notes:
  // None.
  // --------------------------------------------------------------------
  public RunMotionProfile() {
    PathContainer thePath = new CenterGearToShootBlue();
    
    addSequential(new DrivePathAction(thePath));
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
