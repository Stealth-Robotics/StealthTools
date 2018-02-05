// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package org.usfirst.frc4444.TerrifiedCameliansLittleSistem.commands;

import org.usfirst.frc4444.TerrifiedCameliansLittleSistem.paths.*;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ScoreInSwitch extends CommandGroup {
  public ScoreInSwitch() {
//    addSequential(new DrivePathAction(new Path()));
//    addSequential(new DrivePathAction(new Move10Path()));
    addSequential(new DrivePathAction(new Red21Path()));
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
