package org.usfirst.frc4444.TerrifiedCameliansLittleSistem.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc4444.TerrifiedCameliansLittleSistem.Robot;
import org.usfirst.frc4444.TerrifiedCameliansLittleSistem.control.*;

/**
 * Drives the robot along the Path defined in the PathContainer object. The action finishes once the robot reaches the
 * end of the path.
 * 
 * @see PathContainer
 * @see Path
 * @see Action
 */
public class DrivePathAction extends Command {

    private PathContainer mPathContainer;
    private Path mPath;

    public DrivePathAction(PathContainer p) {
        mPathContainer = p;
        mPath = mPathContainer.buildPath();
        
        System.out.println("start");
        System.out.println(mPath.toString());        
    }

    @Override
    public boolean isFinished() {
        return Robot.driveBase.isDoneWithPath();
    }

    @Override
    public void execute() {
        // Nothing done here, controller updates in mEnabedLooper in robot
    }

    @Override
    public void end() {
        // TODO: Perhaps set wheel velocity to 0?
    }

    @Override
    public void initialize() {
        Robot.driveBase.setWantDrivePath(mPath, mPathContainer.isReversed());
    }
}
