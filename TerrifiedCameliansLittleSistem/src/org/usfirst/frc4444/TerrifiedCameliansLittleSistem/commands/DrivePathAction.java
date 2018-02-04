package org.usfirst.frc4444.TerrifiedCameliansLittleSistem.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc4444.TerrifiedCameliansLittleSistem.Robot;
import org.usfirst.frc4444.TerrifiedCameliansLittleSistem.pathfinding.*;
import org.usfirst.frc4444.TerrifiedCameliansLittleSistem.paths.*;
import org.usfirst.frc4444.TerrifiedCameliansLittleSistem.pathfinding.*;


public class DrivePathAction extends Command {
  RawPath mPath;
  double[][] mLeftPath;
  double[][] mRightPath;
  int mCount = 0;
  
    public DrivePathAction(Path path) {
      mCount = 0;
      mPath = new RawPath(220,4,14);
      mPath.SetPath(path.GetWaypoint(), path.GetTime());
    
      mLeftPath = mPath.GetLeftPath();
      mRightPath = mPath.GetRightPath();
    }

    @Override
    public boolean isFinished() {
        return (mCount>mRightPath.length);
    }

    @Override
    public void execute() {
      if(mCount<mRightPath.length)
      {
        Robot.driveBase.AutoDrive(mLeftPath[mCount][1], mRightPath[mCount][1], mLeftPath[mCount][3]);
        /*
        System.out.format("%6.2f %6.2f %6.2f %6.2f   %6.2f %6.2f %6.2f %6.2f\n", 
          mLeftPath[mCount][0],
          mLeftPath[mCount][1],
          mLeftPath[mCount][2],
          mLeftPath[mCount][3],
          mRightPath[mCount][0],
          mRightPath[mCount][1],
          mRightPath[mCount][2],
          mRightPath[mCount][3]);
          */
      }
      mCount++;
    }

    @Override
    public void end() {
        // TODO: Perhaps set wheel velocity to 0?
    }

    @Override
    public void initialize() {
      mCount = 0;
    }
}
