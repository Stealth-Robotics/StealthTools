package org.usfirst.frc4444.TerrifiedCameliansLittleSistem.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc4444.TerrifiedCameliansLittleSistem.Robot;
import org.usfirst.frc4444.TerrifiedCameliansLittleSistem.paths.*;


public class DrivePathAction extends Command {
    private Path mPath;
    private int mCount = 0;
  
    public DrivePathAction(Path path) {
      mPath = path;
      mCount = 0;
    }

    @Override
    public boolean isFinished() {
      int curCount = mPath.kNumPoints - mCount;
        if(curCount<1)
        {
          System.out.println(mPath.kNumPoints);
          return true;
        }
        
        return false;
    }

    @Override
    public void execute() {
      if(mCount<mPath.kNumPoints)
      {
        Robot.driveBase.AutoDrive(
            mPath.kPoints[mCount][0],
            mPath.kPoints[mCount][1],
            mPath.kPoints[mCount][2]);
      }
      mCount++;
    }

    @Override
    public void end() {
        Robot.driveBase.Drive(0, 0);
    }

    @Override
    public void initialize() {
      mCount = 0;
    }
}
