package org.usfirst.frc4444.TerrifiedCameliansLittleSistem.pathfinding;


public class RawPath {
  static final int kPositionLoc = 0;
  static final int kVelocityLoc = 1;
  static final int kTimeLoc = 2;
  static final int kHeading = 3;
  static final int kMaxCol = 4;
  static final double kTimeStep = 0.01; //period of control loop on Rio, seconds

  double[][] mRaw;
  FalconPathPlanner mPlanner;
  double mTotalTime;
  double[][] mLeftPath;
  double[][] mRightPath;
  int mNumberOfPoints;

  double mTicksPerRev = 100;
  double mWheelDiam = 6;
  double mInchesPerRev = 0;
  double mRobotTrackWidth = 28; //distance between left and right wheels, inches
  
  public RawPath()
  {
  }

  public RawPath(double ticks, double wheelDiam, double wheelDistance)
  {
    mTicksPerRev = ticks;
    mWheelDiam = wheelDiam;
    mInchesPerRev = mWheelDiam * Math.PI;
    mRobotTrackWidth = wheelDistance;
  }
  
  public void SetPath(double[][] path, double time)
  {
    System.out.println(path.length);
    mRaw = new double[path.length][2];
    
    for(int i=0;i<path.length;i++)
    {
      mRaw[i][0] = (path[i][0]);
      mRaw[i][1] = (path[i][1]);
    }
    
    mTotalTime = time;
    
    mPlanner = new FalconPathPlanner(mRaw);
    mPlanner.calculate(mTotalTime, kTimeStep, mRobotTrackWidth);
    
    mNumberOfPoints = mPlanner.smoothRightVelocity.length;
    mLeftPath = new double[mNumberOfPoints][kMaxCol];
    mRightPath = new double[mNumberOfPoints][kMaxCol];
    System.out.println(mNumberOfPoints);
    if(mNumberOfPoints<1)
    {
      return;
    }

    mLeftPath[0][kPositionLoc] = 0;
    mLeftPath[0][kVelocityLoc] = 0;
    mLeftPath[0][kTimeLoc] = 0;
    mLeftPath[0][kHeading] = 0;

    mRightPath[0][kPositionLoc] = 0;
    mRightPath[0][kVelocityLoc] = 0;
    mRightPath[0][kTimeLoc] = 0;
    mRightPath[0][kHeading] = 0;
    
    System.out.println("Start");
    for(int i=1;i<mNumberOfPoints;i++)
    {
      mLeftPath[i][kPositionLoc] = mLeftPath[i-1][kPositionLoc] +
          GetDistance(mPlanner.leftPath[i-1],
          mPlanner.leftPath[i]);
      mLeftPath[i][kVelocityLoc] = mPlanner.smoothLeftVelocity[i][1];
      mLeftPath[i][kTimeLoc] = mPlanner.smoothLeftVelocity[i][0];
      mLeftPath[i][kHeading] = mPlanner.heading[i][1];
      
      mRightPath[i][kPositionLoc] = mRightPath[i-1][kPositionLoc] +
          GetDistance(mPlanner.rightPath[i-1],
          mPlanner.rightPath[i]);
      mRightPath[i][kVelocityLoc] = mPlanner.smoothRightVelocity[i][1];
      mRightPath[i][kTimeLoc] = mPlanner.smoothRightVelocity[i][0];
      mRightPath[i][kHeading] = mPlanner.heading[i][1];
      /*
      System.out.format("%6.2f %6.2f %6.2f %6.2f    %6.2f %6.2f %6.2f %6.2f\n", 
          mLeftPath[i][kPositionLoc],
          mLeftPath[i][kVelocityLoc],
          mLeftPath[i][kTimeLoc],
          mLeftPath[i][kHeading],
          mRightPath[i][kPositionLoc],
          mRightPath[i][kVelocityLoc],
          mRightPath[i][kTimeLoc],
          mRightPath[i][kHeading]);
      */
    }
  }
  
  
  double GetDistance(double first[], double second[])
  {
    return Math.sqrt(Math.pow(second[1]-first[1],2)+Math.pow(second[0]-first[0],2));
  }
  
  public double[][] GetLeftPath()
  {
    return mLeftPath;
  }
  
  public double[][] GetRightPath()
  {
    return mRightPath;
  }
}
