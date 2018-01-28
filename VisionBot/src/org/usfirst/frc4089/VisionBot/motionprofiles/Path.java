package org.usfirst.frc4089.VisionBot.motionprofiles;

public class Path {
  RawPath mPath;
  
  public Path()
  { 
    double[][] waypointPath = new double[][]{
      {  0, 0},
      {120, 0},
      {120, 120}
      };
      
      mPath = new RawPath(4096,6,22);
      mPath.SetPath(waypointPath, 2);
  }

  public double[][] GetLeftPath()
  {
    return mPath.GetLeftPath();
  }
  
  public double[][] GetRightPath()
  {
    return mPath.GetRightPath();
  }
}
