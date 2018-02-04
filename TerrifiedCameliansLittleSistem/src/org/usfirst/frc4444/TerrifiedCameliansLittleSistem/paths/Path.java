package org.usfirst.frc4444.TerrifiedCameliansLittleSistem.paths;

import org.usfirst.frc4444.TerrifiedCameliansLittleSistem.pathfinding.*;

public class Path {

  protected double[][] mPoints;
  protected double mTime;
  
  public Path()
  {
    mPoints = new double[][]{
      {  0, 0},
      {48, 0}
      };
      mTime = 2;
  }
  
  public double[][] GetWaypoint()
  {
    return mPoints;
  }
  
  public double GetTime()
  {
    return mTime;
  }
}
