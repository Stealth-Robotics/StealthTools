package org.usfirst.frc4444.TerrifiedCameliansLittleSistem.paths;

import org.usfirst.frc4444.TerrifiedCameliansLittleSistem.pathfinding.*;

public class TurnCorner extends Path{

  public TurnCorner()
  {
    mPoints = new double[][]{
      {  0, 0},
      {120, 0},
      {120, 120}
      };
      mTime = 4;
  }
}
