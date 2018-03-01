//----------------------------------------------------------------------------
//
//  $Workfile: BadPath60InPerSec.java
//
//  $Revision: X$
//
//  Project:    Paths
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
//
//
//----------------------------------------------------------------------------
//    Parameters Used
//----------------------------------------------------------------------------
//   Time Slice= 0.020000
//   Max Vel   = 60.000000
//   Max Accel = 100.000000
//   Max Jerk  =  1000.000000
package org.usfirst.frc4089.Stealth2018.MPPaths;

public class BadPath60InPerSec extends Path {
    public BadPath60InPerSec() {
       kNumPoints = 136;
       kPoints = new double[][]{
  {0.400000, 0.400000, 0.000000, 0.002800, 7.000000, 0.002800, -7.000000},
  {0.800000, 0.800000, 0.000000, 0.018800, 7.000000, 0.018800, -7.000000},
  {1.800000, 1.800000, 0.000000, 0.054800, 7.000000, 0.054800, -7.000000},
  {3.200000, 3.200000, 0.000000, 0.118800, 7.000000, 0.118800, -7.000000},
  {5.000000, 5.000000, 0.000000, 0.218800, 7.000000, 0.218800, -7.000000},
  {7.000000, 7.000000, 0.000000, 0.358800, 7.000000, 0.358800, -7.000000},
  {9.000000, 9.000000, 0.000000, 0.538800, 7.000000, 0.538800, -7.000000},
  {11.000000, 11.000000, 0.000000, 0.758800, 7.000000, 0.758800, -7.000000},
  {13.000000, 13.000000, 0.000000, 1.018800, 7.000000, 1.018800, -7.000000},
  {15.000000, 15.000000, 0.000000, 1.318800, 7.000000, 1.318800, -7.000000},
  {17.000000, 17.000000, 0.000000, 1.658800, 7.000000, 1.658800, -7.000000},
  {19.000000, 19.000000, 0.000000, 2.038800, 7.000000, 2.038800, -7.000000},
  {21.000000, 21.000000, 0.000000, 2.458800, 7.000000, 2.458800, -7.000000},
  {23.000000, 23.000000, 0.000000, 2.918800, 7.000000, 2.918800, -7.000000},
  {25.000000, 25.000000, 0.000000, 3.418800, 7.000000, 3.418800, -7.000000},
  {27.000000, 27.000000, 0.000000, 3.958800, 7.000000, 3.958800, -7.000000},
  {29.000000, 29.000000, 0.000000, 4.538800, 7.000000, 4.538800, -7.000000},
  {31.000000, 31.000000, 0.000000, 5.158800, 7.000000, 5.158800, -7.000000},
  {33.000000, 33.000000, 0.000000, 5.818800, 7.000000, 5.818800, -7.000000},
  {35.000000, 35.000000, 0.000000, 6.518800, 7.000000, 6.518800, -7.000000},
  {37.000000, 37.000000, 0.000000, 7.258800, 7.000000, 7.258800, -7.000000},
  {39.000000, 39.000000, 0.000000, 8.038800, 7.000000, 8.038800, -7.000000},
  {41.000000, 41.000000, 0.000000, 8.858800, 7.000000, 8.858800, -7.000000},
  {43.000000, 43.000000, 0.000000, 9.718800, 7.000000, 9.718800, -7.000000},
  {45.000000, 45.000000, 0.000000, 10.618800, 7.000000, 10.618800, -7.000000},
  {47.000000, 47.000000, 0.000000, 11.558800, 7.000000, 11.558800, -7.000000},
  {49.000000, 49.000000, 0.000000, 12.538800, 7.000000, 12.538800, -7.000000},
  {51.000000, 51.000000, 0.000000, 13.558800, 7.000000, 13.558800, -7.000000},
  {53.000000, 53.000000, 0.000000, 14.618800, 7.000000, 14.618800, -7.000000},
  {55.000000, 55.000000, 0.000000, 15.718800, 7.000000, 15.718800, -7.000000},
  {56.800000, 56.800000, 0.000000, 16.854800, 7.000000, 16.854800, -7.000000},
  {58.200000, 58.200000, 0.000000, 18.018800, 7.000000, 18.018800, -7.000000},
  {59.200000, 59.200000, 0.000000, 19.202800, 7.000000, 19.202800, -7.000000},
  {59.800000, 59.800000, 0.000000, 20.398800, 7.000000, 20.398800, -7.000000},
  {60.000000, 60.000000, 0.000000, 21.598800, 7.000000, 21.598800, -7.000000},
  {60.000000, 60.000000, 0.000000, 22.798800, 7.000000, 22.798800, -7.000000},
  {60.000000, 60.000000, 0.000000, 23.998800, 7.000000, 23.998800, -7.000000},
  {60.000000, 60.000000, 0.000000, 25.198800, 7.000000, 25.198800, -7.000000},
  {60.000000, 60.000000, 0.000000, 26.398800, 7.000000, 26.398800, -7.000000},
  {60.000000, 60.000000, 0.000000, 27.598800, 7.000000, 27.598800, -7.000000},
  {60.000000, 60.000000, 0.000000, 28.798800, 7.000000, 28.798800, -7.000000},
  {60.000000, 60.000000, 0.000000, 29.998800, 7.000000, 29.998800, -7.000000},
  {60.000000, 60.000000, 0.000000, 31.198800, 7.000000, 31.198800, -7.000000},
  {60.000000, 60.000000, 0.000000, 32.398800, 7.000000, 32.398800, -7.000000},
  {60.000000, 60.000000, 0.000000, 33.598800, 7.000000, 33.598800, -7.000000},
  {60.000000, 60.000000, 0.000000, 34.798800, 7.000000, 34.798800, -7.000000},
  {60.000000, 60.000000, 0.000000, 35.998800, 7.000000, 35.998800, -7.000000},
  {60.000000, 60.000000, 0.000000, 37.198800, 7.000000, 37.198800, -7.000000},
  {60.000000, 60.000000, 0.000000, 38.398800, 7.000000, 38.398800, -7.000000},
  {60.000000, 60.000000, 0.000000, 39.598800, 7.000000, 39.598800, -7.000000},
  {60.000000, 60.000000, 0.000000, 40.798800, 7.000000, 40.798800, -7.000000},
  {60.000000, 60.000000, 0.000000, 41.998800, 7.000000, 41.998800, -7.000000},
  {60.000000, 60.000000, 0.000000, 43.198800, 7.000000, 43.198800, -7.000000},
  {60.000000, 60.000000, 0.000000, 44.398800, 7.000000, 44.398800, -7.000000},
  {60.000000, 60.000000, 0.000000, 45.598800, 7.000000, 45.598800, -7.000000},
  {60.000000, 60.000000, 0.000000, 46.798800, 7.000000, 46.798800, -7.000000},
  {60.000000, 60.000000, 0.000000, 47.998800, 7.000000, 47.998800, -7.000000},
  {60.000000, 60.000000, 0.000000, 49.198800, 7.000000, 49.198800, -7.000000},
  {60.000000, 60.000000, 0.000000, 50.398800, 7.000000, 50.398800, -7.000000},
  {60.000000, 60.000000, 0.000000, 51.598800, 7.000000, 51.598800, -7.000000},
  {60.000000, 60.000000, 0.000000, 52.798800, 7.000000, 52.798800, -7.000000},
  {60.000000, 60.000000, 0.000000, 53.998800, 7.000000, 53.998800, -7.000000},
  {60.000000, 60.000000, 0.000000, 55.198800, 7.000000, 55.198800, -7.000000},
  {60.000000, 60.000000, 0.000000, 56.398800, 7.000000, 56.398800, -7.000000},
  {60.000000, 60.000000, 0.000000, 57.598800, 7.000000, 57.598800, -7.000000},
  {60.000000, 60.000000, 0.000000, 58.798800, 7.000000, 58.798800, -7.000000},
  {60.000000, 60.000000, 0.000000, 59.998800, 7.000000, 59.998800, -7.000000},
  {60.000000, 60.000000, 0.000000, 61.198800, 7.000000, 61.198800, -7.000000},
  {60.000000, 60.000000, 0.000000, 62.398800, 7.000000, 62.398800, -7.000000},
  {60.000000, 60.000000, 0.000000, 63.598800, 7.000000, 63.598800, -7.000000},
  {60.000000, 60.000000, 0.000000, 64.798800, 7.000000, 64.798800, -7.000000},
  {60.000000, 60.000000, 0.000000, 65.998800, 7.000000, 65.998800, -7.000000},
  {60.000000, 60.000000, 0.000000, 67.198800, 7.000000, 67.198800, -7.000000},
  {60.000000, 60.000000, 0.000000, 68.398800, 7.000000, 68.398800, -7.000000},
  {60.000000, 60.000000, 0.000000, 69.598800, 7.000000, 69.598800, -7.000000},
  {60.000000, 60.000000, 0.000000, 70.798800, 7.000000, 70.798800, -7.000000},
  {60.000000, 60.000000, 0.000000, 71.998800, 7.000000, 71.998800, -7.000000},
  {60.000000, 60.000000, 0.000000, 73.198800, 7.000000, 73.198800, -7.000000},
  {60.000000, 60.000000, 0.000000, 74.398800, 7.000000, 74.398800, -7.000000},
  {60.000000, 60.000000, 0.000000, 75.598800, 7.000000, 75.598800, -7.000000},
  {60.000000, 60.000000, 0.000000, 76.798800, 7.000000, 76.798800, -7.000000},
  {60.000000, 60.000000, 0.000000, 77.998800, 7.000000, 77.998800, -7.000000},
  {60.000000, 60.000000, 0.000000, 79.198800, 7.000000, 79.198800, -7.000000},
  {60.000000, 60.000000, 0.000000, 80.398800, 7.000000, 80.398800, -7.000000},
  {60.000000, 60.000000, 0.000000, 81.598800, 7.000000, 81.598800, -7.000000},
  {60.000000, 60.000000, 0.000000, 82.798800, 7.000000, 82.798800, -7.000000},
  {60.000000, 60.000000, 0.000000, 83.998800, 7.000000, 83.998800, -7.000000},
  {60.000000, 60.000000, 0.000000, 85.198800, 7.000000, 85.198800, -7.000000},
  {60.000000, 60.000000, 0.000000, 86.398800, 7.000000, 86.398800, -7.000000},
  {60.000000, 60.000000, 0.000000, 87.598800, 7.000000, 87.598800, -7.000000},
  {60.000000, 60.000000, 0.000000, 88.798800, 7.000000, 88.798800, -7.000000},
  {60.000000, 60.000000, 0.000000, 89.998800, 7.000000, 89.998800, -7.000000},
  {60.000000, 60.000000, 0.000000, 91.198800, 7.000000, 91.198800, -7.000000},
  {60.000000, 60.000000, 0.000000, 92.398800, 7.000000, 92.398800, -7.000000},
  {60.000000, 60.000000, 0.000000, 93.598800, 7.000000, 93.598800, -7.000000},
  {60.000000, 60.000000, 0.000000, 94.798800, 7.000000, 94.798800, -7.000000},
  {60.000000, 60.000000, 0.000000, 95.998800, 7.000000, 95.998800, -7.000000},
  {60.000000, 60.000000, 0.000000, 97.198800, 7.000000, 97.198800, -7.000000},
  {60.000000, 60.000000, 0.000000, 98.398800, 7.000000, 98.398800, -7.000000},
  {60.000000, 60.000000, 0.000000, 99.598800, 7.000000, 99.598800, -7.000000},
  {59.800200, 59.800200, 0.000000, 100.794804, 7.000000, 100.794804, -7.000000},
  {59.200600, 59.200600, 0.000000, 101.978816, 7.000000, 101.978816, -7.000000},
  {58.201000, 58.201000, 0.000000, 103.142836, 7.000000, 103.142836, -7.000000},
  {56.801400, 56.801400, 0.000000, 104.278864, 7.000000, 104.278864, -7.000000},
  {55.001800, 55.001800, 0.000000, 105.378900, 7.000000, 105.378900, -7.000000},
  {53.002000, 53.002000, 0.000000, 106.438940, 7.000000, 106.438940, -7.000000},
  {51.002000, 51.002000, 0.000000, 107.458980, 7.000000, 107.458980, -7.000000},
  {49.002000, 49.002000, 0.000000, 108.439020, 7.000000, 108.439020, -7.000000},
  {47.002000, 47.002000, 0.000000, 109.379060, 7.000000, 109.379060, -7.000000},
  {45.002000, 45.002000, 0.000000, 110.279100, 7.000000, 110.279100, -7.000000},
  {43.002000, 43.002000, 0.000000, 111.139140, 7.000000, 111.139140, -7.000000},
  {41.002000, 41.002000, 0.000000, 111.959180, 7.000000, 111.959180, -7.000000},
  {39.002000, 39.002000, 0.000000, 112.739220, 7.000000, 112.739220, -7.000000},
  {37.002000, 37.002000, 0.000000, 113.479260, 7.000000, 113.479260, -7.000000},
  {35.002000, 35.002000, 0.000000, 114.179300, 7.000000, 114.179300, -7.000000},
  {33.002000, 33.002000, 0.000000, 114.839340, 7.000000, 114.839340, -7.000000},
  {31.002000, 31.002000, 0.000000, 115.459380, 7.000000, 115.459380, -7.000000},
  {29.002000, 29.002000, 0.000000, 116.039420, 7.000000, 116.039420, -7.000000},
  {27.002000, 27.002000, 0.000000, 116.579460, 7.000000, 116.579460, -7.000000},
  {25.002000, 25.002000, 0.000000, 117.079500, 7.000000, 117.079500, -7.000000},
  {23.002000, 23.002000, 0.000000, 117.539540, 7.000000, 117.539540, -7.000000},
  {21.002000, 21.002000, 0.000000, 117.959580, 7.000000, 117.959580, -7.000000},
  {19.002000, 19.002000, 0.000000, 118.339620, 7.000000, 118.339620, -7.000000},
  {17.002000, 17.002000, 0.000000, 118.679660, 7.000000, 118.679660, -7.000000},
  {15.002000, 15.002000, 0.000000, 118.979700, 7.000000, 118.979700, -7.000000},
  {13.002000, 13.002000, 0.000000, 119.239740, 7.000000, 119.239740, -7.000000},
  {11.002000, 11.002000, 0.000000, 119.459780, 7.000000, 119.459780, -7.000000},
  {9.002000, 9.002000, 0.000000, 119.639820, 7.000000, 119.639820, -7.000000},
  {7.002000, 7.002000, 0.000000, 119.779860, 7.000000, 119.779860, -7.000000},
  {5.002000, 5.002000, 0.000000, 119.879900, 7.000000, 119.879900, -7.000000},
  {3.201800, 3.201800, 0.000000, 119.943936, 7.000000, 119.943936, -7.000000},
  {1.801400, 1.801400, 0.000000, 119.979964, 7.000000, 119.979964, -7.000000},
  {0.801000, 0.801000, 0.000000, 119.995984, 7.000000, 119.995984, -7.000000},
  {0.200600, 0.200600, 0.000000, 119.999996, 7.000000, 119.999996, -7.000000},
  {0.000200, 0.000200, 0.000000, 120.000000, 7.000000, 120.000000, -7.000000},
  {0.000000, 0.000000, 0.000000, 120.000000, 7.000000, 120.000000, -7.000000}};
}}
