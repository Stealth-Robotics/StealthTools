//----------------------------------------------------------------------------
//
//  $Workfile: Red21Path60InPerSec.java
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

public class Red21Path60InPerSec extends Path {
    public Red21Path60InPerSec() {
       kNumPoints = 121;
       kPoints = new double[][]{
  {0.400000, 0.400000, 359.999802, 16.003064, 242.750000, 16.003015, 228.750000},
  {0.806375, 0.793625, 359.998758, 16.019191, 242.750000, 16.018888, 228.750000},
  {1.814335, 1.785665, 359.996412, 16.055478, 242.749998, 16.054601, 228.749998},
  {3.225458, 3.174542, 359.992244, 16.119987, 242.749992, 16.118092, 228.749992},
  {5.039710, 4.960290, 359.985743, 16.220781, 242.749973, 16.217298, 228.749973},
  {7.055455, 6.944545, 359.976665, 16.361891, 242.749926, 16.356189, 228.749927},
  {9.071061, 8.928939, 359.965033, 16.543312, 242.749834, 16.534768, 228.749837},
  {11.086488, 10.913512, 359.950874, 16.765041, 242.749671, 16.753038, 228.749676},
  {13.101696, 12.898303, 359.934226, 17.027075, 242.749408, 17.011004, 228.749418},
  {15.116647, 14.883353, 359.915131, 17.329408, 242.749011, 17.308670, 228.749026},
  {17.131299, 16.868701, 359.893637, 17.672033, 242.748439, 17.646044, 228.748463},
  {19.145613, 18.854387, 359.869800, 18.054945, 242.747648, 18.023131, 228.747685},
  {21.159550, 20.840450, 359.843681, 18.478135, 242.746590, 18.439939, 228.746642},
  {23.173069, 22.826931, 359.815349, 18.941594, 242.745211, 18.896475, 228.745284},
  {25.186130, 24.813869, 359.784879, 19.445313, 242.743453, 19.392750, 228.743552},
  {27.198694, 26.801305, 359.752353, 19.989283, 242.741256, 19.928771, 228.741387},
  {29.210721, 28.789278, 359.717857, 20.573491, 242.738555, 20.504551, 228.738725},
  {31.222171, 30.777828, 359.681487, 21.197926, 242.735281, 21.120099, 228.735498},
  {33.233003, 32.766996, 359.643344, 21.862574, 242.731365, 21.775427, 228.731636},
  {35.243179, 34.756820, 359.603535, 22.567423, 242.726731, 22.470549, 228.727067},
  {37.252657, 36.747341, 359.562174, 23.312456, 242.721306, 23.205476, 228.721715},
  {39.261398, 38.738600, 359.519383, 24.097659, 242.715012, 23.980223, 228.715504},
  {41.269362, 40.730636, 359.475287, 24.923014, 242.707770, 24.794805, 228.708357},
  {43.276509, 42.723488, 359.430022, 25.788505, 242.699500, 25.649235, 228.700193},
  {45.282799, 44.717198, 359.383727, 26.694112, 242.690123, 26.543531, 228.690933},
  {47.288192, 46.711805, 359.336549, 27.639817, 242.679560, 27.477709, 228.680499},
  {49.292648, 48.707349, 359.288641, 28.625599, 242.667731, 28.451786, 228.668810},
  {51.296127, 50.703870, 359.240164, 29.651437, 242.654558, 29.465780, 228.655789},
  {53.298589, 52.701408, 359.191285, 30.717309, 242.639964, 30.519709, 228.641359},
  {55.299994, 54.700003, 359.142175, 31.823192, 242.623878, 31.613594, 228.625447},
  {57.099267, 56.500729, 359.093184, 32.965042, 242.606289, 32.743474, 228.608043},
  {58.495529, 57.904468, 359.044805, 34.134798, 242.587277, 33.901411, 228.589223},
  {59.489064, 58.910933, 358.997484, 35.324406, 242.566948, 35.079457, 228.569091},
  {60.080175, 59.519822, 358.951618, 36.525817, 242.545440, 36.269663, 228.547784},
  {60.269173, 59.730824, 358.907554, 37.730990, 242.522919, 37.464071, 228.525464},
  {60.257214, 59.742784, 358.865447, 38.935906, 242.499496, 38.658701, 228.502241},
  {60.245254, 59.754744, 358.825298, 40.140567, 242.475213, 39.853553, 228.478155},
  {60.233294, 59.766704, 358.787107, 41.344971, 242.450111, 41.048628, 228.453248},
  {60.221334, 59.778664, 358.750874, 42.549120, 242.424233, 42.243925, 228.427560},
  {60.209374, 59.790624, 358.716599, 43.753013, 242.397618, 43.439445, 228.401130},
  {60.197415, 59.802584, 358.684282, 44.956651, 242.370310, 44.635189, 228.374001},
  {60.185455, 59.814544, 358.653922, 46.160036, 242.342348, 45.831157, 228.346212},
  {60.173495, 59.826504, 358.625521, 47.363166, 242.313776, 47.027350, 228.317804},
  {60.161536, 59.838463, 358.599077, 48.566044, 242.284633, 48.223768, 228.288818},
  {60.149577, 59.850423, 358.574591, 49.768670, 242.254961, 49.420412, 228.259294},
  {60.137618, 59.862382, 358.552062, 50.971044, 242.224803, 50.617284, 228.229273},
  {60.125659, 59.874340, 358.531492, 52.173168, 242.194198, 51.814383, 228.198796},
  {60.113700, 59.886299, 358.512879, 53.375042, 242.163188, 53.011710, 228.167903},
  {60.101742, 59.898257, 358.496223, 54.576667, 242.131814, 54.209267, 228.136636},
  {60.089784, 59.910215, 358.481525, 55.778045, 242.100118, 55.407055, 228.105034},
  {60.077826, 59.922173, 358.468785, 56.979175, 242.068141, 56.605074, 228.073140},
  {60.065869, 59.934131, 358.458002, 58.180061, 242.035923, 57.803325, 228.040993},
  {60.053912, 59.946088, 358.449177, 59.380701, 242.003507, 59.001810, 228.008635},
  {60.041954, 59.958046, 358.442308, 60.581099, 241.970932, 60.200530, 227.976106},
  {60.029997, 59.970003, 358.437398, 61.781253, 241.938241, 61.399485, 227.943447},
  {60.018040, 59.981960, 358.434445, 62.981167, 241.905474, 62.598677, 227.910700},
  {60.006083, 59.993917, 358.433449, 64.180840, 241.872672, 63.798107, 227.877904},
  {59.994126, 60.005874, 358.434410, 65.380275, 241.839876, 64.997776, 227.845102},
  {59.982169, 60.017830, 358.437329, 66.579471, 241.807127, 66.197686, 227.812334},
  {59.970212, 60.029787, 358.442205, 67.778430, 241.774466, 67.397836, 227.779641},
  {59.958255, 60.041745, 358.449039, 68.977154, 241.741935, 68.598229, 227.747064},
  {59.946298, 60.053702, 358.457830, 70.175643, 241.709573, 69.798866, 227.714644},
  {59.934341, 60.065659, 358.468579, 71.373899, 241.677422, 70.999746, 227.682423},
  {59.922383, 60.077617, 358.481285, 72.571922, 241.645523, 72.200873, 227.650441},
  {59.910425, 60.089574, 358.495948, 73.769713, 241.613916, 73.402246, 227.618740},
  {59.898467, 60.101532, 358.512569, 74.967274, 241.582643, 74.603867, 227.587360},
  {59.886509, 60.113491, 358.531148, 76.164606, 241.551743, 75.805737, 227.556343},
  {59.874550, 60.125449, 358.551684, 77.361709, 241.521258, 77.007856, 227.525731},
  {59.862592, 60.137408, 358.574178, 78.558584, 241.491229, 78.210226, 227.495564},
  {59.850632, 60.149367, 358.598630, 79.755232, 241.461696, 79.412847, 227.465883},
  {59.838673, 60.161326, 358.625040, 80.951654, 241.432700, 80.615721, 227.436731},
  {59.826714, 60.173285, 358.653407, 82.147851, 241.404282, 81.818847, 227.408148},
  {59.814754, 60.185245, 358.683732, 83.343823, 241.376481, 83.022227, 227.380176},
  {59.802794, 60.197205, 358.716015, 84.539571, 241.349340, 84.225861, 227.352855},
  {59.790834, 60.209165, 358.750256, 85.735095, 241.322898, 85.429750, 227.326228},
  {59.778874, 60.221124, 358.786455, 86.930397, 241.297196, 86.633894, 227.300336},
  {59.766913, 60.233084, 358.824611, 88.125475, 241.272275, 87.838294, 227.275221},
  {59.754953, 60.245044, 358.864726, 89.320331, 241.248175, 89.042949, 227.250923},
  {59.742994, 60.257004, 358.906798, 90.514965, 241.224936, 90.247862, 227.227484},
  {59.731034, 60.268963, 358.950828, 91.709377, 241.202600, 91.453030, 227.204947},
  {59.719075, 60.280922, 358.996816, 92.903567, 241.181206, 92.658455, 227.183352},
  {59.707116, 60.292880, 359.044761, 94.097535, 241.160795, 93.864137, 227.162741},
  {59.695159, 60.304838, 359.094664, 95.291280, 241.141408, 95.070074, 227.143155},
  {59.683202, 60.316794, 359.146524, 96.484804, 241.123084, 96.276268, 227.124637},
  {59.671246, 60.328750, 359.200342, 97.678104, 241.105864, 97.482717, 227.107228},
  {59.463788, 60.142928, 359.255930, 98.867272, 241.089840, 98.685467, 227.091021},
  {58.862178, 59.557977, 359.312882, 100.044424, 241.075134, 99.876533, 227.076141},
  {57.863387, 58.570209, 359.370736, 101.201615, 241.061837, 101.047860, 227.062681},
  {56.467716, 57.179319, 359.428982, 102.330908, 241.050005, 102.191384, 227.050700},
  {54.675451, 55.385024, 359.487061, 103.424368, 241.039659, 103.299035, 227.040220},
  {52.682176, 53.385020, 359.544590, 104.477973, 241.030753, 104.366697, 227.031195},
  {50.686620, 51.380576, 359.601391, 105.491678, 241.023195, 105.394280, 227.023534},
  {48.692082, 49.375114, 359.657298, 106.465499, 241.016894, 106.381762, 227.017144},
  {46.698521, 47.368675, 359.712151, 107.399455, 241.011753, 107.329121, 227.011930},
  {44.705898, 45.361299, 359.765796, 108.293564, 241.007678, 108.236337, 227.007795},
  {42.714172, 43.353026, 359.818087, 109.147842, 241.004575, 109.103392, 227.004645},
  {40.723303, 41.343895, 359.868883, 109.962305, 241.002349, 109.930267, 227.002385},
  {38.733251, 39.333947, 359.918051, 110.736968, 241.000907, 110.716944, 227.000922},
  {36.743976, 37.323222, 359.965463, 111.471848, 241.000159, 111.463408, 227.000162},
  {34.819623, 35.241576, 0.000000, 112.168240, 241.000000, 112.168240, 227.000000},
  {33.033600, 33.033600, 0.000000, 112.828912, 241.000000, 112.828912, 227.000000},
  {31.033600, 31.033600, 0.000000, 113.449584, 241.000000, 113.449584, 227.000000},
  {29.033600, 29.033600, 0.000000, 114.030256, 241.000000, 114.030256, 227.000000},
  {27.033600, 27.033600, 0.000000, 114.570928, 241.000000, 114.570928, 227.000000},
  {25.033600, 25.033600, 0.000000, 115.071600, 241.000000, 115.071600, 227.000000},
  {23.033600, 23.033600, 0.000000, 115.532272, 241.000000, 115.532272, 227.000000},
  {21.033600, 21.033600, 0.000000, 115.952944, 241.000000, 115.952944, 227.000000},
  {19.033600, 19.033600, 0.000000, 116.333616, 241.000000, 116.333616, 227.000000},
  {17.033600, 17.033600, 0.000000, 116.674288, 241.000000, 116.674288, 227.000000},
  {15.033600, 15.033600, 0.000000, 116.974960, 241.000000, 116.974960, 227.000000},
  {13.033600, 13.033600, 0.000000, 117.235632, 241.000000, 117.235632, 227.000000},
  {11.033600, 11.033600, 0.000000, 117.456304, 241.000000, 117.456304, 227.000000},
  {9.033600, 9.033600, 0.000000, 117.636976, 241.000000, 117.636976, 227.000000},
  {7.033600, 7.033600, 0.000000, 117.777648, 241.000000, 117.777648, 227.000000},
  {5.033600, 5.033600, 0.000000, 117.878320, 241.000000, 117.878320, 227.000000},
  {3.230240, 3.230240, 0.000000, 117.942925, 241.000000, 117.942925, 227.000000},
  {1.823520, 1.823520, 0.000000, 117.979395, 241.000000, 117.979395, 227.000000},
  {0.816800, 0.816800, 0.000000, 117.995731, 241.000000, 117.995731, 227.000000},
  {0.210080, 0.210080, 0.000000, 117.999933, 241.000000, 117.999933, 227.000000},
  {0.003360, 0.003360, 0.000000, 118.000000, 241.000000, 118.000000, 227.000000},
  {0.000000, 0.000000, 0.000000, 118.000000, 241.000000, 118.000000, 227.000000}};
}}
