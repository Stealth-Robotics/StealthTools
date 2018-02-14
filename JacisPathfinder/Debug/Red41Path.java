/* Red41 Feed Switch from Side start 4 aud */
//----------------------------------------------------------------------------////  $Workfile: Red41Path.java$////  $Revision: X$////  Project:    Paths////                            Copyright (c) 2018//                               Cedarcrest High School//                            All Rights Reserved////  Modification History://  $Log://  $////----------------------------------------------------------------------------//////----------------------------------------------------------------------------//    Parameters Used//----------------------------------------------------------------------------//   Time Slice= 0.020000//   Max Vel   = 144.000000//   Max Accel = 600.000000//   Max Jerk  = 1000.000000package org.usfirst.frc4089.paths;
public class Red41Path extends Path {
    public Red41Path() {
       kNumPoints =105;
       kPoints = new double[][]{
  {0.400000, 0.400000, 359.998733, 16.001865, 189.250000, 16.001555, 175.250000},
  {0.872421, 0.727579, 359.986878, 16.019313, 189.249998, 16.016107, 175.249998},
  {1.963103, 1.636897, 359.960178, 16.058575, 189.249980, 16.048845, 175.249983},
  {3.490497, 2.909503, 359.912623, 16.128385, 189.249902, 16.107035, 175.249918},
  {5.455279, 4.544720, 359.838092, 16.237490, 189.249665, 16.197929, 175.249721},
  {7.858567, 6.541432, 359.730283, 16.394661, 189.249073, 16.328757, 175.249228},
  {10.702041, 8.897954, 359.582617, 16.608698, 189.247790, 16.506712, 175.248161},
  {13.988088, 11.611900, 359.388123, 16.888448, 189.245278, 16.738941, 175.246076},
  {17.719955, 14.680019, 359.139301, 17.242818, 189.240725, 17.032517, 175.242305},
  {21.901941, 18.098011, 358.827944, 17.680788, 189.232958, 17.394421, 175.235887},
  {26.539615, 21.860295, 358.444936, 18.211430, 189.220332, 17.831503, 175.225488},
  {31.640100, 25.959742, 357.979991, 18.843924, 189.200601, 18.350445, 175.209301},
  {36.991958, 30.207779, 357.424696, 19.583169, 189.170954, 18.954116, 175.185093},
  {42.383018, 34.416568, 356.772630, 20.429744, 189.128069, 19.641566, 175.150274},
  {47.821042, 38.578330, 356.016098, 21.384274, 189.067952, 20.411607, 175.101782},
  {53.314669, 42.684409, 355.145991, 22.447401, 188.985840, 21.262761, 175.036050},
  {58.873486, 46.725188, 354.151627, 23.619744, 188.876089, 22.193198, 174.948959},
  {64.508046, 50.690083, 353.020593, 24.901839, 188.732048, 23.200663, 174.835789},
  {70.229761, 54.567636, 351.738604, 26.294044, 188.545893, 24.282392, 174.691174},
  {76.050609, 58.345809, 350.289408, 27.796410, 188.308462, 25.435008, 174.509050},
  {81.982516, 62.012601, 348.654793, 29.408492, 188.009052, 26.654414, 174.282615},
  {88.036238, 65.557161, 346.814776, 31.129075, 187.635229, 27.935678, 174.004300},
  {94.219498, 68.971655, 344.748100, 32.955806, 187.172649, 29.272921, 173.665748},
  {100.534090, 72.254156, 342.433198, 34.884681, 186.604949, 30.659235, 173.257829},
  {106.971702, 75.412846, 339.849852, 36.909391, 185.913772, 32.086650, 172.770668},
  {113.508404, 78.471547, 336.981746, 39.020522, 185.079032, 33.546181, 172.193708},
  {120.098300, 81.476116, 333.820076, 41.204668, 184.079559, 35.027988, 171.515777},
  {126.667886, 84.500160, 330.368079, 43.443622, 182.894238, 36.521655, 170.725164},
  {133.113962, 87.647195, 326.645919, 45.713913, 181.503730, 38.016552, 169.809687},
  {139.309046, 91.045275, 322.694693, 47.987056, 179.892636, 39.502187, 168.756793},
  {144.875930, 94.672667, 318.584618, 50.227256, 178.054942, 40.966071, 167.555873},
  {149.461194, 98.483947, 314.411151, 52.395181, 175.996895, 42.394471, 166.199662},
  {153.014006, 102.530548, 310.278128, 54.455978, 173.734500, 43.775165, 164.683520},
  {155.561277, 106.785594, 306.284976, 56.382617, 171.291594, 45.097448, 163.006369},
  {157.199689, 111.151884, 302.515231, 58.157789, 168.696703, 46.352308, 161.171370},
  {158.068341, 115.489459, 299.029564, 59.773978, 165.979688, 47.532806, 159.186036},
  {158.315436, 119.649210, 295.864287, 61.232062, 163.169083, 48.634444, 157.061708},
  {158.071923, 123.499451, 293.034204, 62.539165, 160.290510, 49.655364, 154.812582},
  {157.437873, 126.939631, 290.537691, 63.706451, 157.366110, 50.596268, 152.454582},
  {156.480252, 129.902546, 288.362145, 64.747281, 154.414653, 51.460100, 150.004344},
  {155.237571, 132.349618, 286.488657, 65.675857, 151.452014, 52.251594, 147.478457},
  {153.727085, 134.263638, 284.895501, 66.506335, 148.491759, 52.976787, 144.892962},
  {152.162120, 135.831349, 283.558779, 67.253271, 145.541604, 53.643452, 142.259405},
  {150.738014, 137.257535, 282.455369, 67.930897, 142.603986, 54.260397, 139.584479},
  {149.431804, 138.565304, 281.565925, 68.551971, 139.680595, 54.836245, 136.873661},
  {148.220964, 139.777289, 280.874797, 69.128009, 136.772682, 55.379424, 134.131393},
  {147.084078, 140.914989, 280.369849, 69.669516, 133.881270, 55.898187, 131.361249},
  {146.000982, 141.998624, 280.042251, 70.186193, 131.007325, 56.400681, 128.566084},
  {144.952605, 143.047304, 279.886300, 70.687127, 128.151880, 56.895022, 125.748170},
  {143.920656, 144.079342, 279.899288, 71.180958, 125.316145, 57.389398, 122.909309},
  {142.887225, 145.112652, 280.081442, 71.676025, 122.501609, 57.892186, 120.050939},
  {141.834368, 146.165172, 280.435924, 72.180498, 119.710139, 58.412084, 117.174238},
  {140.743670, 147.255290, 280.968909, 72.702491, 116.944089, 58.958263, 114.280221},
  {139.595872, 148.402228, 281.689724, 73.250162, 114.206415, 59.540534, 111.369851},
  {138.370558, 149.626338, 282.611032, 73.831790, 111.500814, 60.169544, 108.444178},
  {137.046038, 150.949227, 283.749042, 74.455831, 108.831878, 60.856986, 105.504503},
  {135.599544, 152.393549, 285.123683, 75.130937, 106.205259, 61.615829, 102.552610},
  {134.008005, 153.982225, 286.758651, 75.865942, 103.627853, 62.460553, 99.591081},
  {132.249804, 155.736687, 288.681169, 76.669779, 101.107962, 63.407360, 96.623738},
  {130.308082, 157.673578, 290.921208, 77.551322, 98.655420, 64.474309, 93.656248},
  {128.176337, 159.799171, 293.509793, 78.519159, 96.281613, 65.681272, 90.696931},
  {125.867011, 162.100830, 296.475903, 79.581266, 93.999305, 67.049559, 87.757805},
  {123.255538, 164.307950, 299.836571, 80.743000, 91.825105, 68.598727, 84.859717},
  {120.276147, 166.086871, 303.586900, 82.006035, 89.777842, 70.343366, 82.033027},
  {117.091025, 167.271622, 307.695118, 83.369393, 87.873800, 72.291534, 79.313365},
  {113.901806, 167.661945, 312.096528, 84.829568, 86.125276, 74.441338, 76.739933},
  {110.911677, 167.056182, 316.693251, 86.380856, 84.539695, 76.778199, 74.352008},
  {108.272311, 165.303822, 321.362636, 88.015601, 83.119566, 79.274153, 72.183977},
  {106.035037, 162.353822, 325.973635, 89.724095, 81.863215, 81.890055, 70.260292},
  {104.133007, 158.272299, 330.406102, 91.494187, 80.765832, 84.580297, 68.592166},
  {102.405235, 153.218813, 334.566164, 93.310979, 79.820316, 87.298419, 67.177170},
  {100.647832, 147.395710, 338.393241, 95.157026, 79.017756, 90.001747, 66.001493},
  {98.665221, 140.997352, 341.858702, 97.013090, 78.347712, 92.654029, 65.043630},
  {96.302219, 134.178207, 344.959272, 98.859169, 77.798497, 95.226091, 64.278114},
  {93.622571, 127.269433, 347.713571, 100.678790, 77.356846, 97.699604, 63.677502},
  {90.747950, 120.549365, 350.153034, 102.460109, 77.009027, 100.065868, 63.215274},
  {87.668935, 114.032418, 352.311042, 104.193070, 76.742224, 102.319937, 62.868098},
  {84.392160, 107.712232, 354.219904, 105.869312, 76.544676, 104.459363, 62.615855},
  {80.933584, 101.573081, 355.909331, 107.482009, 76.405727, 106.483319, 62.441393},
  {77.312971, 95.595390, 357.405810, 109.025653, 76.315820, 108.391989, 62.330168},
  {73.550682, 89.758945, 358.732504, 110.495838, 76.266450, 110.186155, 62.269875},
  {69.665996, 84.044577, 359.909428, 111.889061, 76.250088, 111.866930, 62.250105},
  {71.497362, 72.603910, 0.000000, 113.319009, 76.250000, 113.319009, 62.250000},
  {67.256638, 67.256638, 0.000000, 114.664141, 76.250000, 114.664141, 62.250000},
  {62.456638, 62.456638, 0.000000, 115.913274, 76.250000, 115.913274, 62.250000},
  {57.656638, 57.656638, 0.000000, 117.066407, 76.250000, 117.066407, 62.250000},
  {52.856638, 52.856638, 0.000000, 118.123540, 76.250000, 118.123540, 62.250000},
  {48.056638, 48.056638, 0.000000, 119.084672, 76.250000, 119.084672, 62.250000},
  {43.256638, 43.256638, 0.000000, 119.949805, 76.250000, 119.949805, 62.250000},
  {38.456638, 38.456638, 0.000000, 120.718938, 76.250000, 120.718938, 62.250000},
  {33.656638, 33.656638, 0.000000, 121.392071, 76.250000, 121.392071, 62.250000},
  {28.856638, 28.856638, 0.000000, 121.969203, 76.250000, 121.969203, 62.250000},
  {24.254278, 24.254278, 0.000000, 122.454289, 76.250000, 122.454289, 62.250000},
  {20.049558, 20.049558, 0.000000, 122.855280, 76.250000, 122.855280, 62.250000},
  {16.244838, 16.244838, 0.000000, 123.180177, 76.250000, 123.180177, 62.250000},
  {12.840118, 12.840118, 0.000000, 123.436979, 76.250000, 123.436979, 62.250000},
  {9.835399, 9.835399, 0.000000, 123.633687, 76.250000, 123.633687, 62.250000},
  {7.230679, 7.230679, 0.000000, 123.778301, 76.250000, 123.778301, 62.250000},
  {5.025959, 5.025959, 0.000000, 123.878820, 76.250000, 123.878820, 62.250000},
  {3.221239, 3.221239, 0.000000, 123.943245, 76.250000, 123.943245, 62.250000},
  {1.816519, 1.816519, 0.000000, 123.979575, 76.250000, 123.979575, 62.250000},
  {0.811800, 0.811800, 0.000000, 123.995811, 76.250000, 123.995811, 62.250000},
  {0.207080, 0.207080, 0.000000, 123.999953, 76.250000, 123.999953, 62.250000},
  {0.002360, 0.002360, 0.000000, 124.000000, 76.250000, 124.000000, 62.250000},
  {0.000000, 0.000000, 0.000000, 124.000000, 76.250000, 124.000000, 62.250000}};
}}
