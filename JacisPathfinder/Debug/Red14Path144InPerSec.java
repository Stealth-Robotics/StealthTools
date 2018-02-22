/* Red14 Feed Scale from front aud */
//----------------------------------------------------------------------------////  $Workfile: Red14Path144InPerSec.java$////  $Revision: X$////  Project:    Paths////                            Copyright (c) 2018//                               Cedarcrest High School//                            All Rights Reserved////  Modification History://  $Log://  $////----------------------------------------------------------------------------//////----------------------------------------------------------------------------//    Parameters Used//----------------------------------------------------------------------------//   Time Slice= 0.020000//   Max Vel   = 144.000000//   Max Accel = 600.000000//   Max Jerk  = 1000.000000package org.usfirst.frc4089.Stealth2018.MPPaths;

public class Red14Path144InPerSec extends Path {
    public Red14Path144InPerSec() {
       kNumPoints =158;
       kPoints = new double[][]{
  {0.400000, 0.400000, 0.002041, 16.002496, 50.750000, 16.002995, 36.750000},
  {0.727349, 0.872651, 0.013934, 16.017043, 50.750002, 16.020448, 36.750002},
  {1.636604, 1.963396, 0.040682, 16.049775, 50.750018, 16.059716, 36.750021},
  {2.909760, 3.490240, 0.088195, 16.107970, 50.750083, 16.129520, 36.750100},
  {4.547118, 5.452881, 0.162333, 16.198912, 50.750282, 16.238578, 36.750338},
  {6.549181, 7.850817, 0.268874, 16.329895, 50.750775, 16.395593, 36.750929},
  {8.916704, 10.683291, 0.413471, 16.508226, 50.751837, 16.609255, 36.752201},
  {11.650758, 13.949231, 0.601603, 16.741232, 50.753901, 16.888229, 36.754673},
  {14.752799, 17.647178, 0.838512, 17.036265, 50.757610, 17.241144, 36.759109},
  {18.224747, 21.775210, 1.129121, 17.400706, 50.763870, 17.676584, 36.766588},
  {22.069078, 26.330847, 1.477952, 17.841973, 50.773913, 18.203065, 36.778570},
  {26.288927, 31.310950, 1.889012, 18.367525, 50.789363, 18.829014, 36.796971},
  {30.705317, 36.494491, 2.362863, 18.981208, 50.812150, 19.558401, 36.824053},
  {35.138835, 41.660887, 2.896702, 19.683244, 50.844404, 20.390741, 36.862292},
  {39.592556, 46.807062, 3.487220, 20.473866, 50.888509, 21.325429, 36.914432},
  {44.069715, 51.929780, 4.130579, 21.353312, 50.947078, 22.361729, 36.983443},
  {48.573680, 57.025679, 4.822389, 22.321820, 51.022927, 23.498761, 37.072486},
  {53.107895, 62.091314, 5.557696, 23.379620, 51.119044, 24.735493, 37.184855},
  {57.675826, 67.123227, 6.330983, 24.526930, 51.238548, 26.070735, 37.323928},
  {62.280886, 72.118008, 7.136169, 25.763949, 51.384657, 27.503139, 37.493105},
  {66.926356, 77.072383, 7.966640, 27.090858, 51.560630, 29.031209, 37.695745},
  {71.615312, 81.983284, 8.815277, 28.507819, 51.769727, 30.653310, 37.935101},
  {76.350546, 86.847925, 9.674508, 30.014979, 52.015151, 32.367690, 38.214254},
  {81.134512, 91.663859, 10.536355, 31.612474, 52.299990, 34.172505, 38.536043},
  {85.969287, 96.429016, 11.392503, 33.300445, 52.627160, 36.065852, 38.903002},
  {90.856554, 101.141718, 12.234363, 35.079054, 52.999339, 38.045807, 39.317294},
  {95.797620, 105.800664, 13.053130, 36.948501, 53.418909, 40.110464, 39.780655},
  {100.793463, 110.404878, 13.839841, 38.909059, 53.887890, 42.257980, 40.294335},
  {105.844809, 114.953633, 14.585414, 40.961098, 54.407876, 44.486620, 40.859049},
  {110.952243, 119.446343, 15.280670, 43.105128, 54.979973, 46.794794, 41.474924},
  {115.922359, 123.676416, 15.915351, 45.338107, 55.603672, 49.177144, 42.140322},
  {120.557228, 127.441772, 16.478861, 47.653453, 56.276569, 51.624715, 42.851627},
  {124.850162, 130.749081, 16.961695, 50.044779, 56.995307, 54.129032, 43.604307},
  {128.794625, 133.604858, 17.355419, 52.505909, 57.755614, 56.682084, 44.392996},
  {132.384393, 136.015306, 17.652614, 55.030873, 58.552361, 59.276304, 45.211585},
  {135.613639, 137.986229, 17.846813, 57.613901, 59.379638, 61.904525, 46.053327},
  {138.476949, 139.523024, 17.932436, 60.249394, 60.230829, 64.559928, 46.910945},
  {140.969256, 140.630740, 17.904728, 62.931879, 61.098704, 67.235971, 47.776738},
  {143.085726, 141.314198, 17.759726, 65.655959, 61.975513, 69.926323, 48.642697},
  {144.821577, 141.578166, 17.494249, 68.416246, 62.853085, 72.624787, 49.500625},
  {146.171870, 141.427578, 17.105923, 71.207276, 63.722936, 75.325223, 50.342259},
  {147.131254, 140.867784, 16.593249, 74.023419, 64.576386, 78.021476, 51.159399},
  {147.899663, 140.098846, 15.954740, 76.862744, 65.405818, 80.711036, 51.945110},
  {148.681569, 139.316282, 15.188175, 79.727088, 66.204581, 83.394948, 52.693593},
  {149.477845, 138.519213, 14.291190, 82.618123, 66.965740, 86.074023, 53.398989},
  {150.288297, 137.707827, 13.261450, 85.537288, 67.682054, 88.748817, 54.055386},
  {151.111382, 136.883660, 12.096875, 88.485692, 68.345962, 91.419605, 54.656837},
  {151.943904, 136.049908, 10.795906, 91.464015, 68.949592, 94.086371, 55.197383},
  {152.780709, 135.211731, 9.357828, 94.472393, 69.484796, 96.748789, 55.671107},
  {153.614409, 134.376525, 7.783136, 97.510289, 69.943210, 99.406224, 56.072181},
  {154.435206, 133.554114, 6.073932, 100.576370, 70.316356, 102.057733, 56.394950},
  {155.230854, 132.756774, 4.234325, 103.668386, 70.595793, 104.702085, 56.634007},
  {155.986863, 131.999041, 2.270798, 106.783069, 70.773300, 107.337785, 56.784294},
  {156.686992, 131.297215, 0.192503, 109.916075, 70.841116, 109.963113, 56.841195},
  {157.314077, 130.668530, 358.011404, 113.061977, 70.792206, 112.576168, 56.800637},
  {157.851173, 130.130000, 355.742246, 116.214329, 70.620537, 115.174921, 56.659175},
  {158.282933, 129.697046, 353.402297, 119.365818, 70.321353, 117.757256, 56.414070},
  {158.597043, 129.382046, 351.010844, 122.508485, 69.891403, 120.321019, 56.063352},
  {158.785493, 129.193052, 348.588489, 125.634016, 69.329099, 122.864056, 55.605859},
  {158.845473, 129.132897, 346.156299, 128.734081, 68.634582, 125.384243, 55.041253},
  {158.779705, 129.198857, 343.734893, 131.800665, 67.809681, 127.879515, 54.370017},
  {158.596151, 129.382942, 341.343586, 134.826385, 66.857781, 130.347892, 53.593426},
  {158.307160, 129.672754, 338.999665, 137.804738, 65.783595, 132.787510, 52.713499},
  {157.928231, 130.052734, 336.717873, 140.730274, 64.592895, 135.196648, 51.732919},
  {153.462449, 134.320292, 335.151002, 143.527507, 63.329654, 137.644312, 50.625796},
  {139.851252, 148.147063, 335.830027, 146.072483, 62.169288, 140.340253, 49.396601},
  {139.792525, 148.205741, 336.518663, 148.630051, 61.039854, 143.051747, 48.199195},
  {139.734315, 148.263904, 337.216823, 151.200031, 59.941911, 145.778602, 47.034235},
  {139.676742, 148.321428, 337.924405, 153.782221, 58.876017, 148.520607, 45.902374},
  {139.619935, 148.378186, 338.641282, 156.376406, 57.842724, 151.277524, 44.804266},
  {139.564027, 148.434046, 339.367308, 158.982352, 56.842579, 154.049093, 43.740558},
  {139.509156, 148.488870, 340.102313, 161.599809, 55.876120, 156.835026, 42.711894},
  {139.455460, 148.542517, 340.846104, 164.228507, 54.943878, 159.635014, 41.718909},
  {139.403086, 148.594845, 341.598465, 166.868162, 54.046375, 162.448719, 40.762229},
  {139.352178, 148.645706, 342.359156, 169.518468, 53.184118, 165.275778, 39.842470},
  {139.302886, 148.694954, 343.127913, 172.179106, 52.357604, 168.115802, 38.960233},
  {139.255358, 148.742438, 343.904447, 174.849736, 51.567314, 170.968375, 38.116105},
  {139.209743, 148.788010, 344.688445, 177.530002, 50.813714, 173.833056, 37.310656},
  {139.166189, 148.831524, 345.479570, 180.219532, 50.097252, 176.709380, 36.544436},
  {139.124841, 148.872832, 346.277460, 182.917937, 49.418355, 179.596853, 35.817973},
  {139.085842, 148.911793, 347.081732, 185.624811, 48.777433, 182.494959, 35.131773},
  {139.049331, 148.948270, 347.891978, 188.339734, 48.174870, 185.403157, 34.486316},
  {139.015439, 148.982128, 348.707769, 191.062270, 47.611031, 188.320885, 33.882054},
  {138.984294, 149.013243, 349.528657, 193.791970, 47.086253, 191.247558, 33.319410},
  {138.956014, 149.041495, 350.354172, 196.528371, 46.600848, 194.182568, 32.798775},
  {138.930709, 149.066775, 351.183827, 199.270998, 46.155099, 197.125291, 32.320507},
  {138.908479, 149.088983, 352.017120, 202.019366, 45.749265, 200.075085, 31.884930},
  {138.889413, 149.108029, 352.853533, 204.772977, 45.383570, 203.031290, 31.492331},
  {138.873590, 149.123837, 353.692534, 207.531327, 45.058213, 205.993233, 31.142960},
  {138.861074, 149.136340, 354.533584, 210.293901, 44.773357, 208.960229, 30.837026},
  {138.851918, 149.145487, 355.376132, 213.060180, 44.529138, 211.931582, 30.574702},
  {138.846160, 149.151239, 356.219621, 215.829638, 44.325655, 214.906587, 30.356118},
  {138.843826, 149.153571, 357.063493, 218.601745, 44.162979, 217.884535, 30.181362},
  {138.844926, 149.152472, 357.907185, 221.375970, 44.041145, 220.864712, 30.050483},
  {138.849455, 149.147948, 358.750136, 224.151778, 43.960155, 223.846402, 29.963485},
  {138.857396, 149.140015, 359.591787, 226.928635, 43.919979, 226.828891, 29.920334},
  {138.868715, 149.128707, 0.431586, 229.706009, 43.920554, 229.811465, 29.920951},
  {138.883367, 149.114069, 1.268988, 232.483370, 43.961785, 232.793417, 29.965219},
  {138.901293, 149.096162, 2.103457, 235.260193, 44.043544, 235.774049, 30.052978},
  {138.922419, 149.075057, 2.934469, 238.035956, 44.165673, 238.752669, 30.184031},
  {138.946661, 149.050839, 3.761514, 240.810145, 44.327981, 241.728596, 30.358140},
  {138.973924, 149.023602, 4.584099, 243.582254, 44.530249, 244.701166, 30.575034},
  {139.004103, 148.993453, 5.401745, 246.351785, 44.772228, 247.669726, 30.834401},
  {139.037082, 148.960506, 6.213995, 249.118250, 45.053643, 250.633641, 31.135900},
  {139.072738, 148.924884, 7.020411, 251.881173, 45.374191, 253.592294, 31.479153},
  {139.110942, 148.886718, 7.820575, 254.640087, 45.733542, 256.545086, 31.863755},
  {139.151557, 148.846142, 8.614094, 257.394541, 46.131344, 259.491441, 32.289270},
  {139.194442, 148.803297, 9.400596, 260.144095, 46.567223, 262.430802, 32.755236},
  {139.239452, 148.758328, 10.179733, 262.888324, 47.040780, 265.362636, 33.261166},
  {139.286442, 148.711383, 10.951180, 265.626818, 47.551600, 268.286433, 33.806548},
  {139.335261, 148.662608, 11.714640, 268.359181, 48.099247, 271.201706, 34.390854},
  {139.385762, 148.612154, 12.469836, 271.085034, 48.683269, 274.107992, 35.013532},
  {139.437794, 148.560168, 13.216517, 273.804013, 49.303198, 277.004854, 35.674016},
  {139.491211, 148.506798, 13.954458, 276.515771, 49.958553, 279.891879, 36.371725},
  {139.545868, 148.452190, 14.683456, 279.219976, 50.648838, 282.768677, 37.106064},
  {139.517637, 148.307245, 15.402900, 281.914695, 51.373102, 285.633163, 37.875955},
  {139.212236, 147.866254, 16.111245, 284.594302, 52.129227, 288.479347, 38.679081},
  {138.519035, 147.013079, 16.806496, 287.251151, 52.914188, 291.299115, 39.512174},
  {137.437251, 145.748497, 17.486785, 289.877712, 53.724604, 294.084513, 40.371597},
  {135.966109, 144.073283, 18.150370, 292.466577, 54.556763, 296.827744, 41.253372},
  {134.104850, 141.988194, 18.795634, 295.010463, 55.406641, 299.521173, 42.153207},
  {131.852743, 139.493959, 19.421079, 297.502208, 56.269928, 302.157322, 43.066523},
  {129.209092, 136.591272, 20.025322, 299.934775, 57.142059, 304.728871, 43.988480},
  {126.173247, 133.280781, 20.607084, 302.301247, 58.018236, 307.228650, 44.914012},
  {122.744609, 129.563081, 21.165185, 304.594820, 58.893460, 309.649632, 45.837853},
  {118.922640, 125.438712, 21.698535, 306.808800, 59.762557, 311.984922, 46.754569},
  {114.706860, 120.908150, 22.206119, 308.936593, 60.620207, 314.227749, 47.658584},
  {110.181236, 116.060652, 22.687357, 310.973253, 61.461623, 316.373087, 48.544898},
  {105.540430, 111.101554, 23.142543, 312.917478, 62.283512, 318.419757, 49.410093},
  {100.895028, 106.147040, 23.572427, 314.770003, 63.083557, 320.368715, 50.251783},
  {96.245241, 101.196901, 23.977727, 316.531546, 63.859589, 322.220887, 51.067740},
  {91.591288, 96.250918, 24.359123, 318.202800, 64.609586, 323.977165, 51.855891},
  {86.933389, 91.308875, 24.717262, 319.784436, 65.331662, 325.638406, 52.614311},
  {77.351760, 91.260945, 25.855783, 321.187469, 65.983442, 327.292973, 53.384917},
  {36.522406, 122.422283, 32.891363, 321.828383, 66.333847, 329.431053, 54.578022},
  {0.881755, 149.020645, 45.100709, 321.841446, 66.321999, 331.758326, 56.439920},
  {63.345610, 202.303995, 66.975605, 321.155415, 65.256904, 334.040153, 59.781181},
  {101.532713, 230.499371, 94.414239, 320.824565, 63.253383, 334.783036, 64.330919},
  {48.641085, 168.773293, 112.281229, 321.031136, 62.302746, 333.985812, 67.610889},
  {3.325635, 107.645544, 120.830572, 320.993196, 62.357377, 333.014808, 69.532392},
  {28.112114, 73.313039, 124.531007, 320.685795, 62.828143, 332.219268, 70.764073},
  {39.649637, 52.191630, 125.557601, 320.228302, 63.475860, 331.617740, 71.617156},
  {45.492245, 36.749738, 124.842011, 319.702321, 64.218263, 331.192548, 72.216680},
  {48.651955, 23.986848, 122.823035, 319.159316, 65.025698, 330.924199, 72.614343},
  {50.166331, 12.868837, 119.769823, 318.637044, 65.882375, 330.789423, 72.833611},
  {50.219706, 3.386391, 115.935719, 318.166892, 66.769936, 330.756886, 72.893011},
  {48.782954, 4.031556, 111.611782, 317.772894, 67.662503, 330.788704, 72.818924},
  {45.714063, 9.014786, 107.131027, 317.469175, 68.524863, 330.848046, 72.648673},
  {40.920823, 11.472579, 102.841567, 317.257275, 69.315372, 330.907112, 72.426955},
  {34.722262, 11.724894, 99.039118, 317.125348, 69.997171, 330.951486, 72.196694},
  {27.768489, 10.422593, 95.912748, 317.053035, 70.547813, 330.978554, 71.990007},
  {20.790128, 8.296524, 93.531802, 317.018785, 70.962202, 330.992196, 71.824638},
  {14.389397, 5.948736, 91.867045, 317.005226, 71.249671, 330.997794, 71.705795},
  {8.969675, 3.782335, 90.823264, 317.001013, 71.429015, 330.999568, 71.630169},
  {4.763513, 2.029658, 90.267233, 317.000107, 71.524281, 330.999954, 71.589578},
  {1.888988, 0.808670, 90.046427, 317.000003, 71.562060, 330.999999, 71.573404},
  {0.396991, 0.170218, 90.000000, 317.000000, 71.570000, 331.000000, 71.570000},
  {0.000000, 0.000000, 90.000000, 317.000000, 71.570000, 331.000000, 71.570000}};
}}