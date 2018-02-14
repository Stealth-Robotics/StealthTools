/* Red15 Feed scale from side aud */
//----------------------------------------------------------------------------////  $Workfile: Red15Path.java$////  $Revision: X$////  Project:    Paths////                            Copyright (c) 2018//                               Cedarcrest High School//                            All Rights Reserved////  Modification History://  $Log://  $////----------------------------------------------------------------------------//////----------------------------------------------------------------------------//    Parameters Used//----------------------------------------------------------------------------//   Time Slice= 0.020000//   Max Vel   = 144.000000//   Max Accel = 600.000000//   Max Jerk  = 1000.000000package org.usfirst.frc4089.paths;
public class Red15Path extends Path {
    public Red15Path() {
       kNumPoints =160;
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
  {153.341715, 134.482110, 335.174130, 143.525369, 63.330786, 137.647302, 50.624554},
  {139.205560, 148.792189, 335.958813, 146.060115, 62.179118, 140.356609, 49.393579},
  {139.116037, 148.881627, 336.758144, 148.608897, 61.063315, 143.084312, 48.199453},
  {139.026634, 148.970944, 337.572104, 151.171488, 59.984199, 145.830201, 47.043154},
  {138.937573, 149.059918, 338.400636, 153.747631, 58.942591, 148.594031, 45.925663},
  {138.849094, 149.148309, 339.243646, 156.337043, 57.939311, 151.375517, 44.847968},
  {138.761450, 149.235863, 340.100996, 158.939413, 56.975171, 154.174329, 43.811055},
  {138.674911, 149.322312, 340.972506, 161.554402, 56.050980, 156.990096, 42.815909},
  {138.589761, 149.407373, 341.857948, 164.181638, 55.167534, 159.822402, 41.863509},
  {138.506298, 149.490747, 342.757047, 166.820723, 54.325615, 162.670786, 40.954826},
  {138.424828, 149.572129, 343.669476, 169.471228, 53.525993, 165.534736, 40.090814},
  {138.345671, 149.651199, 344.594856, 172.132694, 52.769414, 168.413696, 39.272412},
  {138.269152, 149.727633, 345.532756, 174.804631, 52.056605, 171.307060, 38.500537},
  {138.195600, 149.801101, 346.482690, 177.486521, 51.388268, 174.214173, 37.776077},
  {138.125350, 149.871271, 347.444118, 180.177817, 50.765075, 177.134333, 37.099892},
  {138.058732, 149.937813, 348.416446, 182.877944, 50.187665, 180.066789, 36.472804},
  {137.996073, 150.000398, 349.399025, 185.586298, 49.656644, 183.010745, 35.895593},
  {137.937692, 150.058709, 350.391156, 188.302252, 49.172580, 185.965358, 35.368996},
  {137.883899, 150.112439, 351.392089, 191.025152, 48.735998, 188.929745, 34.893698},
  {137.834984, 150.161295, 352.401024, 193.754322, 48.347380, 191.902980, 34.470329},
  {137.791222, 150.205004, 353.417119, 196.489065, 48.007161, 194.884100, 34.099462},
  {137.752863, 150.243316, 354.439490, 199.228664, 47.715727, 197.872107, 33.781605},
  {137.720132, 150.276007, 355.467216, 201.972387, 47.473410, 200.865974, 33.517198},
  {137.693225, 150.302881, 356.499345, 204.719486, 47.280492, 203.864647, 33.306615},
  {137.672304, 150.323776, 357.534896, 207.469201, 47.137198, 206.867048, 33.150153},
  {137.657497, 150.338564, 358.572869, 210.220763, 47.043694, 209.872085, 33.048036},
  {137.648897, 150.347154, 359.612250, 212.973396, 47.000090, 212.878651, 33.000411},
  {137.646557, 150.349491, 0.652014, 215.726319, 47.006440, 215.885633, 33.007346},
  {137.650491, 150.345562, 1.691134, 218.478754, 47.062734, 218.891915, 33.068831},
  {137.660675, 150.335390, 2.728587, 221.229919, 47.168906, 221.896387, 33.184779},
  {137.677046, 150.319040, 3.763363, 223.979042, 47.324832, 224.897944, 33.355021},
  {137.699503, 150.296611, 4.794464, 226.725354, 47.530329, 227.895496, 33.579316},
  {137.727907, 150.268242, 5.820918, 229.468100, 47.785159, 230.887973, 33.857347},
  {137.762088, 150.234102, 6.841780, 232.206534, 48.089027, 233.874326, 34.188723},
  {137.801843, 150.194396, 7.856137, 234.939927, 48.441587, 236.853534, 34.572985},
  {137.846939, 150.149354, 8.863117, 237.667569, 48.842439, 239.824610, 35.009609},
  {137.897121, 150.099233, 9.861886, 240.388766, 49.291137, 242.786599, 35.498009},
  {137.952109, 150.044310, 10.851658, 243.102850, 49.787188, 245.738586, 36.037538},
  {138.011606, 149.984883, 11.831696, 245.809171, 50.330055, 248.679697, 36.627498},
  {138.075302, 149.921261, 12.801313, 248.507109, 50.919161, 251.609101, 37.267141},
  {138.142875, 149.853766, 13.759874, 251.196068, 51.553891, 254.526014, 37.955676},
  {138.213996, 149.782727, 14.706798, 253.875479, 52.233597, 257.429697, 38.692270},
  {138.288333, 149.708473, 15.641559, 256.544802, 52.957598, 260.319460, 39.476057},
  {138.365554, 149.631338, 16.563686, 259.203527, 53.725188, 263.194660, 40.306140},
  {138.445329, 149.551650, 17.472760, 261.851172, 54.535633, 266.054705, 41.181596},
  {138.527336, 149.469732, 18.368417, 264.487285, 55.388180, 268.899048, 42.101482},
  {138.611257, 149.385900, 19.250342, 267.111445, 56.282058, 271.727193, 43.064839},
  {138.696787, 149.300459, 20.118273, 269.723260, 57.216479, 274.538688, 44.070694},
  {138.783633, 149.213703, 20.971994, 272.322368, 58.190644, 277.333129, 45.118067},
  {138.871514, 149.125910, 21.811335, 274.908435, 59.203744, 280.110157, 46.205971},
  {138.960165, 149.037348, 22.636171, 277.481159, 60.254963, 282.869452, 47.333419},
  {139.049335, 148.948266, 23.446416, 280.040263, 61.343482, 285.610741, 48.499426},
  {139.138789, 148.858897, 24.242024, 282.585499, 62.468479, 288.333786, 49.703010},
  {138.490386, 149.039222, 25.105466, 285.103248, 63.622937, 291.043250, 50.945540},
  {115.246853, 171.123322, 29.680249, 287.153185, 64.676736, 294.085414, 52.513505},
  {99.172878, 185.126175, 36.720142, 288.820441, 65.751148, 297.191139, 54.529231},
  {68.817737, 212.401837, 48.494091, 289.851439, 66.662957, 300.335863, 57.385195},
  {16.952202, 259.862554, 68.486481, 290.060760, 66.929670, 303.085395, 61.795579},
  {17.671762, 289.165882, 93.806978, 290.003620, 66.580884, 303.972728, 67.510420},
  {21.825906, 245.676370, 112.214562, 289.868898, 66.996092, 302.829742, 72.289158},
  {70.824924, 191.483072, 122.103366, 289.200928, 68.245206, 301.060198, 75.685482},
  {97.709397, 158.071429, 127.045664, 288.081368, 69.846903, 299.255547, 78.281222},
  {111.469495, 136.807082, 129.119698, 286.699987, 71.596751, 297.561600, 80.429947},
  {119.534736, 120.385886, 129.189366, 285.185605, 73.446634, 296.036470, 82.293030},
  {125.757642, 104.985151, 127.489046, 283.620521, 75.415518, 294.729097, 83.936054},
  {132.917244, 88.211898, 123.829219, 282.064691, 77.571021, 293.694500, 85.365091},
  {143.813214, 67.649696, 117.591987, 280.587014, 80.038685, 292.994771, 86.523094},
  {160.996133, 40.724631, 107.735127, 279.333667, 83.004663, 292.668316, 87.269302},
  {182.867729, 9.023494, 93.466848, 278.647144, 86.597006, 292.621523, 87.443600},
  {194.313966, 12.223071, 76.499774, 278.983028, 90.468743, 292.596194, 87.200454},
  {179.133753, 6.491038, 61.262122, 280.280453, 93.808241, 292.556052, 87.076995},
  {149.735888, 13.576600, 50.099234, 281.975588, 96.277016, 292.715780, 87.296578},
  {133.410041, 20.387583, 40.837808, 283.840490, 98.185272, 292.995369, 87.593380},
  {153.855868, 9.832267, 27.409889, 286.382895, 99.918716, 292.827837, 87.490412},
  {160.176283, 25.812246, 12.141403, 289.396780, 101.004567, 292.341331, 87.317726},
  {142.906568, 18.021325, 358.940257, 292.242037, 101.275534, 291.983108, 87.277929},
  {112.524183, 2.937677, 349.960096, 294.481391, 101.051989, 292.040715, 87.266377},
  {84.796234, 21.134254, 344.747430, 296.135243, 100.676579, 292.452201, 87.169722},
  {63.577181, 32.781665, 342.226567, 297.353556, 100.312523, 293.080003, 86.980729},
  {47.413014, 39.353098, 341.566849, 298.254297, 100.016105, 293.827525, 86.734400},
  {34.648744, 42.517458, 342.210917, 298.912531, 99.799443, 294.635337, 86.468817},
  {24.338162, 43.226413, 343.756989, 299.377712, 99.656107, 295.461745, 86.214931},
  {16.076563, 41.886786, 345.869706, 299.687846, 99.571260, 296.270057, 85.994857},
  {9.792966, 38.954869, 348.256814, 299.878642, 99.527011, 297.029287, 85.820035},
  {5.345149, 34.972649, 350.682040, 299.983699, 99.507231, 297.716915, 85.691961},
  {2.422791, 30.265308, 352.961132, 300.031642, 99.500205, 298.316046, 85.605719},
  {0.684011, 25.174372, 354.965799, 300.045240, 99.498699, 298.816734, 85.552704},
  {0.209488, 20.037934, 356.623130, 300.041059, 99.498977, 299.216412, 85.523285},
  {0.553940, 15.152224, 357.908724, 300.029992, 99.499494, 299.519111, 85.508818},
  {0.581328, 10.749255, 358.836155, 300.018370, 99.499820, 299.734009, 85.502708},
  {0.456600, 6.994055, 359.446002, 300.009239, 99.499956, 299.873874, 85.500610},
  {0.287186, 3.994113, 359.796432, 300.003496, 99.499994, 299.953755, 85.500082},
  {0.136514, 1.812895, 359.955993, 300.000765, 99.500000, 299.990013, 85.500004},
  {0.036997, 0.482826, 359.998541, 300.000025, 99.500000, 299.999669, 85.500000},
  {0.001273, 0.016549, 360.000000, 300.000000, 99.500000, 300.000000, 85.500000},
  {0.000000, 0.000000, 360.000000, 300.000000, 99.500000, 300.000000, 85.500000}};
}}
