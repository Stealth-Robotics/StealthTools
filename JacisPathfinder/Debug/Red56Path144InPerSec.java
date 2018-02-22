/* Red56 Feed cross scale from side */
//----------------------------------------------------------------------------////  $Workfile: Red56Path144InPerSec.java$////  $Revision: X$////  Project:    Paths////                            Copyright (c) 2018//                               Cedarcrest High School//                            All Rights Reserved////  Modification History://  $Log://  $////----------------------------------------------------------------------------//////----------------------------------------------------------------------------//    Parameters Used//----------------------------------------------------------------------------//   Time Slice= 0.020000//   Max Vel   = 144.000000//   Max Accel = 600.000000//   Max Jerk  = 1000.000000package org.usfirst.frc4089.Stealth2018.MPPaths;

public class Red56Path144InPerSec extends Path {
    public Red56Path144InPerSec() {
       kNumPoints =217;
       kPoints = new double[][]{
  {0.400000, 0.400000, 359.997959, 16.002995, 251.250000, 16.002496, 237.250000},
  {0.872651, 0.727349, 359.986066, 16.020448, 251.249998, 16.017043, 237.249998},
  {1.963396, 1.636604, 359.959318, 16.059716, 251.249979, 16.049775, 237.249982},
  {3.490240, 2.909760, 359.911805, 16.129520, 251.249900, 16.107970, 237.249917},
  {5.452881, 4.547118, 359.837667, 16.238578, 251.249662, 16.198912, 237.249718},
  {7.850817, 6.549181, 359.731126, 16.395593, 251.249071, 16.329895, 237.249225},
  {10.683291, 8.916704, 359.586529, 16.609255, 251.247799, 16.508226, 237.248163},
  {13.949231, 11.650758, 359.398397, 16.888229, 251.245327, 16.741232, 237.246099},
  {17.647178, 14.752799, 359.161488, 17.241144, 251.240891, 17.036265, 237.242390},
  {21.775210, 18.224747, 358.870879, 17.676584, 251.233412, 17.400706, 237.236130},
  {26.330847, 22.069078, 358.522048, 18.203065, 251.221430, 17.841973, 237.226087},
  {31.310950, 26.288927, 358.110988, 18.829014, 251.203029, 18.367525, 237.210637},
  {36.494491, 30.705317, 357.637137, 19.558401, 251.175947, 18.981208, 237.187850},
  {41.660887, 35.138835, 357.103298, 20.390741, 251.137708, 19.683244, 237.155596},
  {46.807062, 39.592556, 356.512780, 21.325429, 251.085568, 20.473866, 237.111491},
  {51.929780, 44.069715, 355.869421, 22.361729, 251.016557, 21.353312, 237.052922},
  {57.025679, 48.573680, 355.177611, 23.498761, 250.927514, 22.321820, 236.977073},
  {62.091314, 53.107895, 354.442304, 24.735493, 250.815145, 23.379620, 236.880956},
  {67.123227, 57.675826, 353.669017, 26.070735, 250.676072, 24.526930, 236.761452},
  {72.118008, 62.280886, 352.863831, 27.503139, 250.506895, 25.763949, 236.615343},
  {77.072383, 66.926356, 352.033360, 29.031209, 250.304255, 27.090858, 236.439370},
  {81.983284, 71.615312, 351.184723, 30.653310, 250.064899, 28.507819, 236.230273},
  {86.847925, 76.350546, 350.325492, 32.367690, 249.785746, 30.014979, 235.984849},
  {91.663859, 81.134512, 349.463645, 34.172505, 249.463957, 31.612474, 235.700010},
  {96.429016, 85.969287, 348.607497, 36.065852, 249.096998, 33.300445, 235.372840},
  {101.141718, 90.856554, 347.765637, 38.045807, 248.682706, 35.079054, 235.000661},
  {105.800664, 95.797620, 346.946870, 40.110464, 248.219345, 36.948501, 234.581091},
  {110.404878, 100.793463, 346.160159, 42.257980, 247.705665, 38.909059, 234.112110},
  {114.953633, 105.844809, 345.414586, 44.486620, 247.140951, 40.961098, 233.592124},
  {119.446343, 110.952243, 344.719330, 46.794794, 246.525076, 43.105128, 233.020027},
  {123.676416, 115.922359, 344.084649, 49.177144, 245.859678, 45.338107, 232.396328},
  {127.441772, 120.557228, 343.521139, 51.624715, 245.148373, 47.653453, 231.723431},
  {130.749081, 124.850162, 343.038305, 54.129032, 244.395693, 50.044779, 231.004693},
  {133.604858, 128.794625, 342.644581, 56.682084, 243.607004, 52.505909, 230.244386},
  {136.015306, 132.384393, 342.347386, 59.276304, 242.788415, 55.030873, 229.447639},
  {137.986229, 135.613639, 342.153187, 61.904525, 241.946673, 57.613901, 228.620362},
  {139.523024, 138.476949, 342.067564, 64.559928, 241.089055, 60.249394, 227.769171},
  {140.630740, 140.969256, 342.095272, 67.235971, 240.223262, 62.931879, 226.901296},
  {141.314198, 143.085726, 342.240274, 69.926323, 239.357303, 65.655959, 226.024487},
  {141.578166, 144.821577, 342.505751, 72.624787, 238.499375, 68.416246, 225.146915},
  {141.427578, 146.171870, 342.894077, 75.325223, 237.657741, 71.207276, 224.277064},
  {140.867784, 147.131254, 343.406751, 78.021476, 236.840601, 74.023419, 223.423614},
  {140.098846, 147.899663, 344.045260, 80.711036, 236.054890, 76.862744, 222.594182},
  {139.316282, 148.681569, 344.811825, 83.394948, 235.306407, 79.727088, 221.795419},
  {138.519213, 149.477845, 345.708810, 86.074023, 234.601011, 82.618123, 221.034260},
  {137.707827, 150.288297, 346.738550, 88.748817, 233.944614, 85.537288, 220.317946},
  {136.883660, 151.111382, 347.903125, 91.419605, 233.343163, 88.485692, 219.654038},
  {136.049908, 151.943904, 349.204094, 94.086371, 232.802617, 91.464015, 219.050408},
  {135.211731, 152.780709, 350.642172, 96.748789, 232.328893, 94.472393, 218.515204},
  {134.376525, 153.614409, 352.216864, 99.406224, 231.927819, 97.510289, 218.056790},
  {133.554114, 154.435206, 353.926068, 102.057733, 231.605050, 100.576370, 217.683644},
  {132.756774, 155.230854, 355.765675, 104.702085, 231.365993, 103.668386, 217.404207},
  {131.999041, 155.986863, 357.729202, 107.337785, 231.215706, 106.783069, 217.226700},
  {131.297215, 156.686992, 359.807497, 109.963113, 231.158805, 109.916075, 217.158884},
  {130.668530, 157.314077, 1.988596, 112.576168, 231.199363, 113.061977, 217.207794},
  {130.130000, 157.851173, 4.257754, 115.174921, 231.340825, 116.214329, 217.379463},
  {129.697046, 158.282933, 6.597703, 117.757256, 231.585930, 119.365818, 217.678647},
  {129.382046, 158.597043, 8.989156, 120.321019, 231.936648, 122.508485, 218.108597},
  {129.193052, 158.785493, 11.411511, 122.864056, 232.394141, 125.634016, 218.670901},
  {129.132897, 158.845473, 13.843701, 125.384243, 232.958747, 128.734081, 219.365418},
  {129.198857, 158.779705, 16.265107, 127.879515, 233.629983, 131.800665, 220.190319},
  {129.382942, 158.596151, 18.656414, 130.347892, 234.406574, 134.826385, 221.142219},
  {129.672754, 158.307160, 21.000335, 132.787510, 235.286501, 137.804738, 222.216405},
  {130.052734, 157.928231, 23.282127, 135.196648, 236.267081, 140.730274, 223.407105},
  {135.229515, 152.660297, 24.708913, 137.661147, 237.381109, 143.513264, 224.662905},
  {152.062287, 135.931340, 23.388548, 140.438340, 238.620613, 145.995843, 225.770937},
  {152.310594, 135.682634, 22.027499, 143.248356, 239.796715, 148.499078, 226.818660},
  {152.557125, 135.435695, 20.626056, 146.090500, 240.906532, 151.022242, 227.803940},
  {152.799979, 135.192428, 19.184820, 148.963865, 241.947159, 153.564495, 228.724671},
  {153.037060, 134.954932, 17.704737, 151.867322, 242.915701, 156.124888, 229.578792},
  {153.266095, 134.725486, 16.187123, 154.799504, 243.809295, 158.702358, 230.364306},
  {153.484671, 134.506507, 14.633692, 157.758794, 244.625149, 161.295731, 231.079298},
  {153.690276, 134.300516, 13.046567, 160.743325, 245.360577, 163.903726, 231.721960},
  {153.880350, 134.110076, 11.428295, 163.750975, 246.013038, 166.524955, 232.290610},
  {154.052358, 133.937732, 9.781834, 166.779375, 246.580178, 169.157934, 232.783713},
  {154.203855, 133.785934, 8.110545, 169.825919, 247.059868, 171.801087, 233.199901},
  {154.332572, 133.656957, 6.418161, 172.887784, 247.450247, 174.452759, 233.537992},
  {154.436489, 133.552828, 4.708747, 175.961958, 247.749756, 177.111228, 233.797008},
  {154.513916, 133.475242, 2.986643, 179.045268, 247.957171, 179.774712, 233.976187},
  {154.563557, 133.425499, 1.256404, 182.134419, 248.071631, 182.441392, 234.074997},
  {154.584564, 133.404449, 359.522722, 185.226039, 248.092656, 185.109419, 234.093142},
  {154.576575, 133.412454, 357.790349, 188.316721, 248.020158, 187.776935, 234.030568},
  {154.539728, 133.449377, 356.064015, 191.403069, 247.854440, 190.442083, 233.887461},
  {154.474656, 133.514583, 354.348345, 194.481751, 247.596196, 193.103029, 233.664250},
  {154.382459, 133.606969, 352.647786, 197.549532, 247.246489, 195.757973, 233.361594},
  {154.264658, 133.725009, 350.966532, 200.603324, 246.806730, 198.405165, 232.980375},
  {154.123132, 133.866818, 349.308473, 203.640215, 246.278650, 201.042917, 232.521686},
  {153.960050, 134.030222, 347.677139, 206.657499, 245.664260, 203.669616, 231.986814},
  {153.777786, 134.212838, 346.075674, 209.652699, 244.965821, 206.283737, 231.377219},
  {153.578845, 134.412157, 344.506810, 212.623582, 244.185792, 208.883848, 230.694521},
  {153.365785, 134.625614, 342.972861, 215.568163, 243.326801, 211.468618, 229.940475},
  {153.141145, 134.850662, 341.475722, 218.484715, 242.391597, 214.036825, 229.116950},
  {152.907390, 135.084830, 340.016886, 221.371760, 241.383014, 216.587356, 228.225907},
  {152.666861, 135.325774, 338.597462, 224.228063, 240.303938, 219.119210, 227.269383},
  {152.421736, 135.571310, 337.218203, 227.052618, 239.157272, 221.631500, 226.249465},
  {152.174005, 135.819445, 335.879533, 229.844638, 237.945912, 224.123447, 225.168276},
  {149.978761, 137.689773, 334.873652, 232.569327, 236.691510, 226.624705, 224.016280},
  {146.447420, 141.551994, 334.472955, 235.216786, 235.438635, 229.183667, 222.805287},
  {146.559869, 141.439489, 334.053845, 237.857243, 234.165880, 231.731875, 221.577001},
  {146.680758, 141.318538, 333.614940, 240.490269, 232.872329, 234.268646, 220.330741},
  {146.810959, 141.188266, 333.154715, 243.115395, 231.556998, 236.793235, 219.065789},
  {146.951462, 141.047685, 332.671483, 245.732108, 230.218825, 239.304823, 217.781381},
  {147.103387, 140.895669, 332.163373, 248.339843, 228.856666, 241.802515, 216.476709},
  {147.268010, 140.730944, 331.628305, 250.937979, 227.469281, 244.285325, 215.150913},
  {147.446785, 140.552051, 331.063961, 253.525828, 226.055329, 246.752166, 213.803083},
  {147.641376, 140.357325, 330.467750, 256.102625, 224.613349, 249.201838, 212.432251},
  {147.853689, 140.144855, 329.836771, 258.667522, 223.141755, 251.633009, 211.037390},
  {148.085920, 139.912444, 329.167759, 261.219569, 221.638813, 254.044203, 209.617411},
  {148.340600, 139.657554, 328.457037, 263.757701, 220.102630, 256.433772, 208.171157},
  {148.620655, 139.377253, 327.700449, 266.280719, 218.531128, 258.799879, 206.697404},
  {148.929480, 139.068139, 326.893281, 268.787267, 216.922024, 261.140464, 205.194859},
  {149.271019, 138.726259, 326.030172, 271.275801, 215.272803, 263.453214, 203.662156},
  {149.649866, 138.347005, 325.105010, 273.744562, 213.580688, 265.735524, 202.097861},
  {150.071382, 137.925006, 324.110803, 276.191526, 211.842604, 267.984451, 200.500473},
  {150.541819, 137.453986, 323.039534, 278.614357, 210.055141, 270.196664, 198.868433},
  {151.068482, 136.926621, 321.881987, 281.010344, 208.214510, 272.368378, 197.200136},
  {151.659881, 136.334368, 320.627552, 283.376315, 206.316492, 274.495291, 195.493950},
  {152.325901, 135.667304, 319.263995, 285.708540, 204.356392, 276.572495, 193.748250},
  {153.077949, 134.913973, 317.777212, 288.002605, 202.328977, 278.594393, 191.961454},
  {153.929050, 134.061285, 316.150958, 290.253244, 200.228433, 280.554594, 190.132088},
  {154.893819, 133.094546, 314.366591, 292.454141, 198.048320, 282.445813, 188.258867},
  {155.988198, 131.997709, 312.402847, 294.597668, 195.781556, 284.259762, 186.340809},
  {157.228784, 130.754053, 310.235731, 296.674568, 193.420446, 285.987062, 184.377372},
  {158.631443, 129.347559, 307.838638, 298.673562, 190.956791, 287.617181, 182.368634},
  {160.208823, 127.765401, 305.182870, 300.580883, 188.382131, 289.138442, 180.315499},
  {161.966235, 126.002089, 302.238840, 302.379770, 185.688206, 290.538126, 178.219909},
  {163.895363, 124.065781, 298.978304, 304.049970, 182.867726, 291.802725, 176.085029},
  {165.965597, 121.987023, 295.378016, 305.567395, 179.915565, 292.918397, 173.915326},
  {168.113751, 119.829126, 291.425081, 306.904160, 176.830447, 293.871616, 171.716467},
  {170.234931, 117.697426, 287.123817, 308.029369, 173.617057, 294.649979, 169.494931},
  {172.180094, 115.741826, 282.503030, 308.911001, 170.288226, 295.243017, 167.257348},
  {173.767770, 114.145071, 277.621369, 309.519119, 166.866488, 295.642793, 165.009723},
  {174.814921, 113.091656, 272.567613, 309.830083, 163.384046, 295.844139, 162.756870},
  {175.182443, 112.721884, 267.453449, 309.830728, 159.880397, 295.844554, 160.502432},
  {174.817905, 113.088655, 262.399202, 309.521026, 156.397783, 295.644034, 158.249565},
  {173.773320, 114.139489, 257.516629, 308.914091, 152.975722, 295.245068, 156.001910},
  {172.187511, 115.734368, 252.894623, 308.033530, 149.646454, 294.652814, 153.764274},
  {170.243435, 117.688877, 248.591963, 306.909260, 146.432555, 293.875195, 151.542658},
  {168.122647, 119.820187, 244.637567, 305.573302, 143.346893, 292.922674, 149.343692},
  {165.974358, 121.978224, 241.035841, 304.056564, 140.394183, 291.807645, 147.173856},
  {163.903647, 124.057465, 237.773946, 302.386942, 137.573168, 290.543631, 145.038822},
  {161.973855, 125.994442, 234.828666, 300.588542, 134.878735, 289.144477, 142.943063},
  {160.215705, 127.758497, 232.171769, 298.681631, 132.303600, 287.623690, 140.889748},
  {158.637586, 129.341398, 229.773669, 296.682986, 129.839504, 285.993996, 138.880824},
  {157.234229, 130.748594, 227.605660, 294.606382, 127.477988, 284.267075, 136.917200},
  {155.993006, 131.992890, 225.641128, 292.463110, 125.210851, 282.453463, 134.998955},
  {154.898057, 133.090298, 223.856066, 290.262433, 123.030395, 280.562546, 133.125551},
  {153.932787, 134.057541, 222.229201, 288.011986, 120.929536, 278.602613, 131.296007},
  {153.081248, 134.910667, 220.741877, 285.718089, 118.901832, 276.580956, 129.509038},
  {152.328819, 135.664382, 219.377842, 283.386011, 116.941465, 274.503968, 127.763171},
  {151.662468, 136.331776, 218.122982, 281.020171, 115.043202, 272.377250, 126.056826},
  {151.070783, 136.924317, 216.965058, 278.624301, 113.202343, 270.205711, 124.388376},
  {150.543871, 137.451932, 215.893453, 276.201575, 111.414668, 267.993658, 122.756189},
  {150.073218, 137.923168, 214.898945, 273.754706, 109.676388, 265.744875, 121.158662},
  {149.651514, 138.345356, 213.973514, 271.286031, 107.984089, 263.462696, 119.594233},
  {149.272502, 138.724774, 213.110162, 268.797574, 106.334697, 261.150066, 118.061403},
  {148.930819, 139.066799, 212.302775, 266.291097, 104.725433, 258.809592, 116.558736},
  {148.621868, 139.376039, 211.545988, 263.768144, 103.153780, 256.443586, 115.084868},
  {148.341701, 139.656452, 210.835086, 261.230072, 101.617456, 254.054109, 113.638503},
  {148.086923, 139.911440, 210.165910, 258.678080, 100.114382, 251.643001, 112.218417},
  {147.854605, 140.143939, 209.534780, 256.113234, 98.642662, 249.211909, 110.823454},
  {147.642214, 140.356486, 208.938432, 253.536484, 97.200563, 246.762311, 109.452525},
  {147.447554, 140.551281, 208.373962, 250.948680, 95.786498, 244.295538, 108.104602},
  {147.268718, 140.730236, 207.838778, 248.350585, 94.399006, 241.812792, 106.778718},
  {147.104039, 140.895016, 207.330561, 245.742887, 93.036746, 239.315159, 105.473960},
  {146.952065, 141.047082, 206.847230, 243.126210, 91.698477, 236.803626, 104.189471},
  {146.811517, 141.187708, 206.386914, 240.501118, 90.383054, 234.279089, 102.924440},
  {146.681276, 141.318021, 205.947925, 237.868124, 89.089415, 231.742367, 101.678104},
  {146.560350, 141.439007, 205.528736, 235.227697, 87.816576, 229.194205, 100.449745},
  {146.447867, 141.551546, 205.127966, 232.580265, 86.563620, 226.635286, 99.238683},
  {700.592078, 702.590037, 22.827636, 227.800539, 99.735025, 233.231981, 86.831559},
  {163.955397, 124.005488, 19.557248, 230.857856, 100.920504, 235.544336, 87.728199},
  {163.911754, 124.049301, 16.294020, 233.976999, 101.929351, 237.904931, 88.491666},
  {163.537803, 124.424702, 13.092153, 237.140890, 102.758591, 240.312141, 89.122493},
  {162.823631, 125.109305, 10.004820, 240.331554, 103.409956, 242.763788, 89.622852},
  {161.578018, 125.894205, 7.083743, 243.527362, 103.889327, 245.253841, 89.996189},
  {159.654518, 126.557760, 4.374484, 246.704592, 104.207188, 247.772442, 90.247972},
  {157.129093, 127.023533, 1.910121, 249.842502, 104.378478, 250.309146, 90.386257},
  {154.079046, 127.213803, 359.711036, 252.923789, 104.421065, 252.853182, 90.421243},
  {150.573961, 127.058683, 357.786193, 255.934528, 104.354298, 255.393727, 90.364747},
  {146.670740, 126.501093, 356.135228, 258.863765, 104.197807, 257.920139, 90.229644},
  {142.412349, 125.498000, 354.750736, 261.702936, 103.970610, 260.422093, 90.029325},
  {137.829078, 124.019125, 353.620357, 264.445251, 103.690519, 262.889629, 89.777215},
  {132.941039, 122.044420, 352.728448, 267.085137, 103.373772, 265.313128, 89.486368},
  {127.760951, 119.561251, 352.057290, 269.617778, 103.034836, 267.683218, 89.169142},
  {122.296657, 116.561874, 351.587890, 272.038755, 102.686330, 269.990666, 88.836950},
  {116.569390, 113.057360, 351.300426, 274.344118, 102.338954, 272.226570, 88.500024},
  {110.782680, 109.244266, 351.174505, 276.533840, 102.000960, 274.385882, 88.166717},
  {105.115761, 105.311229, 351.190504, 278.611247, 101.678298, 276.467152, 87.843456},
  {99.560048, 101.266893, 351.330212, 280.579274, 101.375394, 278.468920, 87.535365},
  {94.108269, 97.118575, 351.576609, 282.440492, 101.095374, 280.389676, 87.246398},
  {88.754321, 92.872408, 351.913679, 284.197149, 100.840250, 282.227841, 86.979448},
  {83.493044, 88.533576, 352.326253, 285.851211, 100.611087, 283.981762, 86.736464},
  {78.319964, 84.106565, 352.799894, 287.404409, 100.408148, 285.649718, 86.518546},
  {73.231045, 79.595420, 353.320827, 288.858280, 100.231026, 287.229944, 86.326044},
  {68.222459, 75.003973, 353.875904, 290.214206, 100.078759, 288.720655, 86.158655},
  {63.290394, 70.336033, 354.452600, 291.473443, 99.949945, 290.120074, 86.015513},
  {58.430911, 65.595539, 355.039035, 292.637143, 99.842841, 291.426464, 85.895287},
  {53.639837, 60.786657, 355.624013, 293.706374, 99.755456, 292.638158, 85.796268},
  {48.912719, 55.913836, 356.197064, 294.682134, 99.685638, 293.753584, 85.716465},
  {44.244804, 50.981820, 356.748499, 295.565351, 99.631157, 294.771286, 85.653694},
  {39.631070, 45.995626, 357.269447, 296.356892, 99.589768, 295.689944, 85.605664},
  {35.066262, 40.960505, 357.751898, 297.057554, 99.559287, 296.508380, 85.570062},
  {30.544956, 35.881875, 358.188732, 297.668069, 99.537637, 297.225567, 85.544632},
  {26.076373, 30.782720, 358.573953, 298.189389, 99.522896, 297.840975, 85.527233},
  {21.837665, 25.885886, 358.905304, 298.626036, 99.513285, 298.358568, 85.515840},
  {17.989835, 21.398161, 359.184280, 298.985783, 99.507284, 298.786471, 85.508703},
  {14.525942, 17.326490, 359.413508, 299.276280, 99.503727, 299.132975, 85.504461},
  {11.440336, 13.676523, 359.596543, 299.505078, 99.501750, 299.406496, 85.502097},
  {8.728527, 10.452755, 359.737673, 299.679646, 99.500735, 299.615547, 85.500882},
  {6.387038, 7.658664, 359.841757, 299.807385, 99.500266, 299.768719, 85.500320},
  {4.413277, 5.296844, 359.914078, 299.895651, 99.500078, 299.874656, 85.500094},
  {2.805398, 3.369141, 359.960221, 299.951759, 99.500017, 299.942039, 85.500020},
  {1.562169, 1.876788, 359.985972, 299.983002, 99.500002, 299.979575, 85.500003},
  {0.682846, 0.820528, 359.997242, 299.996659, 99.500000, 299.995985, 85.500000},
  {0.167047, 0.200744, 360.000000, 300.000000, 99.500000, 300.000000, 85.500000},
  {0.000000, 0.000000, 360.000000, 300.000000, 99.500000, 300.000000, 85.500000}};
}}