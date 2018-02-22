/* Red12 Feed Switch from Front Start 1 aud */
//----------------------------------------------------------------------------
//
//  $Workfile: Red12Path60InPerSec.java$
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
//   Max Jerk  = 1000.000000
package org.usfirst.frc4089.Stealth2018.MPPaths;

public class Red12Path60InPerSec extends Path {
    public Red12Path60InPerSec() {
       kNumPoints =272;
       kPoints = new double[][]{
  {0.400000, 0.400000, 0.001337, 16.001698, 314.750000, 16.002024, 300.750000},
  {0.729828, 0.870172, 0.012824, 16.016294, 314.750002, 16.019428, 300.750002},
  {1.642288, 1.957712, 0.038642, 16.049140, 314.750017, 16.058582, 300.750020},
  {2.920222, 3.479778, 0.084442, 16.107544, 314.750079, 16.128177, 300.750095},
  {4.564375, 5.435624, 0.155755, 16.198832, 314.750271, 16.236890, 300.750322},
  {6.393244, 7.606754, 0.255082, 16.326696, 314.750729, 16.389024, 300.750868},
  {8.225196, 9.774800, 0.381919, 16.491197, 314.751644, 16.584517, 300.751955},
  {10.061064, 11.938929, 0.535624, 16.692412, 314.753256, 16.823288, 300.753867},
  {11.901643, 14.098346, 0.715427, 16.930431, 314.755855, 17.105238, 300.756946},
  {13.747686, 16.252298, 0.920432, 17.205356, 314.759781, 17.430251, 300.761588},
  {15.599896, 18.400082, 1.149631, 17.517303, 314.765419, 17.798192, 300.768237},
  {17.458919, 20.541050, 1.401907, 17.866395, 314.773195, 18.208911, 300.777386},
  {19.325342, 22.674618, 1.676049, 18.252762, 314.783579, 18.662240, 300.789568},
  {21.199685, 24.800264, 1.970761, 18.676541, 314.797073, 19.157994, 300.805354},
  {23.082402, 26.917536, 2.284671, 19.137871, 314.814218, 19.695973, 300.825347},
  {24.973872, 29.026053, 2.616347, 19.636891, 314.835580, 20.275963, 300.850174},
  {26.874404, 31.125507, 2.964305, 20.173742, 314.861752, 20.897735, 300.880485},
  {28.784234, 33.215663, 3.327023, 20.748559, 314.893351, 21.561047, 300.916948},
  {30.703525, 35.296356, 3.702952, 21.361473, 314.931010, 22.265645, 300.960238},
  {32.632371, 37.367495, 4.090528, 22.012611, 314.975374, 23.011267, 301.011038},
  {34.570797, 39.429054, 4.488183, 22.702089, 315.027101, 23.797638, 301.070032},
  {36.518765, 41.481072, 4.894354, 23.430017, 315.086850, 24.624479, 301.137898},
  {38.476176, 43.523646, 5.307496, 24.196491, 315.155286, 25.491503, 301.215309},
  {40.442878, 45.556931, 5.726088, 25.001600, 315.233065, 26.398420, 301.302922},
  {42.418670, 47.581126, 6.148642, 25.845421, 315.320842, 27.344935, 301.401379},
  {44.403309, 49.596476, 6.573709, 26.728017, 315.419258, 28.330755, 301.511303},
  {46.396514, 51.603261, 6.999888, 27.649442, 315.528941, 29.355585, 301.633291},
  {48.397975, 53.601790, 7.425827, 28.609738, 315.650500, 30.419134, 301.767918},
  {50.407360, 55.592397, 7.850229, 29.608937, 315.784525, 31.521114, 301.915726},
  {52.424321, 57.575431, 8.271854, 30.647059, 315.931581, 32.661241, 302.077229},
  {54.257246, 59.342504, 8.688089, 31.720336, 316.091632, 33.835111, 302.252278},
  {55.712506, 60.687249, 9.095278, 32.821191, 316.263890, 35.034264, 302.439914},
  {56.786632, 61.613133, 9.490334, 33.942013, 316.447320, 36.250349, 302.638932},
  {57.476381, 62.123400, 9.870698, 35.075166, 316.640650, 37.475120, 302.847890},
  {57.778771, 62.221028, 10.234302, 36.212996, 316.842386, 38.700431, 303.065134},
  {57.884230, 62.115587, 10.580644, 37.351629, 317.051548, 39.922299, 303.289584},
  {57.985121, 62.014713, 10.910471, 38.490991, 317.267800, 41.140839, 303.520862},
  {58.081746, 61.918104, 11.224482, 39.631016, 317.490819, 42.356165, 303.758610},
  {58.174387, 61.825477, 11.523328, 40.771649, 317.720297, 43.568386, 304.002489},
  {58.263309, 61.736568, 11.807618, 41.912842, 317.955935, 44.777608, 304.252172},
  {58.348758, 61.651131, 12.077921, 43.054552, 318.197446, 45.983937, 304.507351},
  {58.430964, 61.568935, 12.334768, 44.196747, 318.444553, 47.187473, 304.767727},
  {58.510144, 61.489765, 12.578653, 45.339398, 318.696990, 48.388313, 305.033018},
  {58.586500, 61.413419, 12.810040, 46.482482, 318.954498, 49.586553, 305.302951},
  {58.660220, 61.339707, 13.029359, 47.625981, 319.216830, 50.782285, 305.577264},
  {58.731483, 61.268451, 13.237013, 48.769883, 319.483743, 51.975600, 305.855707},
  {58.800456, 61.199485, 13.433376, 49.914180, 319.755006, 53.166583, 306.138036},
  {58.867296, 61.132651, 13.618798, 51.058866, 320.030391, 54.355320, 306.424018},
  {58.932151, 61.067802, 13.793604, 52.203941, 320.309680, 55.541891, 306.713428},
  {58.995161, 61.004798, 13.958095, 53.349407, 320.592661, 56.726378, 307.006047},
  {59.056457, 60.943506, 14.112552, 54.495271, 320.879125, 57.908856, 307.301664},
  {59.116165, 60.883803, 14.257235, 55.641541, 321.168872, 59.089401, 307.600074},
  {59.174404, 60.825568, 14.392385, 56.788229, 321.461704, 60.268085, 307.901078},
  {59.231285, 60.768691, 14.518223, 57.935349, 321.757432, 61.444980, 308.204480},
  {59.286916, 60.713063, 14.634955, 59.082917, 322.055866, 62.620152, 308.510093},
  {59.341399, 60.658583, 14.742768, 60.230952, 322.356825, 63.793670, 308.817732},
  {59.394833, 60.605152, 14.841834, 61.379475, 322.660129, 64.965597, 309.127216},
  {59.447310, 60.552678, 14.932309, 62.528509, 322.965601, 66.135997, 309.438368},
  {59.498920, 60.501070, 15.014336, 63.678080, 323.273069, 67.304930, 309.751014},
  {59.549751, 60.450240, 15.088042, 64.828213, 323.582363, 68.472455, 310.064985},
  {59.599887, 60.400106, 15.153541, 65.978938, 323.893315, 69.638630, 310.380112},
  {59.649408, 60.350587, 15.210934, 67.130284, 324.205760, 70.803511, 310.696230},
  {59.698394, 60.301602, 15.260307, 68.282284, 324.519535, 71.967151, 311.013175},
  {59.746921, 60.253076, 15.301736, 69.434972, 324.834479, 73.129604, 311.330787},
  {59.795065, 60.204933, 15.335284, 70.588382, 325.150431, 74.290920, 311.648904},
  {59.842901, 60.157098, 15.361002, 71.742551, 325.467232, 75.451148, 311.967369},
  {59.890499, 60.109500, 15.378927, 72.897517, 325.784726, 76.610338, 312.286024},
  {59.937933, 60.062067, 15.389088, 74.053320, 326.102754, 77.768535, 312.604710},
  {59.985274, 60.014726, 15.391498, 75.210001, 326.421160, 78.925783, 312.923273},
  {60.032593, 59.967407, 15.386163, 76.367602, 326.739788, 80.082128, 313.241555},
  {60.079961, 59.920039, 15.373073, 77.526168, 327.058481, 81.237610, 313.559399},
  {60.127448, 59.872551, 15.352209, 78.685744, 327.377082, 82.392271, 313.876650},
  {60.175127, 59.824872, 15.323541, 79.846378, 327.695435, 83.546149, 314.193150},
  {60.223069, 59.776929, 15.287023, 81.008117, 328.013380, 84.699281, 314.508740},
  {60.271348, 59.728649, 15.242603, 82.171012, 328.330759, 85.851705, 314.823261},
  {60.320039, 59.679957, 15.190212, 83.335115, 328.647411, 87.003455, 315.136553},
  {60.369216, 59.630778, 15.129770, 84.500477, 328.963174, 88.154563, 315.448454},
  {60.418958, 59.581035, 15.061185, 85.667156, 329.277884, 89.305061, 315.758799},
  {60.469345, 59.530646, 14.984351, 86.835205, 329.591374, 90.454978, 316.067424},
  {60.520458, 59.479531, 14.899150, 88.004685, 329.903477, 91.604343, 316.374158},
  {60.572383, 59.427603, 14.805448, 89.175654, 330.214019, 92.753182, 316.678832},
  {60.625208, 59.374776, 14.703099, 90.348174, 330.522827, 93.901518, 316.981271},
  {60.679024, 59.320957, 14.591940, 91.522309, 330.829723, 95.049374, 317.281298},
  {60.733925, 59.266053, 14.471793, 92.698124, 331.134523, 96.196771, 317.578732},
  {60.790013, 59.209962, 14.342464, 93.875686, 331.437043, 97.343726, 317.873389},
  {60.847390, 59.152581, 14.203742, 95.055065, 331.737090, 98.490254, 318.165080},
  {60.906165, 59.093801, 14.055398, 96.236331, 332.034469, 99.636370, 318.453610},
  {60.966454, 59.033508, 13.897184, 97.419558, 332.328979, 100.782083, 318.738783},
  {61.028377, 58.971579, 13.728833, 98.604823, 332.620412, 101.927401, 319.020394},
  {61.092062, 58.907889, 13.550056, 99.792203, 332.908554, 103.072329, 319.298235},
  {61.157644, 58.842301, 13.360542, 100.981778, 333.193184, 104.216869, 319.572090},
  {61.225265, 58.774674, 13.159958, 102.173631, 333.474073, 105.361017, 319.841737},
  {61.295076, 58.704855, 12.947946, 103.367848, 333.750985, 106.504768, 320.106949},
  {61.367239, 58.632685, 12.724119, 104.564516, 334.023675, 107.648112, 320.367488},
  {61.441924, 58.557991, 12.488066, 105.763727, 334.291886, 108.791035, 320.623111},
  {61.519314, 58.480592, 12.239343, 106.965574, 334.555354, 109.933516, 320.873567},
  {61.599602, 58.400294, 11.977476, 108.170152, 334.813802, 111.075532, 321.118593},
  {61.682995, 58.316889, 11.701956, 109.377561, 335.066942, 112.217052, 321.357919},
  {61.769715, 58.230157, 11.412239, 110.587904, 335.314471, 113.358038, 321.591266},
  {61.859998, 58.139861, 11.107742, 111.801284, 335.556074, 114.498448, 321.818341},
  {61.954097, 58.045748, 10.787838, 113.017810, 335.791420, 115.638229, 322.038842},
  {62.052282, 57.947546, 10.451861, 114.237593, 336.020163, 116.777324, 322.252456},
  {62.154843, 57.844967, 10.099092, 115.460747, 336.241938, 117.915663, 322.458854},
  {62.262091, 57.737700, 9.728765, 116.687389, 336.456360, 119.053168, 322.657697},
  {62.374358, 57.625412, 9.340058, 117.917639, 336.663026, 120.189751, 322.848631},
  {62.492001, 57.507746, 8.932090, 119.151618, 336.861509, 121.325309, 323.031286},
  {62.615400, 57.384321, 8.503919, 120.389451, 337.051359, 122.459730, 323.205279},
  {62.744965, 57.254727, 8.054536, 121.631267, 337.232099, 123.592885, 323.370207},
  {62.881133, 57.118528, 7.582859, 122.877192, 337.403224, 124.724630, 323.525653},
  {63.024371, 56.975255, 7.087731, 124.127359, 337.564199, 125.854804, 323.671182},
  {63.175178, 56.824410, 6.567911, 125.381896, 337.714456, 126.983227, 323.806338},
  {63.334086, 56.665461, 6.022075, 126.640935, 337.853391, 128.109698, 323.930649},
  {63.501659, 56.497841, 5.448802, 127.904606, 337.980361, 129.233993, 324.043621},
  {63.678497, 56.320951, 4.846576, 129.173034, 338.094681, 130.355865, 324.144738},
  {63.865232, 56.134158, 4.213776, 130.446344, 338.195623, 131.475036, 324.233467},
  {64.062532, 55.936794, 3.548672, 131.724652, 338.282406, 132.591202, 324.309250},
  {64.271092, 55.728163, 2.849419, 133.008068, 338.354200, 133.704025, 324.371510},
  {64.491636, 55.507541, 2.114056, 134.296688, 338.410116, 134.813132, 324.419645},
  {64.724907, 55.274182, 1.340498, 135.590596, 338.449204, 135.918111, 324.453035},
  {64.971662, 55.027328, 0.526536, 136.889855, 338.470447, 137.018511, 324.471038},
  {65.232662, 54.766220, 359.669838, 138.194507, 338.472759, 138.113833, 324.472992},
  {65.508649, 54.490112, 358.767949, 139.504559, 338.454979, 139.203535, 324.458216},
  {65.800336, 54.198291, 357.818298, 140.819984, 338.415866, 140.287023, 324.426014},
  {66.108372, 53.890105, 356.818207, 142.140708, 338.354095, 141.363649, 324.375676},
  {66.433315, 53.564996, 355.764906, 143.466598, 338.268252, 142.432711, 324.306480},
  {66.775587, 53.222538, 354.655556, 144.797454, 338.156837, 143.493453, 324.217699},
  {67.135427, 52.862494, 353.487280, 146.132992, 338.018255, 144.545059, 324.108602},
  {67.512824, 52.484872, 352.257202, 147.472828, 337.850822, 145.586658, 323.978462},
  {67.907448, 52.089999, 350.962499, 148.816458, 337.652765, 146.617325, 323.826564},
  {68.318568, 51.678606, 349.600467, 150.163240, 337.422228, 147.636085, 323.652207},
  {68.744955, 51.251923, 348.168606, 151.512371, 337.157287, 148.641918, 323.454715},
  {69.184780, 50.811775, 346.664712, 152.862858, 336.855961, 149.633771, 323.233443},
  {69.635515, 50.360694, 345.086996, 154.213498, 336.516232, 150.610568, 322.987785},
  {70.093827, 49.902012, 343.434216, 155.562847, 336.136083, 151.571223, 322.717181},
  {70.555499, 49.439951, 341.705820, 156.909204, 335.713525, 152.514660, 322.421122},
  {71.015369, 48.979676, 339.902099, 158.250584, 335.246649, 153.439830, 322.099154},
  {71.467321, 48.527309, 338.024350, 159.584710, 334.733680, 154.345735, 321.750878},
  {71.904335, 48.089877, 336.075016, 160.909011, 334.173035, 155.231448, 321.375954},
  {72.318616, 47.675185, 334.057819, 162.220625, 333.563396, 156.096130, 320.974092},
  {72.701817, 47.291593, 331.977850, 163.516434, 332.903773, 156.939053, 320.545049},
  {73.045349, 46.947699, 329.841604, 164.793099, 332.193579, 157.759607, 320.088622},
  {73.340793, 46.651937, 327.656960, 166.047127, 331.432685, 158.557307, 319.604642},
  {73.580354, 46.412112, 325.433069, 167.274950, 330.621465, 159.331791, 319.092969},
  {73.757359, 46.234909, 323.180181, 168.473017, 329.760827, 160.082809, 318.553490},
  {73.866708, 46.125437, 320.909378, 169.637895, 328.852216, 160.810213, 317.986122},
  {73.905252, 46.086849, 318.632261, 170.766381, 327.897588, 161.513929, 317.390822},
  {73.872036, 46.120103, 316.360586, 171.855589, 326.899361, 162.193943, 316.767599},
  {73.768369, 46.223886, 314.105894, 172.903041, 325.860346, 162.850275, 316.116533},
  {73.597724, 46.394723, 311.879157, 173.906729, 324.783656, 163.482967, 315.437792},
  {73.365463, 46.627240, 309.690471, 174.865148, 323.672608, 164.092067, 314.731650},
  {73.078449, 46.914564, 307.548803, 175.777313, 322.530618, 164.677630, 313.998501},
  {72.744576, 47.248790, 305.461829, 176.642743, 321.361114, 165.239712, 313.238867},
  {72.372284, 47.621464, 303.435842, 177.461428, 320.167441, 165.778381, 312.453401},
  {71.970104, 48.024044, 301.475734, 178.233781, 318.952802, 166.293722, 311.642878},
  {71.546273, 48.448282, 299.585052, 178.960578, 317.720198, 166.785845, 310.808187},
  {71.108432, 48.886528, 297.766088, 179.642894, 316.472396, 167.254898, 309.950314},
  {70.663427, 49.331929, 296.020014, 180.282039, 315.211911, 167.701067, 309.070320},
  {70.217193, 49.778544, 294.347028, 180.879499, 313.940997, 168.124586, 308.169325},
  {69.774713, 50.221385, 292.746515, 181.436880, 312.661649, 168.525737, 307.248481},
  {69.340045, 50.656392, 291.217191, 181.955864, 311.375620, 168.904850, 306.308959},
  {68.916378, 51.080376, 289.757255, 182.438165, 310.084429, 169.262299, 305.351927},
  {68.506120, 51.490926, 288.364508, 182.885500, 308.789390, 169.598501, 304.378533},
  {68.111001, 51.886312, 287.036470, 183.299566, 307.491625, 169.913907, 303.389901},
  {67.732177, 52.265382, 285.770470, 183.682015, 306.192090, 170.209000, 302.387110},
  {67.370321, 52.627460, 284.563728, 184.034446, 304.891592, 170.484286, 301.371198},
  {67.025727, 52.972257, 283.413416, 184.358393, 303.590808, 170.740291, 300.343149},
  {66.698380, 53.299788, 282.316710, 184.655320, 302.290307, 170.977552, 299.303892},
  {66.388031, 53.610303, 281.270824, 184.926616, 300.990558, 171.196616, 298.254303},
  {66.094256, 53.904229, 280.273044, 185.173596, 299.691951, 171.398030, 297.195200},
  {65.816503, 54.182115, 279.320746, 185.397502, 298.394804, 171.582342, 296.127348},
  {65.554135, 54.444605, 278.411409, 185.599501, 297.099375, 171.750096, 295.051455},
  {65.306458, 54.692393, 277.542628, 185.780692, 295.805875, 171.901828, 293.968182},
  {65.072744, 54.926205, 276.712115, 185.942107, 294.514469, 172.038063, 292.878138},
  {64.852260, 55.146780, 275.917704, 186.084712, 293.225287, 172.159317, 291.781888},
  {64.644268, 55.354850, 275.157349, 186.209415, 291.938429, 172.266092, 290.679952},
  {64.448053, 55.551141, 274.429122, 186.317066, 290.653972, 172.358875, 289.572811},
  {64.262910, 55.736348, 273.731209, 186.408464, 289.371967, 172.438139, 288.460905},
  {64.088169, 55.911148, 273.061907, 186.484355, 288.092453, 172.504342, 287.344644},
  {63.923190, 56.076182, 272.419617, 186.545442, 286.815449, 172.557924, 286.224401},
  {63.767359, 56.232061, 271.802842, 186.592382, 285.540966, 172.599312, 285.100521},
  {63.620103, 56.379362, 271.210177, 186.625793, 284.269003, 172.628916, 283.973323},
  {63.480876, 56.518629, 270.640306, 186.646256, 282.999550, 172.647130, 282.843097},
  {63.349169, 56.650373, 270.092000, 186.654316, 281.732592, 172.654334, 281.710112},
  {63.224504, 56.775073, 269.564105, 186.650487, 280.468108, 172.650892, 280.574616},
  {63.106430, 56.893175, 269.055542, 186.635251, 279.206071, 172.637153, 279.436836},
  {62.994533, 57.005100, 268.565299, 186.609064, 277.946453, 172.613452, 278.296980},
  {62.888422, 57.111239, 268.092429, 186.572354, 276.689220, 172.580113, 277.155242},
  {62.787729, 57.211953, 267.636044, 186.525527, 275.434339, 172.537442, 276.011799},
  {62.692121, 57.307583, 267.195312, 186.468965, 274.181773, 172.485735, 274.866814},
  {62.601279, 57.398445, 266.769454, 186.403030, 272.931485, 172.425278, 273.720438},
  {62.514909, 57.484833, 266.357735, 186.328062, 271.683436, 172.356340, 272.572810},
  {62.432737, 57.567020, 265.959470, 186.244386, 270.437588, 172.279183, 271.424058},
  {62.354511, 57.645264, 265.574012, 186.152307, 269.193902, 172.194058, 270.274300},
  {62.279989, 57.719798, 265.200755, 186.052118, 267.952338, 172.101202, 269.123644},
  {62.208954, 57.790846, 264.839127, 185.944092, 266.712858, 172.000847, 267.972192},
  {62.141200, 57.858614, 264.488593, 185.828492, 265.475422, 171.893213, 266.820037},
  {62.076531, 57.923292, 264.148645, 185.705567, 264.239991, 171.778511, 265.667263},
  {62.014774, 57.985062, 263.818808, 185.575553, 263.006529, 171.656944, 264.513951},
  {61.955756, 58.044087, 263.498633, 185.438676, 261.774997, 171.528708, 263.360174},
  {61.899327, 58.100525, 263.187696, 185.295150, 260.545359, 171.393989, 262.205999},
  {61.845339, 58.154522, 262.885599, 185.145179, 259.317577, 171.252968, 261.051490},
  {61.793658, 58.206212, 262.591962, 184.988959, 258.091617, 171.105816, 259.896703},
  {61.744154, 58.255722, 262.306430, 184.826675, 256.867444, 170.952700, 258.741694},
  {61.696711, 58.303171, 262.028664, 184.658506, 255.645023, 170.793780, 257.586511},
  {61.651219, 58.348670, 261.758347, 184.484620, 254.424321, 170.629208, 256.431200},
  {61.607573, 58.392323, 261.495175, 184.305181, 253.205306, 170.459133, 255.275804},
  {61.565675, 58.434225, 261.238862, 184.120344, 251.987945, 170.283697, 254.120362},
  {61.525435, 58.474470, 260.989137, 183.930257, 250.772207, 170.103036, 252.964911},
  {61.486767, 58.513142, 260.745742, 183.735064, 249.558062, 169.917282, 251.809484},
  {61.449593, 58.550322, 260.508433, 183.534900, 248.345479, 169.726562, 250.654114},
  {61.413835, 58.586083, 260.276979, 183.329898, 247.134431, 169.530998, 249.498827},
  {61.379424, 58.620498, 260.051157, 183.120183, 245.924889, 169.330709, 248.343652},
  {61.346294, 58.653634, 259.830760, 182.905875, 244.716825, 169.125808, 247.188613},
  {61.314380, 58.685549, 259.615587, 182.687092, 243.510212, 168.916404, 246.033733},
  {61.283627, 58.716306, 259.405449, 182.463943, 242.305024, 168.702603, 244.879034},
  {61.253976, 58.745958, 259.200165, 182.236538, 241.101235, 168.484509, 243.724534},
  {61.225379, 58.774560, 258.999562, 182.004978, 239.898821, 168.262218, 242.570252},
  {61.197782, 58.802158, 258.803478, 181.769364, 238.697758, 168.035827, 241.416205},
  {61.171144, 58.828801, 258.611754, 181.529791, 237.498021, 167.805427, 240.262408},
  {61.145417, 58.854530, 258.424243, 181.286352, 236.299588, 167.571108, 239.108876},
  {61.120562, 58.879388, 258.240800, 181.039135, 235.102436, 167.332956, 237.955621},
  {61.096538, 58.903412, 258.061290, 180.788226, 233.906543, 167.091054, 236.802656},
  {61.073310, 58.926642, 257.885583, 180.533709, 232.711888, 166.845483, 235.649992},
  {61.050844, 58.949112, 257.713554, 180.275663, 231.518450, 166.596320, 234.497640},
  {61.029103, 58.970853, 257.545084, 180.014165, 230.326209, 166.343641, 233.345607},
  {61.008061, 58.991899, 257.380058, 179.749290, 229.135144, 166.087519, 232.193905},
  {60.987684, 59.012276, 257.218369, 179.481110, 227.945237, 165.828025, 231.042539},
  {60.967946, 59.032016, 257.059911, 179.209694, 226.756469, 165.565228, 229.891518},
  {60.948819, 59.051143, 256.904584, 178.935111, 225.568821, 165.299193, 228.740849},
  {60.930281, 59.069684, 256.752292, 178.657425, 224.382276, 165.029987, 227.590536},
  {60.912305, 59.087661, 256.602943, 178.376700, 223.196815, 164.757671, 226.440586},
  {60.894869, 59.105099, 256.456448, 178.092996, 222.012422, 164.482306, 225.291004},
  {60.877951, 59.122017, 256.312723, 177.806374, 220.829081, 164.203950, 224.141794},
  {60.861532, 59.138439, 256.171686, 177.516890, 219.646774, 163.922662, 222.992960},
  {60.845590, 59.154381, 256.033258, 177.224600, 218.465486, 163.638496, 221.844507},
  {60.830108, 59.169864, 255.897366, 176.929559, 217.285201, 163.351507, 220.696436},
  {60.641344, 59.015794, 255.764312, 176.632672, 216.109273, 163.062579, 219.552029},
  {60.076800, 58.494673, 255.634814, 176.335881, 214.944969, 162.773604, 218.418388},
  {59.107838, 57.577967, 255.509592, 176.041332, 213.800096, 162.486678, 217.303147},
  {57.734770, 56.265371, 255.389320, 175.751230, 212.682436, 162.203959, 216.213933},
  {55.957867, 54.556607, 255.274625, 175.467836, 211.599754, 161.927662, 215.158362},
  {53.950905, 52.620737, 255.165750, 175.192570, 210.556438, 161.659183, 214.140769},
  {51.916396, 50.655248, 255.062523, 174.925828, 209.552957, 161.398920, 213.161665},
  {700.504235, 699.628584, 76.628443, 161.277219, 212.714913, 174.897691, 209.477203},
  {22.245523, 72.305957, 80.726819, 161.364093, 213.151259, 175.181130, 210.895273},
  {23.638490, 66.918729, 84.269921, 161.425266, 213.620054, 175.355312, 212.222265},
  {25.180370, 61.381635, 87.233365, 161.462021, 214.122319, 175.445703, 213.446565},
  {26.613166, 55.952445, 89.634994, 161.476052, 214.654397, 175.475768, 214.565210},
  {27.782524, 50.785597, 91.517907, 161.470009, 215.210015, 175.465096, 215.580866},
  {28.616206, 45.953545, 92.937024, 161.447375, 215.781891, 175.428986, 216.499227},
  {29.094859, 41.475878, 93.950437, 161.412095, 216.362718, 175.378831, 217.327227},
  {29.227832, 37.343453, 94.614714, 161.368170, 216.945622, 175.322786, 218.071990},
  {29.037145, 33.534405, 94.982820, 161.319363, 217.524310, 175.266454, 218.740309},
  {28.548405, 30.023242, 95.103537, 161.268988, 218.093052, 175.213486, 219.338433},
  {27.786427, 26.785227, 95.021588, 161.219800, 218.646599, 175.166065, 219.872034},
  {26.773667, 23.797953, 94.778022, 161.173945, 219.180106, 175.125293, 220.346244},
  {25.530162, 21.041416, 94.410612, 161.132953, 219.689061, 175.091492, 220.765713},
  {24.074132, 18.497414, 93.954151, 161.097766, 220.169256, 175.064440, 221.134670},
  {22.422778, 16.148751, 93.440613, 161.068793, 220.616775, 175.043559, 221.456970},
  {20.593005, 13.978524, 92.899208, 161.045980, 221.028002, 175.028061, 221.736110},
  {18.601935, 11.969609, 92.356343, 161.028897, 221.399649, 175.017059, 221.975249},
  {16.467182, 10.104385, 91.835539, 161.016834, 221.728771, 175.009650, 222.177201},
  {14.206918, 8.364675, 91.357343, 161.008907, 222.012799, 175.004979, 222.344430},
  {11.839781, 6.731837, 90.939252, 161.004156, 222.249547, 175.002275, 222.479039},
  {9.384704, 5.186932, 90.595659, 161.001639, 222.437224, 175.000882, 222.582768},
  {6.860742, 3.710908, 90.337841, 161.000520, 222.574434, 175.000276, 222.656984},
  {4.511031, 2.403459, 90.165334, 161.000123, 222.664654, 175.000065, 222.705053},
  {2.618259, 1.381901, 90.064137, 161.000018, 222.717019, 175.000010, 222.732691},
  {1.236518, 0.649311, 90.016073, 161.000001, 222.741750, 175.000001, 222.745677},
  {0.374999, 0.196498, 90.001463, 161.000000, 222.749250, 175.000000, 222.749607},
  {0.037518, 0.019648, 90.000000, 161.000000, 222.750000, 175.000000, 222.750000},
  {0.000000, 0.000000, 90.000000, 161.000000, 222.750000, 175.000000, 222.750000}};
}}