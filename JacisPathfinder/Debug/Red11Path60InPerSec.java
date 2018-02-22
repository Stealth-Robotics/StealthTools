/* Red11 Feed Switch from Side start 1 aud */
//----------------------------------------------------------------------------
//
//  $Workfile: Red11Path60InPerSec.java$
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

public class Red11Path60InPerSec extends Path {
    public Red11Path60InPerSec() {
       kNumPoints =189;
       kPoints = new double[][]{
  {0.400000, 0.400000, 359.995186, 16.003098, 314.750000, 16.001922, 300.750000},
  {0.987604, 0.612396, 359.964475, 16.022850, 314.749993, 16.014170, 300.749996},
  {2.223203, 1.376796, 359.895196, 16.067314, 314.749938, 16.041706, 300.749962},
  {3.956123, 2.443876, 359.771417, 16.146436, 314.749708, 16.090583, 300.749820},
  {6.191163, 3.808832, 359.576420, 16.270258, 314.749004, 16.166759, 300.749387},
  {8.687802, 5.312184, 359.300122, 16.444005, 314.747301, 16.272997, 300.748346},
  {11.205238, 6.794732, 358.939116, 16.668084, 314.743860, 16.408876, 300.746259},
  {13.750323, 8.249621, 358.488877, 16.943021, 314.737691, 16.573827, 300.742560},
  {16.330626, 9.669276, 357.943636, 17.269475, 314.727530, 16.767119, 300.736546},
  {18.954638, 11.045202, 357.296236, 17.648242, 314.711795, 16.987833, 300.727381},
  {21.631984, 12.367768, 356.537944, 18.080256, 314.688541, 17.234831, 300.714091},
  {24.373655, 13.625972, 355.658226, 18.566600, 314.655388, 17.506720, 300.695565},
  {27.192244, 14.807207, 354.644483, 19.108501, 314.609447, 17.801806, 300.670561},
  {30.102161, 15.897050, 353.481759, 19.707319, 314.547214, 18.118046, 300.637713},
  {33.119754, 16.879124, 352.152416, 20.364522, 314.464440, 18.452985, 300.595553},
  {36.263285, 17.735139, 350.635822, 21.081632, 314.355981, 18.803705, 300.542544},
  {39.552535, 18.445269, 348.908103, 21.860128, 314.215604, 19.166764, 300.477126},
  {43.007812, 18.989146, 346.942052, 22.701277, 314.035779, 19.538167, 300.397790},
  {46.647910, 19.347907, 344.707375, 23.605863, 313.807446, 19.913378, 300.303167},
  {50.486394, 19.507893, 342.171543, 24.573756, 313.519814, 20.287402, 300.192129},
  {54.525481, 19.466783, 339.301649, 25.603280, 313.160240, 20.655009, 300.063881},
  {58.746829, 19.242818, 336.067773, 26.690314, 312.714335, 21.011133, 299.917972},
  {63.099322, 18.887042, 332.448346, 27.827172, 312.166462, 21.351499, 299.754143},
  {67.485909, 18.496529, 328.437692, 29.001369, 311.500879, 21.673412, 299.571879},
  {71.755028, 18.223020, 324.054969, 30.194675, 310.703675, 21.976552, 299.369548},
  {75.706012, 18.267586, 319.352250, 31.383050, 309.765418, 22.263355, 299.143217},
  {79.117904, 18.851789, 314.417876, 32.538122, 308.683908, 22.538561, 298.885500},
  {81.801988, 20.165018, 309.371190, 33.630438, 307.465929, 22.807701, 298.585143},
  {83.660494, 22.305536, 304.347609, 34.633772, 306.126918, 23.074955, 298.227946},
  {84.721158, 25.245727, 299.478012, 35.528962, 304.688271, 23.341337, 297.799017},
  {84.832972, 28.736614, 294.885232, 36.303336, 303.178636, 23.603201, 297.287408},
  {83.962316, 32.411374, 290.664775, 36.952628, 301.629995, 23.853371, 296.689400},
  {82.402606, 35.975692, 286.863982, 37.481984, 300.069271, 24.084038, 296.007862},
  {80.391797, 39.190942, 283.491158, 37.902186, 298.517316, 24.288503, 295.251181},
  {78.089678, 41.896959, 280.528414, 38.226487, 296.989563, 24.462185, 294.431439},
  {75.834210, 44.155553, 277.935258, 38.469215, 295.492428, 24.603270, 293.559671},
  {73.881319, 46.110814, 275.662062, 38.643624, 294.025130, 24.711928, 292.643878},
  {72.199082, 47.794843, 273.664447, 38.760574, 292.585893, 24.789198, 291.691110},
  {70.751490, 49.243791, 271.903948, 38.828925, 291.172515, 24.836654, 290.707378},
  {69.503745, 50.492568, 270.347815, 38.855908, 289.782702, 24.856166, 289.697715},
  {68.424575, 51.572528, 268.968422, 38.847455, 288.414236, 24.849724, 288.666285},
  {67.486942, 52.510770, 267.742583, 38.808471, 287.065060, 24.819336, 287.616509},
  {66.667969, 53.330216, 266.650856, 38.743042, 285.733307, 24.766953, 286.551192},
  {65.948518, 54.050037, 265.676940, 38.654607, 284.417305, 24.694438, 285.472626},
  {65.312672, 54.686176, 264.807142, 38.546087, 283.115567, 24.603547, 284.382685},
  {64.747207, 55.251873, 264.029932, 38.419988, 281.826777, 24.495918, 283.282902},
  {64.241133, 55.758132, 263.335585, 38.278478, 280.549771, 24.373077, 282.174526},
  {63.785287, 56.214129, 262.715874, 38.123454, 279.283520, 24.236440, 281.058577},
  {63.371995, 56.627541, 262.163831, 37.956589, 278.027112, 24.087322, 279.935886},
  {62.994802, 57.004832, 261.673544, 37.779373, 276.779742, 23.926947, 278.807125},
  {62.648240, 57.351473, 261.239996, 37.593144, 275.540694, 23.756455, 277.672837},
  {62.327650, 57.672128, 260.858936, 37.399116, 274.309334, 23.576913, 276.533454},
  {62.029027, 57.970806, 260.526766, 37.198400, 273.085099, 23.389323, 275.389314},
  {61.748897, 58.250978, 260.240457, 36.992019, 271.867487, 23.194629, 274.240678},
  {61.484222, 58.515688, 259.997479, 36.780929, 270.656056, 22.993727, 273.087737},
  {61.232313, 58.767625, 259.795741, 36.566026, 269.450413, 22.787472, 271.930624},
  {60.990758, 59.009202, 259.633548, 36.348160, 268.250212, 22.576682, 270.769416},
  {60.757369, 59.242608, 259.509564, 36.128145, 267.055148, 22.362151, 269.604148},
  {60.530126, 59.469863, 259.422780, 35.906764, 265.864960, 22.144647, 268.434807},
  {60.307135, 59.692861, 259.372501, 35.684782, 264.679420, 21.924925, 267.261343},
  {60.086589, 59.913410, 259.358326, 35.462947, 263.498341, 21.703729, 266.083668},
  {59.866728, 60.133271, 259.380143, 35.242002, 262.321568, 21.481800, 264.901656},
  {59.645807, 60.354187, 259.438124, 35.022688, 261.148986, 21.259882, 263.715147},
  {59.422059, 60.577927, 259.532733, 34.805751, 259.980512, 21.038727, 262.523944},
  {59.193661, 60.806312, 259.664731, 34.591949, 258.816105, 20.819102, 261.327814},
  {58.958695, 61.041260, 259.835191, 34.382054, 257.655762, 20.601796, 260.126485},
  {58.715110, 61.284822, 260.045525, 34.176865, 256.499525, 20.387629, 258.919644},
  {58.460673, 61.539231, 260.297509, 33.977208, 255.347485, 20.177462, 257.706936},
  {58.192918, 61.806949, 260.593321, 33.783945, 254.199785, 19.972201, 256.487958},
  {57.909086, 62.090736, 260.935595, 33.597982, 253.056630, 19.772815, 255.262254},
  {57.606052, 62.393714, 261.327471, 33.420274, 251.918296, 19.580346, 254.029312},
  {57.280238, 62.719460, 261.772679, 33.251838, 250.785142, 19.395924, 252.788554},
  {56.927508, 63.072107, 262.275622, 33.093754, 249.657620, 19.220788, 251.539329},
  {56.543040, 63.456473, 262.841497, 32.947179, 248.536298, 19.056306, 250.280903},
  {56.121169, 63.878217, 263.476423, 32.813356, 247.421881, 18.904003, 249.012450},
  {55.655195, 64.344035, 264.187619, 32.693620, 246.315236, 18.765596, 247.733034},
  {55.137147, 64.861889, 264.983606, 32.589412, 245.217428, 18.643036, 246.441599},
  {54.557492, 65.441300, 265.874467, 32.502279, 244.129763, 18.538556, 245.136950},
  {53.904786, 66.093698, 266.872156, 32.433886, 243.053838, 18.454742, 243.817735},
  {53.165259, 66.832835, 267.990880, 32.386011, 241.991612, 18.394617, 242.482432},
  {52.322327, 67.675269, 269.247560, 32.360538, 240.945475, 18.361745, 241.129325},
  {51.356065, 68.640887, 270.662378, 32.359435, 239.918354, 18.360370, 239.756508},
  {50.242695, 69.753422, 272.259405, 32.384712, 238.913818, 18.395596, 238.361885},
  {48.954203, 71.040821, 274.067294, 32.438345, 237.936204, 18.473605, 236.943211},
  {47.458382, 72.535204, 276.119970, 32.522157, 236.990744, 18.601946, 235.498196},
  {45.719791, 74.271894, 278.457154, 32.637634, 236.083669, 18.789868, 234.024693},
  {43.702561, 76.286609, 281.124436, 32.785657, 235.222244, 19.048711, 232.521077},
  {41.376564, 78.609293, 284.172336, 32.966166, 234.414639, 19.392275, 230.986890},
  {38.729149, 81.252402, 287.653448, 33.177768, 233.669520, 19.837053, 229.423894},
  {35.784858, 84.191231, 291.616354, 33.417434, 232.995144, 20.402034, 227.837685},
  {32.633886, 87.335571, 296.094890, 33.680561, 232.397856, 21.107625, 226.239829},
  {29.463819, 90.498142, 301.092203, 33.961808, 231.880028, 21.973085, 224.650192},
  {26.576756, 93.377655, 306.562009, 34.257084, 231.438053, 23.012107, 223.098359},
  {24.361030, 95.587117, 312.394463, 34.566466, 231.061666, 24.227178, 221.622432},
  {23.194035, 96.750647, 318.417927, 34.896686, 230.735875, 25.604995, 220.263794},
  {23.300288, 96.644714, 324.423999, 35.261120, 230.445446, 27.116167, 219.058622},
  {24.648961, 95.300025, 330.209326, 35.676338, 230.179696, 28.720680, 218.029848},
  {26.969167, 92.986313, 335.614909, 36.156904, 229.934767, 30.376760, 217.183692},
  {29.870491, 90.092479, 340.545669, 36.711452, 229.712563, 32.048677, 216.511861},
  {32.979926, 86.990300, 344.967577, 37.341668, 229.517887, 33.710549, 215.996978},
  {36.022418, 83.954138, 348.891610, 38.043641, 229.355779, 35.346322, 215.618077},
  {38.835575, 81.146160, 352.355306, 38.810099, 229.229988, 36.947685, 215.354419},
  {41.346897, 78.638916, 355.408061, 39.632398, 229.142519, 38.511576, 215.187457},
  {43.541976, 76.446979, 358.101621, 40.501874, 229.093804, 40.038097, 215.101488},
  {45.438104, 74.553248, 0.484900, 41.410573, 229.083096, 41.529055, 215.083598},
  {47.066702, 72.926477, 2.601672, 42.351554, 229.108878, 42.987044, 215.123309},
  {48.463164, 71.531407, 4.489919, 43.318939, 229.169192, 44.414910, 215.212156},
  {49.661665, 70.333975, 6.182032, 44.307838, 229.261874, 45.815464, 215.343287},
  {50.692888, 69.303579, 7.705382, 45.314227, 229.384715, 47.191337, 215.511126},
  {51.583335, 68.413775, 9.083006, 46.334807, 229.535554, 48.544919, 215.711105},
  {52.355406, 67.642211, 10.334273, 47.366884, 229.712339, 49.878354, 215.939448},
  {53.027806, 66.970210, 11.475493, 48.408255, 229.913150, 51.193538, 216.193011},
  {53.616070, 66.382268, 12.520435, 49.457119, 230.136215, 52.492148, 216.469152},
  {54.133063, 65.865532, 13.480762, 50.511998, 230.379906, 53.775662, 216.765631},
  {54.589446, 65.409360, 14.366393, 51.571678, 230.642740, 55.045382, 217.080536},
  {54.994066, 65.004912, 15.185798, 52.635158, 230.923361, 56.302458, 217.412221},
  {55.354286, 64.644833, 15.946246, 53.701612, 231.220540, 57.547907, 217.759262},
  {55.676257, 64.322980, 16.653994, 54.770354, 231.533155, 58.782633, 218.120414},
  {55.965132, 64.034203, 17.314460, 55.840816, 231.860187, 60.007437, 218.494587},
  {56.225252, 63.774168, 17.932350, 56.912523, 232.200709, 61.223037, 218.880819},
  {56.460279, 63.539209, 18.511771, 57.985081, 232.553873, 62.430074, 219.278255},
  {56.673330, 63.326218, 19.056319, 59.058159, 232.918906, 63.629123, 219.686133},
  {56.867056, 63.132544, 19.569158, 60.131482, 233.295103, 64.820704, 220.103772},
  {57.043725, 62.955918, 20.053079, 61.204817, 233.681814, 66.005285, 220.530559},
  {57.205288, 62.794393, 20.510555, 62.277972, 234.078446, 67.183291, 220.965938},
  {57.353427, 62.646288, 20.943782, 63.350784, 234.484452, 68.355109, 221.409410},
  {57.489597, 62.510146, 21.354721, 64.423117, 234.899330, 69.521089, 221.860516},
  {57.615063, 62.384705, 21.745122, 65.494857, 235.322616, 70.681554, 222.318841},
  {57.730929, 62.268861, 22.116558, 66.565910, 235.753881, 71.836798, 222.784003},
  {57.838158, 62.161650, 22.470441, 67.636197, 236.192728, 72.987092, 223.255652},
  {57.937600, 62.062226, 22.808047, 68.705653, 236.638788, 74.132684, 223.733466},
  {58.029998, 61.969843, 23.130528, 69.774224, 237.091722, 75.273804, 224.217149},
  {58.116012, 61.883843, 23.438929, 70.841866, 237.551209, 76.410665, 224.706425},
  {58.196225, 61.803642, 23.734201, 71.908543, 238.016956, 77.543463, 225.201041},
  {58.271155, 61.728723, 24.017207, 72.974228, 238.488684, 78.672382, 225.700758},
  {58.341263, 61.658624, 24.288737, 74.038897, 238.966137, 79.797590, 226.205358},
  {58.406963, 61.592935, 24.549512, 75.102534, 239.449072, 80.919246, 226.714636},
  {58.468619, 61.531284, 24.800195, 76.165125, 239.937263, 82.037498, 227.228398},
  {61.432807, 58.525129, 24.562197, 77.279795, 240.454088, 83.099326, 227.720940},
  {75.495718, 44.494471, 22.024503, 78.666618, 241.051217, 83.916661, 228.072887},
  {75.615629, 44.374407, 19.467162, 80.080869, 241.586925, 84.746600, 228.387268},
  {75.583817, 44.406260, 16.915033, 81.517002, 242.058821, 85.590347, 228.664500},
  {75.393885, 44.596432, 14.394024, 82.968968, 242.465628, 86.449212, 228.905100},
  {75.047088, 44.943660, 11.929835, 84.430508, 242.807280, 87.324500, 229.109659},
  {74.552129, 45.439219, 9.546739, 85.895470, 243.084937, 88.217400, 229.278828},
  {73.924025, 46.068054, 7.266546, 87.358094, 243.300893, 89.128890, 229.413334},
  {73.182293, 46.810607, 5.107866, 88.813239, 243.458416, 90.059673, 229.514012},
  {72.348821, 47.644950, 3.085722, 90.256535, 243.561541, 91.010155, 229.581840},
  {71.445808, 48.548841, 1.211496, 91.684457, 243.614848, 91.980459, 229.617977},
  {70.494070, 49.501432, 359.493161, 93.094313, 243.623240, 92.970471, 229.623788},
  {69.511848, 50.484457, 357.935700, 94.484194, 243.591766, 93.979899, 229.600851},
  {68.514142, 51.482897, 356.541639, 95.852872, 243.525465, 95.008348, 229.550961},
  {67.512505, 52.485190, 355.311613, 97.199691, 243.429267, 96.055380, 229.476111},
  {66.502341, 53.472618, 354.245100, 98.524192, 243.307938, 97.120368, 229.378499},
  {65.272859, 54.255983, 353.343347, 99.821969, 243.166543, 98.199099, 229.260922},
  {63.639166, 54.643480, 352.607035, 101.085166, 243.010620, 99.283732, 229.127002},
  {61.626530, 54.609827, 352.032708, 102.306607, 242.845655, 100.366098, 228.980792},
  {59.257106, 54.132868, 351.613282, 103.479668, 242.676864, 101.437716, 228.826578},
  {56.562277, 53.204537, 351.338447, 104.598387, 242.508981, 102.490022, 228.668649},
  {53.760553, 52.006338, 351.194863, 105.661104, 242.345545, 103.518061, 228.510540},
  {51.043910, 50.723008, 351.168597, 106.669890, 242.188883, 104.520505, 228.354862},
  {48.406774, 49.360138, 351.246631, 107.626626, 242.040757, 105.496085, 228.203821},
  {45.844320, 47.922565, 351.416737, 108.533020, 241.902443, 106.443569, 228.059243},
  {43.352427, 46.414421, 351.667365, 109.390622, 241.774803, 107.361745, 227.922596},
  {40.927579, 44.839229, 351.987538, 110.200847, 241.658345, 108.249408, 227.795016},
  {38.566749, 43.200021, 352.366777, 110.964992, 241.553272, 109.105356, 227.677331},
  {36.267273, 41.499465, 352.795039, 111.684255, 241.459537, 109.928387, 227.570083},
  {34.026724, 39.739991, 353.262678, 112.359751, 241.376877, 110.717304, 227.473555},
  {31.842798, 37.923902, 353.760424, 112.992521, 241.304856, 111.470917, 227.387790},
  {29.713218, 36.053477, 354.279383, 113.583547, 241.242898, 112.188058, 227.312621},
  {27.635653, 34.131045, 354.811040, 114.133754, 241.190318, 112.867584, 227.247692},
  {25.607661, 32.159047, 355.347281, 114.644016, 241.146354, 113.508392, 227.192489},
  {23.626650, 30.140075, 355.880414, 115.115163, 241.110190, 114.109426, 227.146362},
  {21.689861, 28.076886, 356.403201, 115.547976, 241.080981, 114.669689, 227.108558},
  {19.794365, 25.972406, 356.908882, 115.943188, 241.057873, 115.188252, 227.078243},
  {17.937082, 23.829715, 357.391202, 116.301486, 241.040024, 115.664256, 227.054534},
  {16.114797, 21.652024, 357.844431, 116.623503, 241.026619, 116.096922, 227.036526},
  {14.324199, 19.442646, 358.263383, 116.909821, 241.016884, 116.485551, 227.023314},
  {12.561904, 17.204961, 358.643423, 117.160968, 241.010099, 116.829524, 227.014023},
  {10.824496, 14.942386, 358.980477, 117.377411, 241.005607, 117.128308, 227.007824},
  {9.108548, 12.658349, 359.271033, 117.559561, 241.002826, 117.381445, 227.003959},
  {7.410644, 10.356262, 359.512135, 117.707765, 241.001251, 117.588559, 227.001759},
  {5.727400, 8.039513, 359.701384, 117.822311, 241.000465, 117.749345, 227.000655},
  {4.065134, 5.725091, 359.837253, 117.903613, 241.000137, 117.863846, 227.000194},
  {2.586287, 3.650556, 359.924365, 117.955338, 241.000030, 117.936857, 227.000042},
  {1.443367, 2.040093, 359.973208, 117.984206, 241.000004, 117.977659, 227.000005},
  {0.633725, 0.896351, 359.994704, 117.996880, 241.000000, 117.995586, 227.000000},
  {0.155993, 0.220699, 360.000000, 118.000000, 241.000000, 118.000000, 227.000000},
  {0.000000, 0.000000, 360.000000, 118.000000, 241.000000, 118.000000, 227.000000}};
}}