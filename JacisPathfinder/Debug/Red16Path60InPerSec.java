/* Red16 Feed cross scale from side */
//----------------------------------------------------------------------------
//
//  $Workfile: Red16Path60InPerSec.java$
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

public class Red16Path60InPerSec extends Path {
    public Red16Path60InPerSec() {
       kNumPoints =407;
       kPoints = new double[][]{
  {0.400000, 0.400000, 359.998483, 16.002556, 287.250000, 16.002185, 273.250000},
  {0.862539, 0.737461, 359.988246, 16.019807, 287.249998, 16.016934, 273.249998},
  {1.940745, 1.659255, 359.965205, 16.058621, 287.249982, 16.050119, 273.249985},
  {3.450325, 2.949674, 359.924227, 16.127628, 287.249916, 16.109113, 273.249928},
  {5.391420, 4.608579, 359.860150, 16.235456, 287.249713, 16.201284, 273.249754},
  {7.548572, 6.451426, 359.770348, 16.386427, 287.249226, 16.330312, 273.249338},
  {9.706302, 8.293695, 359.654724, 16.580550, 287.248252, 16.496184, 273.248506},
  {11.864766, 10.135229, 359.513159, 16.817839, 287.246529, 16.698883, 273.247034},
  {14.024111, 11.975880, 359.345509, 17.098308, 287.243736, 16.938389, 273.244649},
  {16.184478, 13.815508, 359.151606, 17.421969, 287.239491, 17.214675, 273.241025},
  {18.345994, 15.653985, 358.931262, 17.788838, 287.233353, 17.527711, 273.235788},
  {20.508774, 17.491196, 358.684270, 18.198925, 287.224819, 17.877459, 273.228510},
  {22.672917, 19.327043, 358.410406, 18.652237, 287.213324, 18.263876, 273.218711},
  {24.838502, 21.161445, 358.109435, 19.148778, 287.198240, 18.686910, 273.205860},
  {27.005588, 22.994344, 357.781109, 19.688543, 287.178875, 19.146501, 273.189372},
  {29.174207, 24.825706, 357.425179, 20.271517, 287.154475, 19.642581, 273.168609},
  {31.344364, 26.655527, 357.041392, 20.897673, 287.124216, 20.175070, 273.142877},
  {33.516031, 28.483836, 356.629500, 21.566972, 287.087214, 20.743878, 273.111430},
  {35.689141, 30.310697, 356.189268, 22.279353, 287.042513, 21.348902, 273.073467},
  {37.863588, 32.136217, 355.720474, 23.034739, 286.989095, 21.990026, 273.028129},
  {40.039219, 33.960548, 355.222927, 23.833024, 286.925874, 22.667116, 272.974507},
  {42.215829, 35.783896, 354.696464, 24.674076, 286.851698, 23.380027, 272.911632},
  {44.393158, 37.606521, 354.140968, 25.557730, 286.765349, 24.128592, 272.838484},
  {46.570883, 39.428744, 353.556374, 26.483785, 286.665545, 24.912627, 272.753987},
  {48.748616, 41.250954, 352.942679, 27.451999, 286.550944, 25.731927, 272.657012},
  {50.925896, 43.073611, 352.299957, 28.462082, 286.420141, 26.586265, 272.546378},
  {53.102189, 44.897250, 351.628370, 29.513698, 286.271677, 27.475394, 272.420854},
  {55.276881, 46.722484, 350.928179, 30.606452, 286.104040, 28.399038, 272.279159},
  {57.449278, 48.550008, 350.199759, 31.739891, 285.915669, 29.356900, 272.119968},
  {59.618602, 50.380600, 349.443612, 32.913498, 285.704962, 30.348655, 271.941911},
  {61.567222, 52.031900, 348.663129, 34.122437, 285.471135, 31.370358, 271.744298},
  {63.077592, 53.321466, 347.864573, 35.357616, 285.214526, 32.414492, 271.527378},
  {64.149215, 54.249798, 347.054288, 36.609990, 284.935947, 33.473602, 271.291788},
  {64.781910, 54.817080, 346.238649, 37.870598, 284.636702, 34.540301, 271.038573},
  {64.975852, 55.023137, 345.424001, 39.130581, 284.318603, 35.607285, 270.769197},
  {64.948019, 55.050981, 344.613911, 40.385389, 283.982814, 36.670881, 270.484576},
  {64.915166, 55.083847, 343.809200, 41.634708, 283.629554, 37.730992, 270.184815},
  {64.877382, 55.121647, 343.010676, 42.878239, 283.259064, 38.787529, 269.870035},
  {64.834778, 55.164268, 342.219127, 44.115695, 282.871609, 39.840411, 269.540369},
  {64.787487, 55.211577, 341.435322, 45.346808, 282.467473, 40.889559, 269.195964},
  {64.735663, 55.263421, 340.660003, 46.571329, 282.046959, 41.934905, 268.836979},
  {64.679478, 55.319628, 339.893883, 47.789024, 281.610390, 42.976385, 268.463585},
  {64.619117, 55.380012, 339.137646, 48.999681, 281.158105, 44.013944, 268.075964},
  {64.554784, 55.444369, 338.391944, 50.203107, 280.690456, 45.047533, 267.674310},
  {64.486688, 55.512490, 337.657390, 51.399127, 280.207809, 46.077110, 267.258828},
  {64.415053, 55.584151, 336.934566, 52.587591, 279.710543, 47.102641, 266.829731},
  {64.340106, 55.659125, 336.224014, 53.768366, 279.199046, 48.124101, 266.387244},
  {64.262079, 55.737180, 335.526237, 54.941341, 278.673713, 49.141470, 265.931597},
  {64.181207, 55.818079, 334.841702, 56.106426, 278.134946, 50.154737, 265.463032},
  {64.097725, 55.901590, 334.170835, 57.263549, 277.583155, 51.163899, 264.981796},
  {64.011866, 55.987477, 333.514026, 58.412660, 277.018752, 52.168958, 264.488142},
  {63.923858, 56.075513, 332.871627, 59.553727, 276.442150, 53.169927, 263.982330},
  {63.833928, 56.165472, 332.243953, 60.686737, 275.853764, 54.166826, 263.464626},
  {63.742291, 56.257137, 331.631281, 61.811694, 275.254012, 55.159680, 262.935298},
  {63.649158, 56.350298, 331.033859, 62.928621, 274.643307, 56.148523, 262.394622},
  {63.554731, 56.444753, 330.451897, 64.037555, 274.022062, 57.133398, 261.842874},
  {63.459199, 56.540312, 329.885576, 65.138551, 273.390686, 58.114352, 261.280334},
  {63.362745, 56.636793, 329.335047, 66.231677, 272.749585, 59.091441, 260.707283},
  {63.265538, 56.734026, 328.800434, 67.317016, 272.099160, 60.064728, 260.124006},
  {63.167737, 56.831853, 328.281833, 68.394662, 271.439809, 61.034282, 259.530786},
  {63.069490, 56.930125, 327.779317, 69.464723, 270.771921, 62.000179, 258.927911},
  {62.970932, 57.028708, 327.292938, 70.527317, 270.095882, 62.962500, 258.315663},
  {62.872187, 57.127476, 326.822726, 71.582573, 269.412070, 63.921335, 257.694330},
  {62.773369, 57.226317, 326.368692, 72.630630, 268.720858, 64.876778, 257.064195},
  {62.674580, 57.325128, 325.930833, 73.671635, 268.022609, 65.828929, 256.425542},
  {62.575910, 57.423819, 325.509127, 74.705742, 267.317683, 66.777893, 255.778654},
  {62.477440, 57.522310, 325.103544, 75.733114, 266.606431, 67.723782, 255.123810},
  {62.379240, 57.620529, 324.714037, 76.753919, 265.889197, 68.666712, 254.461289},
  {62.281371, 57.718416, 324.340553, 77.768332, 265.166318, 69.606804, 253.791369},
  {62.183886, 57.815920, 323.983030, 78.776532, 264.438124, 70.544184, 253.114324},
  {62.086826, 57.912997, 323.641397, 79.778704, 263.704939, 71.478983, 252.430426},
  {61.990226, 58.009612, 323.315579, 80.775036, 262.967079, 72.411336, 251.739945},
  {61.894114, 58.105740, 323.005496, 81.765720, 262.224854, 73.341382, 251.043148},
  {61.798509, 58.201359, 322.711065, 82.750950, 261.478567, 74.269263, 250.340300},
  {61.703424, 58.296457, 322.432201, 83.730925, 260.728517, 75.195127, 249.631663},
  {61.608867, 58.391028, 322.168817, 84.705843, 259.974995, 76.119124, 248.917496},
  {61.514837, 58.485069, 321.920826, 85.675905, 259.218286, 77.041408, 248.198056},
  {61.421332, 58.578585, 321.688144, 86.641316, 258.458671, 77.962136, 247.473598},
  {61.328342, 58.671586, 321.470686, 87.602278, 257.696426, 78.881469, 246.744372},
  {61.235853, 58.764085, 321.268369, 88.558997, 256.931820, 79.799569, 246.010629},
  {61.143847, 58.856099, 321.081114, 89.511677, 256.165120, 80.716603, 245.272615},
  {61.052305, 58.947650, 320.908846, 90.460524, 255.396588, 81.632740, 244.530575},
  {60.961199, 59.038763, 320.751492, 91.405744, 254.626480, 82.548152, 243.784752},
  {60.870504, 59.129465, 320.608986, 92.347543, 253.855051, 83.463012, 243.035387},
  {60.780188, 59.219787, 320.481265, 93.286125, 253.082551, 84.377498, 242.282719},
  {60.690220, 59.309761, 320.368273, 94.221695, 252.309227, 85.291787, 241.526985},
  {60.600563, 59.399422, 320.269959, 95.154458, 251.535325, 86.206062, 240.768422},
  {60.511182, 59.488807, 320.186276, 96.084616, 250.761086, 87.120505, 240.007263},
  {60.422038, 59.577955, 320.117187, 97.012373, 249.986750, 88.035301, 239.243744},
  {60.333092, 59.666903, 320.062659, 97.937930, 249.212555, 88.950637, 238.478098},
  {60.244304, 59.755694, 320.022665, 98.861486, 248.438737, 89.866703, 237.710556},
  {60.155632, 59.844367, 319.997188, 99.783242, 247.665531, 90.783689, 236.941351},
  {60.067033, 59.932966, 319.986214, 100.703394, 246.893172, 91.701787, 236.170715},
  {59.978467, 60.021533, 319.989739, 101.622139, 246.121891, 92.621192, 235.398881},
  {59.889890, 60.110110, 320.007765, 102.539672, 245.351922, 93.542099, 234.626081},
  {59.801260, 60.198739, 320.040299, 103.456187, 244.583497, 94.464706, 233.852548},
  {59.712534, 60.287462, 320.087357, 104.371874, 243.816848, 95.389210, 233.078517},
  {59.623672, 60.376322, 320.148962, 105.286925, 243.052207, 96.315811, 232.304224},
  {59.534632, 60.465359, 320.225143, 106.201526, 242.289806, 97.244711, 231.529905},
  {59.445374, 60.554613, 320.315936, 107.115864, 241.529881, 98.176110, 230.755800},
  {59.355859, 60.644124, 320.421382, 108.030123, 240.772664, 99.110213, 229.982149},
  {59.266050, 60.733928, 320.541529, 108.944485, 240.018391, 100.047222, 229.209195},
  {59.175911, 60.824061, 320.676432, 109.859131, 239.267299, 100.987343, 228.437185},
  {59.085409, 60.914557, 320.826150, 110.774237, 238.519627, 101.930780, 227.666367},
  {58.994510, 61.005448, 320.990748, 111.689980, 237.775613, 102.877738, 226.896993},
  {58.903188, 61.096763, 321.170294, 112.606532, 237.035501, 103.828423, 226.129319},
  {58.811414, 61.188528, 321.364864, 113.524064, 236.299533, 104.783041, 225.363605},
  {58.719167, 61.280766, 321.574534, 114.442743, 235.567956, 105.741798, 224.600113},
  {58.626426, 61.373497, 321.799385, 115.362733, 234.841017, 106.704898, 223.839113},
  {58.533176, 61.466736, 322.039500, 116.284198, 234.118967, 107.672545, 223.080877},
  {58.439405, 61.560495, 322.294965, 117.207295, 233.402059, 108.644943, 222.325682},
  {58.345107, 61.654781, 322.565866, 118.132181, 232.690549, 109.622295, 221.573812},
  {58.250279, 61.749596, 322.852289, 119.059008, 231.984694, 110.604801, 220.825555},
  {58.154927, 61.844934, 323.154320, 119.987923, 231.284756, 111.592658, 220.081206},
  {58.059058, 61.940788, 323.472045, 120.919073, 230.590997, 112.586064, 219.341065},
  {57.962690, 62.037140, 323.805543, 121.852599, 229.903684, 113.585213, 218.605439},
  {57.865846, 62.133968, 324.154894, 122.788637, 229.223084, 114.590293, 217.874641},
  {57.768557, 62.231240, 324.520171, 123.727320, 228.549470, 115.601491, 217.148991},
  {57.670861, 62.328917, 324.901438, 124.668776, 227.883113, 116.618990, 216.428815},
  {57.572806, 62.426954, 325.298757, 125.613130, 227.224291, 117.642967, 215.714448},
  {57.474446, 62.525293, 325.712175, 126.560501, 226.573281, 118.673594, 215.006228},
  {57.375849, 62.623870, 326.141732, 127.511001, 225.930362, 119.711035, 214.304505},
  {57.277089, 62.722609, 326.587455, 128.464739, 225.295815, 120.755450, 213.609633},
  {57.178251, 62.821424, 327.049357, 129.421819, 224.669925, 121.806990, 212.921973},
  {57.079432, 62.920220, 327.527433, 130.382338, 224.052975, 122.865798, 212.241894},
  {56.980739, 63.018889, 328.021664, 131.346387, 223.445249, 123.932007, 211.569771},
  {56.882291, 63.117312, 328.532009, 132.314053, 222.847033, 125.005743, 210.905986},
  {56.784218, 63.215360, 329.058407, 133.285413, 222.258613, 126.087117, 210.250926},
  {56.686661, 63.312891, 329.600773, 134.260540, 221.680273, 127.176231, 209.604986},
  {56.589773, 63.409752, 330.158998, 135.239502, 221.112298, 128.273174, 208.968563},
  {56.493718, 63.505780, 330.732946, 136.222356, 220.554970, 129.378023, 208.342063},
  {56.398673, 63.600798, 331.322450, 137.209155, 220.008571, 130.490838, 207.725891},
  {56.304823, 63.694620, 331.927316, 138.199944, 219.473379, 131.611666, 207.120461},
  {56.212363, 63.787051, 332.547316, 139.194761, 218.949670, 132.740537, 206.526184},
  {56.121501, 63.877885, 333.182188, 140.193636, 218.437715, 133.877466, 205.943477},
  {56.032450, 63.966908, 333.831635, 141.196592, 217.937782, 135.022447, 205.372754},
  {55.945430, 64.053899, 334.495326, 142.203645, 217.450135, 136.175459, 204.814432},
  {56.037056, 63.864367, 335.136004, 143.217963, 216.973459, 137.331443, 204.271141},
  {56.681733, 63.317818, 335.679177, 144.248762, 216.501674, 138.482924, 203.744123},
  {56.638525, 63.361014, 336.229422, 145.283227, 216.040098, 139.640171, 203.227764},
  {56.595321, 63.404206, 336.786739, 146.321312, 215.588887, 140.803147, 202.722269},
  {56.552171, 63.447344, 337.351119, 147.362969, 215.148194, 141.971809, 202.227846},
  {56.509127, 63.490376, 337.922544, 148.408147, 214.718176, 143.146111, 201.744704},
  {56.466243, 63.533248, 338.500989, 149.456790, 214.298985, 144.325998, 201.273050},
  {56.423576, 63.575902, 339.086417, 150.508842, 213.890775, 145.511409, 200.813097},
  {56.381187, 63.618279, 339.678784, 151.564240, 213.493698, 146.702279, 200.365052},
  {56.339137, 63.660316, 340.278033, 152.622920, 213.107906, 147.898533, 199.929128},
  {56.297491, 63.701950, 340.884099, 153.684814, 212.733547, 149.100092, 199.505534},
  {56.256315, 63.743113, 341.496905, 154.749850, 212.370769, 150.306868, 199.094478},
  {56.215680, 63.783736, 342.116362, 155.817953, 212.019720, 151.518765, 198.696170},
  {56.175655, 63.823749, 342.742370, 156.889046, 211.680541, 152.735684, 198.310815},
  {56.136312, 63.863078, 343.374818, 157.963047, 211.353375, 153.957513, 197.938619},
  {56.097728, 63.901651, 344.013581, 159.039870, 211.038361, 155.184136, 197.579783},
  {56.059975, 63.939391, 344.658523, 160.119427, 210.735633, 156.415430, 197.234507},
  {56.023132, 63.976223, 345.309496, 161.201627, 210.445323, 157.651260, 196.902986},
  {55.987274, 64.012069, 345.966338, 162.286375, 210.167562, 158.891488, 196.585414},
  {55.952480, 64.046851, 346.628875, 163.373573, 209.902473, 160.135967, 196.281978},
  {55.918828, 64.080493, 347.296920, 164.463121, 209.650179, 161.384540, 195.992861},
  {55.886393, 64.112917, 347.970274, 165.554913, 209.410794, 162.637045, 195.718240},
  {55.855253, 64.144046, 348.648725, 166.648844, 209.184432, 163.893313, 195.458288},
  {55.825483, 64.173806, 349.332048, 167.744803, 208.971200, 165.153165, 195.213169},
  {55.797157, 64.202122, 350.020008, 168.842677, 208.771200, 166.416418, 194.983043},
  {55.770348, 64.228922, 350.712356, 169.942353, 208.584529, 167.682879, 194.768061},
  {55.745123, 64.254138, 351.408833, 171.043712, 208.411277, 168.952351, 194.568365},
  {55.721552, 64.277701, 352.109168, 172.146634, 208.251531, 170.224629, 194.384091},
  {55.699697, 64.299549, 352.813079, 173.250998, 208.105371, 171.499504, 194.215365},
  {55.679618, 64.319620, 353.520278, 174.356679, 207.972869, 172.776757, 194.062303},
  {55.661371, 64.337860, 354.230463, 175.463552, 207.854092, 174.056170, 193.925012},
  {55.645009, 64.354217, 354.943326, 176.571489, 207.749101, 175.337514, 193.803589},
  {55.630578, 64.368643, 355.658551, 177.680360, 207.657949, 176.620559, 193.698121},
  {55.618121, 64.381096, 356.375814, 178.790036, 207.580684, 177.905071, 193.608683},
  {55.607673, 64.391540, 357.094789, 179.900384, 207.517346, 179.190812, 193.535339},
  {55.599265, 64.399944, 357.815139, 181.011273, 207.467966, 180.477540, 193.478144},
  {55.592924, 64.406284, 358.536527, 182.122568, 207.432572, 181.765013, 193.437138},
  {55.588667, 64.410539, 359.258612, 183.234135, 207.411180, 183.052985, 193.412353},
  {55.586507, 64.412698, 359.981050, 184.345841, 207.403804, 184.341211, 193.403804},
  {55.586452, 64.412753, 0.703497, 185.457550, 207.410445, 185.629443, 193.411500},
  {55.588501, 64.410705, 1.425609, 186.569128, 207.431101, 186.917434, 193.435435},
  {55.592648, 64.406559, 2.147042, 187.680441, 207.465762, 188.204940, 193.475590},
  {55.598881, 64.400329, 2.867455, 188.791354, 207.514408, 189.491713, 193.531937},
  {55.607181, 64.392032, 3.586510, 189.901734, 207.577015, 190.777511, 193.604435},
  {55.617522, 64.381694, 4.303872, 191.011448, 207.653551, 192.062094, 193.693030},
  {55.629875, 64.369345, 5.019212, 192.120365, 207.743975, 193.345222, 193.797659},
  {55.644204, 64.355022, 5.732207, 193.228354, 207.848241, 194.626661, 193.918247},
  {55.660466, 64.338766, 6.442540, 194.335286, 207.966296, 195.906180, 194.054708},
  {55.678615, 64.320623, 7.149902, 195.441033, 208.098080, 197.183553, 194.206945},
  {55.698599, 64.300646, 7.853994, 196.545469, 208.243526, 198.458557, 194.374853},
  {55.720362, 64.278890, 8.554523, 197.648470, 208.402561, 199.730977, 194.558315},
  {55.743845, 64.255415, 9.251209, 198.749914, 208.575106, 201.000601, 194.757205},
  {55.768984, 64.230285, 9.943780, 199.849681, 208.761076, 202.267226, 194.971389},
  {55.795713, 64.203566, 10.631976, 200.947653, 208.960379, 203.530651, 195.200724},
  {55.823961, 64.175328, 11.315549, 202.043715, 209.172920, 204.790687, 195.445059},
  {55.853657, 64.145642, 11.994261, 203.137755, 209.398595, 206.047147, 195.704238},
  {55.884727, 64.114582, 12.667888, 204.229662, 209.637299, 207.299854, 195.978093},
  {55.917095, 64.082224, 13.336217, 205.319330, 209.888920, 208.548638, 196.266454},
  {55.950686, 64.048645, 13.999047, 206.406654, 210.153340, 209.793335, 196.569144},
  {55.985422, 64.013920, 14.656192, 207.491533, 210.430440, 211.033789, 196.885979},
  {56.021226, 63.978128, 15.307477, 208.573869, 210.720094, 212.269854, 197.216772},
  {56.058019, 63.941347, 15.952740, 209.653567, 211.022174, 213.501389, 197.561332},
  {56.095726, 63.903652, 16.591831, 210.730536, 211.336548, 214.728260, 197.919461},
  {56.134269, 63.865121, 17.224613, 211.804687, 211.663079, 215.950344, 198.290962},
  {56.173573, 63.825829, 17.850962, 212.875934, 212.001631, 217.167523, 198.675632},
  {56.213564, 63.785851, 18.470765, 213.944196, 212.352062, 218.379686, 199.073266},
  {56.254170, 63.745258, 19.083921, 215.009395, 212.714227, 219.586733, 199.483658},
  {56.295319, 63.704121, 19.690343, 216.071456, 213.087982, 220.788568, 199.906599},
  {56.336942, 63.662511, 20.289952, 217.130306, 213.473179, 221.985102, 200.341882},
  {56.378972, 63.620493, 20.882681, 218.185878, 213.869667, 223.176256, 200.789296},
  {56.421345, 63.578132, 21.468474, 219.238107, 214.277297, 224.361956, 201.248630},
  {56.463999, 63.535491, 22.047286, 220.286931, 214.695916, 225.542134, 201.719675},
  {56.506873, 63.492629, 22.619081, 221.332291, 215.125370, 226.716730, 202.202220},
  {56.549910, 63.449604, 23.183831, 222.374135, 215.565507, 227.885690, 202.696056},
  {56.593056, 63.406470, 23.741518, 223.412408, 216.016170, 229.048965, 203.200975},
  {56.636258, 63.363280, 24.292135, 224.447064, 216.477206, 230.206513, 203.716770},
  {56.679467, 63.320083, 24.835679, 225.478056, 216.948460, 231.358298, 204.243234},
  {700.472017, 700.297613, 205.174260, 230.974094, 204.062111, 225.018875, 216.732366},
  {58.427340, 61.572559, 205.431699, 229.917652, 203.562680, 223.905564, 216.206050},
  {58.381457, 61.618436, 205.696650, 228.864325, 203.058834, 222.793835, 215.674268},
  {58.333408, 61.666479, 205.969466, 227.814238, 202.550471, 221.683749, 215.136856},
  {58.283053, 61.716827, 206.250524, 226.767526, 202.037482, 220.575369, 214.593643},
  {58.230240, 61.769632, 206.540228, 225.724327, 201.519755, 219.468762, 214.044447},
  {58.174806, 61.825058, 206.839005, 224.684791, 200.997177, 218.363999, 213.489079},
  {58.116573, 61.883283, 207.147315, 223.649075, 200.469629, 217.261156, 212.927337},
  {58.055348, 61.944497, 207.465647, 222.617347, 199.936987, 216.160314, 212.359012},
  {57.990924, 62.008911, 207.794524, 221.589786, 199.399125, 215.061556, 211.783882},
  {57.923072, 62.076752, 208.134508, 220.566579, 198.855911, 213.964976, 211.201713},
  {57.851545, 62.148267, 208.486200, 219.547929, 198.307211, 212.870669, 210.612259},
  {57.776073, 62.223725, 208.850246, 218.534049, 197.752884, 211.778742, 210.015258},
  {57.696363, 62.303421, 209.227339, 217.525169, 197.192786, 210.689303, 209.410435},
  {57.612092, 62.387675, 209.618227, 216.521533, 196.626769, 209.602475, 208.797497},
  {57.522909, 62.476840, 210.023712, 215.523401, 196.054678, 208.518384, 208.176136},
  {57.428430, 62.571300, 210.444663, 214.531053, 195.476357, 207.437170, 207.546022},
  {57.328232, 62.671476, 210.882014, 213.544788, 194.891643, 206.358982, 206.906808},
  {57.221853, 62.777832, 211.336778, 212.564925, 194.300369, 205.283980, 206.258122},
  {57.108784, 62.890875, 211.810050, 211.591808, 193.702365, 204.212340, 205.599569},
  {56.988464, 63.011166, 212.303017, 210.625806, 193.097456, 203.144250, 204.930728},
  {56.860279, 63.139319, 212.816965, 209.667317, 192.485464, 202.079918, 204.251150},
  {56.723549, 63.276013, 213.353293, 208.716767, 191.866208, 201.019567, 203.560356},
  {56.577525, 63.421997, 213.913523, 207.774616, 191.239504, 199.963442, 202.857832},
  {56.421381, 63.578096, 214.499311, 206.841360, 190.605167, 198.911811, 202.143029},
  {56.254204, 63.745223, 215.112462, 205.917536, 189.963012, 197.864971, 201.415356},
  {56.074985, 63.924386, 215.754948, 205.003722, 189.312854, 196.823245, 200.674183},
  {55.882610, 64.116699, 216.428921, 204.100544, 188.654510, 195.786993, 199.918828},
  {55.675846, 64.323391, 217.136737, 203.208680, 187.987803, 194.756610, 199.148561},
  {55.453333, 64.545823, 217.880972, 202.328862, 187.312563, 193.732538, 198.362595},
  {55.213570, 64.785495, 218.664451, 201.461886, 186.628627, 192.715270, 197.560082},
  {54.954903, 65.044058, 219.490267, 200.608613, 185.935849, 191.705353, 196.740106},
  {54.675514, 65.323329, 220.361811, 199.769975, 185.234100, 190.703405, 195.901682},
  {54.373409, 65.625299, 221.282800, 198.946985, 184.523273, 189.710120, 195.043744},
  {54.046416, 65.952138, 222.257309, 198.140738, 183.803293, 188.726281, 194.165146},
  {53.692176, 66.306200, 223.289795, 197.352419, 183.074121, 187.752777, 193.264649},
  {53.308155, 66.690017, 224.385132, 196.583308, 182.335763, 186.790617, 192.340922},
  {52.891657, 67.106281, 225.548635, 195.834787, 181.588282, 185.840955, 191.392532},
  {52.439858, 67.557810, 226.786080, 195.108342, 180.831808, 184.905110, 190.417947},
  {51.949866, 68.047490, 228.103717, 194.405562, 180.066553, 183.984593, 189.415533},
  {51.418810, 68.578185, 229.508267, 193.728143, 179.292821, 183.081148, 188.383557},
  {50.843981, 69.152599, 231.006890, 193.077883, 178.511024, 182.196780, 187.320201},
  {50.223021, 69.773079, 232.607136, 192.456668, 177.721700, 181.333804, 186.223576},
  {49.554205, 70.441343, 234.316835, 191.866461, 176.925519, 180.494892, 185.091755},
  {48.836804, 71.158112, 236.143936, 191.309276, 176.123298, 179.683120, 183.922817},
  {48.071565, 71.922630, 238.096267, 190.787145, 175.316001, 178.902024, 182.714912},
  {47.261306, 72.732074, 240.181193, 190.302077, 174.504729, 178.155645, 181.466352},
  {46.411621, 73.580846, 242.405164, 189.856004, 173.690706, 177.448570, 180.175732},
  {45.531656, 74.459804, 244.773133, 189.450722, 172.875231, 176.785940, 178.842080},
  {44.634862, 75.355506, 247.287854, 189.087829, 172.059623, 176.173441, 177.465045},
  {43.739602, 76.249611, 249.949074, 188.768664, 171.245132, 175.617228, 176.045105},
  {42.869384, 77.118643, 252.752694, 188.494266, 170.432840, 175.123792, 174.583793},
  {42.052485, 77.934373, 255.689988, 188.265352, 169.623542, 174.699736, 173.083898},
  {41.320723, 78.665040, 258.747025, 188.082332, 168.817648, 174.351480, 171.549626},
  {40.707247, 79.277566, 261.904448, 187.945365, 168.015107, 174.084880, 169.986648},
  {40.243429, 79.740644, 265.137768, 187.854438, 167.215391, 173.904818, 168.402033},
  {39.955260, 80.028345, 268.418242, 187.809469, 166.417552, 173.814804, 166.803999},
  {39.859904, 80.123544, 271.714321, 187.810392, 165.620355, 173.816658, 165.201530},
  {39.963215, 80.020402, 274.993494, 187.857209, 164.822463, 173.910345, 163.603866},
  {40.258855, 79.725243, 278.224289, 187.949988, 164.022649, 174.093969, 162.019970},
  {40.729247, 79.255600, 281.378113, 188.088810, 163.219980, 174.363957, 160.458020},
  {41.348114, 78.637691, 284.430667, 188.273681, 162.413947, 174.715382, 158.925031},
  {42.083943, 77.902961, 287.362814, 188.504436, 161.604518, 175.142357, 157.426618},
  {42.903585, 77.084490, 290.160837, 188.780652, 160.792119, 175.638449, 155.966926},
  {43.775328, 76.213932, 292.816210, 189.101599, 159.977561, 176.197051, 154.548692},
  {44.671070, 75.319344, 295.325006, 189.466227, 159.161933, 176.811683, 153.173400},
  {45.567507, 74.423996, 297.687109, 189.873183, 158.346491, 177.476208, 151.841492},
  {46.446482, 73.546024, 299.905376, 190.320861, 157.532554, 178.184962, 150.552587},
  {47.294730, 72.698685, 301.984832, 190.807459, 156.721420, 178.932822, 149.305693},
  {48.103264, 71.890962, 303.931975, 191.331040, 155.914306, 179.715228, 148.099391},
  {48.866616, 71.128327, 305.754198, 191.889594, 155.112311, 180.528157, 146.931983},
  {49.582064, 70.413508, 307.459338, 192.481087, 154.316391, 181.368095, 145.801616},
  {50.248931, 69.747189, 309.055343, 193.103509, 153.527358, 182.231981, 144.706367},
  {50.867995, 69.128602, 310.550037, 193.754898, 152.745876, 183.117159, 143.644310},
  {51.441015, 68.555997, 311.950952, 194.433371, 151.972478, 184.021328, 142.613559},
  {51.970363, 68.027006, 313.265235, 195.137136, 151.207571, 184.942494, 141.612298},
  {52.458762, 67.538918, 314.499586, 195.864501, 150.451456, 185.878924, 140.638799},
  {52.909084, 67.088864, 315.660237, 196.613878, 149.704340, 186.829113, 139.691430},
  {53.324221, 66.673960, 316.752945, 197.383788, 148.966351, 187.791750, 138.768665},
  {53.706993, 66.291392, 317.783006, 198.172853, 148.237549, 188.765689, 137.869074},
  {54.060088, 65.938473, 318.755276, 198.979796, 147.517938, 189.749924, 136.991331},
  {54.386035, 65.612679, 319.674199, 199.803437, 146.807479, 190.743573, 136.134201},
  {54.687185, 65.311664, 320.543833, 200.642682, 146.106093, 191.745854, 135.296539},
  {54.965703, 65.033263, 321.367881, 201.496525, 145.413673, 192.756078, 134.477285},
  {55.223575, 64.775494, 322.149723, 202.364034, 144.730091, 193.773631, 133.675454},
  {55.462612, 64.536548, 322.892440, 203.244351, 144.055197, 194.797966, 132.890136},
  {55.684463, 64.314777, 323.598845, 204.136685, 143.388830, 195.828593, 132.120484},
  {55.890622, 64.108688, 324.271507, 205.040303, 142.730820, 196.865074, 131.365714},
  {56.082445, 63.916928, 324.912771, 205.954532, 142.080987, 197.907012, 130.625097},
  {56.261159, 63.738271, 325.524784, 206.878747, 141.439150, 198.954051, 129.897955},
  {56.427873, 63.571606, 326.109509, 207.812370, 140.805124, 200.005867, 129.183657},
  {56.583593, 63.415931, 326.668746, 208.754869, 140.178724, 201.062168, 128.481615},
  {56.729227, 63.270337, 327.204145, 209.705747, 139.559763, 202.122684, 127.791282},
  {56.865599, 63.134000, 327.717222, 210.664547, 138.948060, 203.187172, 127.112146},
  {56.993455, 63.006176, 328.209372, 211.630843, 138.343432, 204.255408, 126.443728},
  {57.113471, 62.886189, 328.681876, 212.604238, 137.745702, 205.327187, 125.785580},
  {57.226261, 62.773425, 329.135919, 213.584365, 137.154696, 206.402320, 125.137282},
  {57.332382, 62.667328, 329.572592, 214.570882, 136.570242, 207.480633, 124.498441},
  {57.432341, 62.567390, 329.992902, 215.563468, 135.992175, 208.561966, 123.868686},
  {57.526599, 62.473151, 330.397784, 216.561827, 135.420332, 209.646170, 123.247670},
  {57.615576, 62.384191, 330.788101, 217.565679, 134.854556, 210.733106, 122.635065},
  {57.699657, 62.300127, 331.164655, 218.574764, 134.294693, 211.822646, 122.030563},
  {57.779191, 62.220608, 331.528191, 219.588840, 133.740595, 212.914672, 121.433871},
  {57.854498, 62.145314, 331.879399, 220.607678, 133.192119, 214.009072, 120.844714},
  {57.925873, 62.073952, 332.218925, 221.631064, 132.649123, 215.105742, 120.262834},
  {57.993582, 62.006254, 332.547367, 222.658797, 132.111474, 216.204585, 119.687982},
  {58.057873, 61.941973, 332.865285, 223.690688, 131.579039, 217.305510, 119.119926},
  {58.118973, 61.880883, 333.173202, 224.726561, 131.051693, 218.408431, 118.558445},
  {58.177090, 61.822774, 333.471606, 225.766248, 130.529312, 219.513270, 118.003329},
  {58.232415, 61.767457, 333.760953, 226.809591, 130.011779, 220.619949, 117.454377},
  {58.285126, 61.714754, 334.041672, 227.856442, 129.498978, 221.728399, 116.911401},
  {58.335386, 61.664501, 334.314164, 228.906661, 128.990798, 222.838553, 116.374220},
  {58.383345, 61.616548, 334.578806, 229.960116, 128.487133, 223.950346, 115.842661},
  {58.429144, 61.570756, 334.835950, 231.016681, 127.987877, 225.063720, 115.316561},
  {56.119452, 63.801385, 335.464730, 232.034065, 127.513865, 226.220518, 114.777984},
  {51.753599, 68.243626, 336.814488, 232.980666, 127.095152, 227.468733, 114.225863},
  {51.700627, 68.296562, 338.172916, 233.935921, 126.699337, 228.730627, 113.702994},
  {51.669490, 68.327678, 339.536439, 234.899729, 126.326552, 230.005166, 113.210026},
  {51.661237, 68.335926, 340.901313, 235.871986, 125.976881, 231.291238, 112.747491},
  {51.676627, 68.320546, 342.263669, 236.852582, 125.650355, 232.587663, 112.315796},
  {51.716097, 68.281103, 343.619564, 237.841405, 125.346955, 233.893211, 111.915210},
  {51.779748, 68.217494, 344.965043, 238.838332, 125.066609, 235.206616, 111.545861},
  {51.867344, 68.129957, 346.296186, 239.843234, 124.809196, 236.526594, 111.207729},
  {51.978316, 68.019058, 347.609167, 240.855970, 124.574542, 237.851863, 110.900649},
  {52.111792, 67.885669, 348.900304, 241.876392, 124.362424, 239.181158, 110.624312},
  {52.266623, 67.730936, 350.166101, 242.904340, 124.172569, 240.513245, 110.378271},
  {52.441429, 67.556240, 351.403289, 243.939640, 124.004656, 241.846940, 110.161947},
  {52.634641, 67.363146, 352.608856, 244.982111, 123.858318, 243.181119, 109.974643},
  {52.844555, 67.153355, 353.780068, 246.031563, 123.733138, 244.514730, 109.815551},
  {53.069385, 66.928655, 354.914483, 247.087796, 123.628658, 245.846801, 109.683769},
  {53.307304, 66.690868, 356.009959, 248.150606, 123.544378, 247.176443, 109.578312},
  {53.556499, 66.441806, 357.064651, 249.219784, 123.479756, 248.502857, 109.498124},
  {53.815203, 66.183236, 358.077001, 250.295125, 123.434212, 249.825335, 109.442097},
  {54.081731, 65.916840, 359.045730, 251.376420, 123.407134, 251.143259, 109.409075},
  {54.354507, 65.644192, 359.969813, 252.463471, 123.397873, 252.456095, 109.397875},
  {54.632084, 65.366740, 0.848465, 253.556084, 123.405755, 253.763396, 109.407290},
  {54.913156, 65.085788, 1.681114, 254.654078, 123.430078, 255.064792, 109.436104},
  {55.196566, 64.802493, 2.467376, 255.757283, 123.470115, 256.359990, 109.483095},
  {55.481306, 64.517860, 3.207033, 256.865545, 123.525121, 257.648762, 109.547047},
  {55.766522, 64.232747, 3.900007, 257.978726, 123.594332, 258.930942, 109.626752},
  {56.051500, 63.947864, 4.546336, 259.096706, 123.676967, 260.206420, 109.721017},
  {56.335666, 63.663786, 5.146154, 260.219384, 123.772235, 261.475137, 109.828667},
  {56.618571, 63.380962, 5.699666, 261.346680, 123.879332, 262.737075, 109.948546},
  {56.899886, 63.099722, 6.207131, 262.478532, 123.997444, 263.992255, 110.079519},
  {57.179387, 62.820289, 6.668846, 263.614899, 124.125751, 265.240729, 110.220476},
  {57.456945, 62.542791, 7.085129, 264.755761, 124.263425, 266.482576, 110.370329},
  {57.732516, 62.267274, 7.456305, 265.901117, 124.409633, 267.717898, 110.528015},
  {58.006130, 61.993708, 7.782693, 267.050986, 124.563535, 268.946814, 110.692492},
  {58.277880, 61.721999, 8.064598, 268.205405, 124.724288, 270.169458, 110.862740},
  {58.547915, 61.451999, 8.302301, 269.364429, 124.891041, 271.385972, 111.037762},
  {58.816426, 61.183516, 8.496050, 270.528129, 125.062942, 272.596506, 111.216577},
  {59.083646, 60.916319, 8.646056, 271.696594, 125.239129, 273.801215, 111.398227},
  {59.349834, 60.650149, 8.752489, 272.869923, 125.418735, 275.000252, 111.581767},
  {59.615273, 60.384721, 8.815469, 274.048233, 125.600886, 276.193770, 111.766268},
  {59.880261, 60.119738, 8.835070, 275.231648, 125.784700, 277.381918, 111.950816},
  {60.145106, 59.854893, 8.811316, 276.420304, 125.969283, 278.564838, 112.134509},
  {60.410118, 59.589875, 8.744178, 277.614344, 126.153732, 279.742665, 112.316454},
  {60.649398, 59.298868, 8.633636, 278.813399, 126.337052, 280.915020, 112.495694},
  {60.659202, 58.785616, 8.480281, 280.013067, 126.517639, 282.077633, 112.670706},
  {60.257777, 57.883589, 8.285951, 281.205335, 126.693413, 283.222925, 112.839557},
  {59.438342, 56.599570, 8.053594, 282.382028, 126.862413, 284.343418, 113.000489},
  {58.194233, 54.940226, 7.787249, 283.534801, 127.022858, 285.431732, 113.151966},
  {56.545769, 52.936955, 7.491864, 284.655670, 127.173261, 286.481066, 113.292774},
  {54.695071, 50.787637, 7.172035, 285.740620, 127.312915, 287.488506, 113.422455},
  {52.821591, 48.661105, 6.831494, 286.789168, 127.441742, 288.454465, 113.541138},
  {50.925979, 46.556708, 6.473864, 287.800825, 127.559780, 289.379324, 113.649052},
  {49.008881, 44.473801, 6.102662, 288.775101, 127.667177, 290.263445, 113.746515},
  {47.070947, 42.411733, 5.721300, 289.711509, 127.764177, 291.107164, 113.833916},
  {45.112845, 40.369837, 5.333078, 290.609568, 127.851109, 291.910804, 113.911712},
  {43.135267, 38.347419, 4.941187, 291.468806, 127.928381, 292.674670, 113.980410},
  {41.138937, 36.343757, 4.548695, 292.288763, 127.996463, 293.399052, 114.040559},
  {39.124612, 34.358091, 4.158549, 293.068996, 128.055882, 294.084230, 114.092741},
  {37.093084, 32.389630, 3.773566, 293.809080, 128.107209, 294.730470, 114.137562},
  {35.045182, 30.437545, 3.396425, 294.508611, 128.151049, 295.338029, 114.175640},
  {32.981767, 28.500974, 3.029666, 295.167209, 128.188032, 295.907151, 114.207600},
  {30.903730, 26.579024, 2.675684, 295.784517, 128.218801, 296.438073, 114.234064},
  {28.811989, 24.670778, 2.336721, 296.360206, 128.244006, 296.931016, 114.255647},
  {26.707484, 22.775296, 2.014866, 296.893970, 128.264291, 297.386193, 114.272947},
  {24.591169, 20.891624, 1.712054, 297.385533, 128.280289, 297.803805, 114.286538},
  {22.464010, 19.018793, 1.430059, 297.834644, 128.292610, 298.184038, 114.296970},
  {20.326982, 17.155831, 1.170496, 298.241079, 128.301837, 298.527066, 114.304759},
  {18.181060, 15.301762, 0.934823, 298.604639, 128.308519, 298.833049, 114.310383},
  {16.027217, 13.455612, 0.724334, 298.925150, 128.313161, 299.102133, 114.314280},
  {13.866425, 11.616409, 0.540167, 299.202462, 128.316222, 299.334447, 114.316845},
  {11.699648, 9.783191, 0.383303, 299.436447, 128.318109, 299.530105, 114.318422},
  {9.527842, 7.955000, 0.254564, 299.627001, 128.319170, 299.689202, 114.319308},
  {7.351956, 6.130887, 0.154618, 299.774039, 128.319695, 299.811819, 114.319746},
  {5.201149, 4.333411, 0.083593, 299.878062, 128.319911, 299.898487, 114.319926},
  {3.294662, 2.743329, 0.038466, 299.943955, 128.319981, 299.953354, 114.319984},
  {1.823568, 1.517855, 0.013443, 299.980426, 128.319998, 299.983711, 114.319998},
  {0.788590, 0.656264, 0.002612, 299.996198, 128.320000, 299.996836, 114.320000},
  {0.190097, 0.158188, 0.000000, 300.000000, 128.320000, 300.000000, 114.320000},
  {0.000000, 0.000000, 0.000000, 300.000000, 128.320000, 300.000000, 114.320000}};
}}
