/* Red25 Feed Scale from front aud */
//----------------------------------------------------------------------------
//
//  $Workfile: Red25Path60InPerSec.java$
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

public class Red25Path60InPerSec extends Path {
    public Red25Path60InPerSec() {
       kNumPoints =318;
       kPoints = new double[][]{
  {0.400000, 0.400000, 359.997363, 16.003032, 242.750000, 16.002387, 228.750000},
  {0.895110, 0.704890, 359.981793, 16.020934, 242.749997, 16.016485, 228.749997},
  {2.013948, 1.586052, 359.946770, 16.061213, 242.749972, 16.048206, 228.749978},
  {3.580181, 2.819818, 359.884533, 16.132816, 242.749866, 16.104603, 228.749895},
  {5.593592, 4.406406, 359.787360, 16.244688, 242.749546, 16.192730, 228.749642},
  {7.830112, 6.169885, 359.651469, 16.401288, 242.748779, 16.316127, 228.749038},
  {10.065682, 7.934311, 359.477014, 16.602596, 242.747248, 16.474808, 228.747831},
  {12.299987, 9.700000, 359.264202, 16.848581, 242.744545, 16.668796, 228.745700},
  {14.532676, 11.467303, 359.013297, 17.139202, 242.740176, 16.898117, 228.742252},
  {16.763360, 13.236608, 358.724629, 17.474403, 242.733558, 17.162797, 228.737026},
  {18.991603, 15.008351, 358.398595, 17.854116, 242.724023, 17.462870, 228.729491},
  {21.216917, 16.783020, 358.035675, 18.278249, 242.710820, 17.798368, 228.719047},
  {23.438758, 18.561157, 357.636437, 18.746690, 242.693118, 18.169326, 228.705029},
  {25.656523, 20.343367, 357.201548, 19.259299, 242.670009, 18.575780, 228.686705},
  {27.869545, 22.130315, 356.731784, 19.815909, 242.640512, 19.017766, 228.663281},
  {30.077095, 23.922731, 356.228041, 20.416316, 242.603575, 19.495318, 228.633902},
  {32.278379, 25.721409, 355.691344, 21.060279, 242.558086, 20.008468, 228.597652},
  {34.472544, 27.527202, 355.122858, 21.747515, 242.502873, 20.557243, 228.553562},
  {36.658680, 29.341019, 354.523896, 22.477698, 242.436715, 21.141670, 228.500610},
  {38.835829, 31.163820, 353.895931, 23.250451, 242.358349, 21.761765, 228.437723},
  {41.002991, 32.996606, 353.240596, 24.065348, 242.266477, 22.417543, 228.363788},
  {43.159136, 34.840405, 352.559694, 24.921911, 242.159776, 23.109007, 228.277652},
  {45.303216, 36.696267, 351.855201, 25.819606, 242.036912, 23.836153, 228.178127},
  {47.434184, 38.565241, 351.129264, 26.757848, 241.896544, 24.598967, 228.064001},
  {49.551004, 40.448362, 350.384197, 27.735997, 241.737340, 25.397427, 227.934040},
  {51.652671, 42.346636, 349.622482, 28.753362, 241.557989, 26.231498, 227.786997},
  {53.738232, 44.261019, 348.846756, 29.809206, 241.357209, 27.101133, 227.621622},
  {55.806797, 46.192401, 348.059801, 30.902747, 241.133765, 28.006278, 227.436668},
  {57.857559, 48.141590, 347.264531, 32.033166, 240.886475, 28.946865, 227.230899},
  {59.889806, 50.109299, 346.463979, 33.199612, 240.614226, 29.922819, 227.003104},
  {61.686033, 51.913044, 345.664043, 34.397008, 240.317055, 30.930508, 226.753008},
  {63.031919, 53.367156, 344.872965, 35.616194, 239.996440, 31.962753, 226.481546},
  {63.932286, 54.466812, 344.098199, 36.848236, 239.654377, 33.012384, 226.190119},
  {64.392355, 55.206787, 343.346345, 38.084444, 239.293350, 34.072244, 225.880585},
  {64.417584, 55.581619, 342.623107, 39.316384, 238.916292, 35.135201, 225.555240},
  {64.227282, 55.771988, 341.931027, 40.539948, 238.525206, 36.197685, 225.215632},
  {64.037554, 55.961781, 341.270013, 41.755209, 238.120898, 37.259687, 224.862305},
  {63.848766, 56.150629, 340.639908, 42.962268, 237.704156, 38.321211, 224.495803},
  {63.661227, 56.338225, 340.040510, 44.161255, 237.275749, 39.382275, 224.116670},
  {63.475191, 56.524316, 339.471570, 45.352320, 236.836428, 40.442911, 223.725451},
  {63.290858, 56.708700, 338.932811, 46.535637, 236.386923, 41.503162, 223.322690},
  {63.108382, 56.891223, 338.423928, 47.711395, 235.927947, 42.563088, 222.908925},
  {62.927873, 57.071777, 337.944599, 48.879801, 235.460191, 43.622760, 222.484694},
  {62.749400, 57.250291, 337.494490, 50.041071, 234.984327, 44.682259, 222.050529},
  {62.573000, 57.426730, 337.073261, 51.195435, 234.501010, 45.741682, 221.606958},
  {62.398674, 57.601091, 336.680573, 52.343131, 234.010876, 46.801135, 221.154505},
  {62.226400, 57.773398, 336.316089, 53.484404, 233.514543, 47.860735, 220.693687},
  {62.056125, 57.943702, 335.979482, 54.619505, 233.012615, 48.920612, 220.225019},
  {61.887779, 58.112076, 335.670436, 55.748688, 232.505679, 49.980904, 219.749008},
  {61.721271, 58.278609, 335.388650, 56.872211, 231.994307, 51.041759, 219.266156},
  {61.556492, 58.443409, 335.133840, 57.990335, 231.479060, 52.103335, 218.776964},
  {61.393322, 58.606599, 334.905744, 59.103320, 230.960483, 53.165799, 218.281925},
  {61.231626, 58.768312, 334.704119, 60.211425, 230.439114, 54.229325, 217.781528},
  {61.071260, 58.928693, 334.528747, 61.314913, 229.915477, 55.294098, 217.276260},
  {60.912071, 59.087895, 334.379436, 62.414039, 229.390089, 56.360308, 216.766605},
  {60.753897, 59.246080, 334.256020, 63.509062, 228.863457, 57.428153, 216.253043},
  {60.596573, 59.403413, 334.158358, 64.600233, 228.336084, 58.497838, 215.736053},
  {60.439927, 59.560065, 334.086341, 65.687803, 227.808462, 59.569576, 215.216112},
  {60.283785, 59.716212, 334.039884, 66.772020, 227.281083, 60.643584, 214.693697},
  {60.127969, 59.872031, 334.018935, 67.853124, 226.754430, 61.720087, 214.169286},
  {59.972299, 60.027701, 334.023470, 68.931354, 226.228987, 62.799313, 213.643357},
  {59.816597, 60.183402, 334.053493, 70.006942, 225.705232, 63.881497, 213.116391},
  {59.660681, 60.339314, 334.109040, 71.080117, 225.183643, 64.966879, 212.588869},
  {59.504375, 60.495615, 334.190174, 72.151099, 224.664697, 66.055703, 212.061280},
  {59.347500, 60.652483, 334.296989, 73.220106, 224.148872, 67.148216, 211.534113},
  {59.189884, 60.810090, 334.429604, 74.287346, 223.636646, 68.244670, 211.007866},
  {59.031356, 60.968606, 334.588171, 75.353022, 223.128497, 69.345319, 210.483043},
  {58.871752, 61.128196, 334.772863, 76.417329, 222.624907, 70.450420, 209.960153},
  {58.710914, 61.289019, 334.983884, 77.480457, 222.126362, 71.560233, 209.439717},
  {58.548691, 61.451223, 335.221460, 78.542585, 221.633347, 72.675017, 208.922264},
  {58.384943, 61.614951, 335.485840, 79.603886, 221.146355, 73.795032, 208.408332},
  {58.219539, 61.780332, 335.777295, 80.664523, 220.665881, 74.920541, 207.898475},
  {58.052363, 61.947483, 336.096115, 81.724651, 220.192425, 76.051800, 207.393254},
  {57.883312, 62.116506, 336.442607, 82.784414, 219.726493, 77.189069, 206.893250},
  {57.712301, 62.287485, 336.817092, 83.843948, 219.268593, 78.332600, 206.399054},
  {57.539268, 62.460485, 337.219900, 84.903377, 218.819242, 79.482641, 205.911274},
  {57.364169, 62.635547, 337.651369, 85.962815, 218.378958, 80.639437, 205.430536},
  {57.186990, 62.812687, 338.111840, 87.022365, 217.948268, 81.803221, 204.957482},
  {57.007744, 62.991891, 338.601650, 88.082118, 217.527701, 82.974219, 204.492773},
  {56.826479, 63.173110, 339.121131, 89.142153, 217.117791, 84.152645, 204.037087},
  {56.643278, 63.356262, 339.670598, 90.202535, 216.719076, 85.338699, 203.591125},
  {56.458267, 63.541221, 340.250349, 91.263319, 216.332097, 86.532565, 203.155604},
  {56.271615, 63.727818, 340.860650, 92.324544, 215.957397, 87.734408, 202.731262},
  {56.083541, 63.915833, 341.501735, 93.386236, 215.595522, 88.944373, 202.318856},
  {55.894318, 64.104994, 342.173792, 94.448410, 215.247015, 90.162579, 201.919163},
  {55.704275, 64.294972, 342.876954, 95.511064, 214.912422, 91.389118, 201.532977},
  {55.513802, 64.485377, 343.611293, 96.574184, 214.592283, 92.624051, 201.161109},
  {55.323352, 64.675755, 344.376803, 97.637742, 214.287137, 93.867406, 200.804387},
  {55.133445, 64.865588, 345.173396, 98.701696, 213.997515, 95.119171, 200.463649},
  {54.944668, 65.054289, 346.000887, 99.765991, 213.723940, 96.379294, 200.139747},
  {54.757674, 65.241205, 346.858984, 100.830559, 213.466928, 97.647680, 199.833539},
  {54.573182, 65.425616, 347.747276, 101.895321, 213.226981, 98.924183, 199.545887},
  {54.391976, 65.606740, 348.665227, 102.960186, 213.004588, 100.208608, 199.277650},
  {54.214896, 65.783738, 349.612161, 104.025050, 212.800222, 101.500705, 199.029686},
  {54.042833, 65.955718, 350.587255, 105.089803, 212.614339, 102.800167, 198.802837},
  {53.876720, 66.121750, 351.589537, 106.154323, 212.447371, 104.106631, 198.597932},
  {53.717520, 66.280870, 352.617875, 107.218480, 212.299733, 105.419673, 198.415774},
  {53.566211, 66.432099, 353.670977, 108.282140, 212.171810, 106.738811, 198.257137},
  {53.423775, 66.574460, 354.747391, 109.345159, 212.063965, 108.063501, 198.122755},
  {53.291174, 66.706989, 355.845508, 110.407390, 211.976531, 109.393145, 198.013318},
  {53.169335, 66.828761, 356.963565, 111.468681, 211.909811, 110.727088, 197.929467},
  {53.059132, 66.938902, 358.099658, 112.528878, 211.864078, 112.064622, 197.871778},
  {52.961361, 67.036618, 359.251753, 113.587822, 211.839572, 113.404996, 197.840766},
  {52.876726, 67.121203, 0.417699, 114.645352, 211.836500, 114.747414, 197.836872},
  {52.805821, 67.192066, 1.595250, 115.701306, 211.855032, 116.091048, 197.860458},
  {52.749112, 67.248742, 2.782082, 116.755519, 211.895308, 117.435043, 197.911809},
  {52.706927, 67.290903, 3.975818, 117.807825, 211.957429, 118.778522, 197.991121},
  {52.679443, 67.318370, 5.174053, 118.858058, 212.041460, 120.120600, 198.098506},
  {52.666690, 67.331116, 6.374374, 119.906047, 212.147433, 121.460390, 198.233986},
  {52.668539, 67.329268, 7.574393, 120.951623, 212.275342, 122.797011, 198.397498},
  {52.684713, 67.313103, 8.771765, 121.994615, 212.425146, 124.129598, 198.588895},
  {52.714792, 67.283042, 9.964214, 123.034848, 212.596769, 125.457310, 198.807945},
  {52.758224, 67.239635, 11.149555, 124.072149, 212.790101, 126.779338, 199.054340},
  {52.814340, 67.183553, 12.325711, 125.106345, 213.004999, 128.094908, 199.327701},
  {52.882368, 67.115565, 13.490734, 126.137260, 213.241288, 129.403294, 199.627581},
  {52.961454, 67.036524, 14.642814, 127.164720, 213.498760, 130.703814, 199.953472},
  {53.050682, 66.947348, 15.780290, 128.188553, 213.777178, 131.995842, 200.304816},
  {53.149088, 66.848996, 16.901661, 129.208586, 214.076279, 133.278805, 200.681007},
  {53.255687, 66.742457, 18.005585, 130.224653, 214.395773, 134.552189, 201.081404},
  {53.369482, 66.628724, 19.090885, 131.236588, 214.735345, 135.815534, 201.505331},
  {53.489487, 66.508784, 20.156544, 132.244231, 215.094657, 137.068439, 201.952092},
  {53.614736, 66.383600, 21.201705, 133.247427, 215.473355, 138.310560, 202.420973},
  {53.744301, 66.254103, 22.225659, 134.246030, 215.871064, 139.541606, 202.911246},
  {53.877295, 66.121176, 23.227847, 135.239898, 216.287395, 140.761339, 203.422182},
  {54.012885, 65.985652, 24.207844, 136.228899, 216.721946, 141.969569, 203.953050},
  {54.986826, 64.818233, 25.012562, 137.228176, 217.181148, 143.147613, 204.494136},
  {59.573364, 60.426629, 25.082403, 138.307591, 217.685586, 144.242489, 205.005799},
  {59.603186, 60.396808, 25.147362, 139.386951, 218.191547, 145.336221, 205.518497},
  {59.632914, 60.367081, 25.207454, 140.466297, 218.698938, 146.428855, 206.032135},
  {59.662554, 60.337441, 25.262695, 141.545668, 219.207668, 147.520436, 206.546620},
  {59.692113, 60.307883, 25.313096, 142.625104, 219.717646, 148.611007, 207.061858},
  {59.721598, 60.278399, 25.358671, 143.704646, 220.228779, 149.700614, 207.577757},
  {59.751016, 60.248982, 25.399430, 144.784334, 220.740978, 150.789300, 208.094224},
  {59.780374, 60.219624, 25.435383, 145.864209, 221.254152, 151.877110, 208.611169},
  {59.809678, 60.190321, 25.466539, 146.944312, 221.768210, 152.964087, 209.128498},
  {59.838936, 60.161063, 25.492906, 148.024686, 222.283062, 154.050277, 209.646121},
  {59.868154, 60.131845, 25.514489, 149.105371, 222.798617, 155.135722, 210.163948},
  {59.897341, 60.102659, 25.531294, 150.186411, 223.314786, 156.220467, 210.681886},
  {59.926502, 60.073498, 25.543326, 151.267846, 223.831478, 157.304555, 211.199845},
  {59.955646, 60.044354, 25.550587, 152.349721, 224.348604, 158.388031, 211.717736},
  {59.984778, 60.015222, 25.553079, 153.432077, 224.866073, 159.470936, 212.235468},
  {60.013907, 59.986093, 25.550802, 154.514958, 225.383796, 160.553315, 212.752951},
  {60.043039, 59.956961, 25.543757, 155.598407, 225.901681, 161.635211, 213.270094},
  {60.072181, 59.927818, 25.531941, 156.682467, 226.419640, 162.716666, 213.786808},
  {60.101341, 59.898658, 25.515351, 157.767181, 226.937581, 163.797722, 214.303002},
  {60.130526, 59.869473, 25.493983, 158.852594, 227.455414, 164.878422, 214.818587},
  {60.159743, 59.840256, 25.467833, 159.938749, 227.973048, 165.958809, 215.333473},
  {60.188999, 59.811000, 25.436893, 161.025690, 228.490393, 167.038924, 215.847568},
  {60.218300, 59.781698, 25.401157, 162.113461, 229.007357, 168.118808, 216.360784},
  {60.247655, 59.752343, 25.360615, 163.202107, 229.523848, 169.198504, 216.873029},
  {60.277069, 59.722928, 25.315258, 164.291671, 230.039775, 170.278051, 217.384213},
  {60.306550, 59.693446, 25.265075, 165.382197, 230.555045, 171.357491, 217.894244},
  {60.336105, 59.663890, 25.210053, 166.473732, 231.069565, 172.436864, 218.403033},
  {60.365741, 59.634254, 25.150180, 167.566317, 231.583243, 173.516210, 218.910486},
  {60.395463, 59.604530, 25.085441, 168.659999, 232.095986, 174.595570, 219.416514},
  {60.425280, 59.574713, 25.015822, 169.754822, 232.607697, 175.674981, 219.921023},
  {60.455197, 59.544794, 24.941304, 170.850830, 233.118285, 176.754484, 220.423921},
  {60.485222, 59.514769, 24.861871, 171.948067, 233.627652, 177.834117, 220.925116},
  {60.515360, 59.484629, 24.777505, 173.046579, 234.135703, 178.913918, 221.424514},
  {60.545618, 59.454370, 24.688185, 174.146409, 234.642342, 179.993925, 221.922021},
  {60.576002, 59.423985, 24.593891, 175.247601, 235.147471, 181.074175, 222.417544},
  {60.606518, 59.393467, 24.494602, 176.350201, 235.650993, 182.154706, 222.910988},
  {60.637172, 59.362811, 24.390294, 177.454251, 236.152809, 183.235553, 223.402258},
  {60.667971, 59.332011, 24.280944, 178.559797, 236.652819, 184.316754, 223.891258},
  {60.698919, 59.301061, 24.166528, 179.666881, 237.150923, 185.398342, 224.377891},
  {60.730023, 59.269955, 24.047019, 180.775548, 237.647021, 186.480354, 224.862062},
  {60.761287, 59.238689, 23.922393, 181.885840, 238.141010, 187.562824, 225.343672},
  {60.792717, 59.207258, 23.792621, 182.997801, 238.632786, 188.645785, 225.822624},
  {60.824317, 59.175655, 23.657677, 184.111473, 239.122248, 189.729271, 226.298818},
  {60.856094, 59.143877, 23.517530, 185.226900, 239.609288, 190.813315, 226.772156},
  {60.888050, 59.111918, 23.372151, 186.344124, 240.093802, 191.897948, 227.242537},
  {60.920190, 59.079775, 23.221511, 187.463185, 240.575683, 192.983203, 227.709860},
  {60.952518, 59.047445, 23.065579, 188.584126, 241.054822, 194.069109, 228.174024},
  {60.985039, 59.014922, 22.904322, 189.706988, 241.531111, 195.155697, 228.634926},
  {61.017754, 58.982203, 22.737710, 190.831812, 242.004439, 196.242996, 229.092464},
  {61.050668, 58.949286, 22.565710, 191.958636, 242.474694, 197.331034, 229.546534},
  {61.083784, 58.916169, 22.388288, 193.087501, 242.941765, 198.419841, 229.997030},
  {61.117102, 58.882847, 22.205412, 194.218446, 243.405537, 199.509441, 230.443849},
  {61.150626, 58.849320, 22.017047, 195.351508, 243.865895, 200.599862, 230.886882},
  {61.184357, 58.815586, 21.823161, 196.486726, 244.322723, 201.691129, 231.326024},
  {61.218295, 58.781644, 21.623718, 197.624135, 244.775902, 202.783267, 231.761166},
  {61.252443, 58.747493, 21.418685, 198.763772, 245.225315, 203.876298, 232.192200},
  {61.286799, 58.713133, 21.208027, 199.905672, 245.670841, 204.970245, 232.619017},
  {61.321364, 58.678565, 20.991711, 201.049869, 246.112357, 206.065130, 233.041505},
  {61.356136, 58.643789, 20.769702, 202.196397, 246.549742, 207.160973, 233.459555},
  {61.391114, 58.608807, 20.541967, 203.345286, 246.982870, 208.257793, 233.873054},
  {61.426296, 58.573621, 20.308472, 204.496569, 247.411616, 209.355610, 234.281889},
  {61.461679, 58.538234, 20.069185, 205.650276, 247.835853, 210.454440, 234.685947},
  {61.497259, 58.502649, 19.824073, 206.806434, 248.255451, 211.554299, 235.085114},
  {61.533033, 58.466871, 19.573104, 207.965072, 248.670282, 212.655202, 235.479275},
  {61.568996, 58.430904, 19.316247, 209.126214, 249.080214, 213.757162, 235.868314},
  {61.605141, 58.394754, 19.053473, 210.289887, 249.485114, 214.860193, 236.252114},
  {61.641463, 58.358427, 18.784753, 211.456112, 249.884848, 215.964305, 236.630558},
  {61.677954, 58.321931, 18.510058, 212.624911, 250.279280, 217.069507, 237.003529},
  {61.714606, 58.285274, 18.229363, 213.796304, 250.668274, 218.175808, 237.370908},
  {61.751410, 58.248465, 17.942643, 214.970308, 251.051691, 219.283214, 237.732576},
  {61.788356, 58.211514, 17.649874, 216.146939, 251.429393, 220.391732, 238.088413},
  {61.825432, 58.174432, 17.351036, 217.326212, 251.801238, 221.501365, 238.438300},
  {61.862627, 58.137231, 17.046107, 218.508139, 252.167084, 222.612115, 238.782115},
  {61.899928, 58.099925, 16.735073, 219.692729, 252.526788, 223.723984, 239.119739},
  {61.937320, 58.062527, 16.417916, 220.879989, 252.880207, 224.836969, 239.451048},
  {61.974789, 58.025052, 16.094625, 222.069926, 253.227194, 225.951070, 239.775922},
  {62.012317, 57.987518, 15.765191, 223.262542, 253.567604, 227.066280, 240.094239},
  {62.049888, 57.949941, 15.429605, 224.457837, 253.901289, 228.182596, 240.405876},
  {62.087482, 57.912340, 15.087864, 225.655808, 254.228101, 229.300009, 240.710712},
  {62.125081, 57.874735, 14.739968, 226.856452, 254.547891, 230.418509, 241.008624},
  {62.162662, 57.837147, 14.385919, 228.059759, 254.860509, 231.538085, 241.299490},
  {62.200204, 57.799599, 14.025724, 229.265720, 255.165806, 232.658725, 241.583187},
  {62.237682, 57.762113, 13.659393, 230.474319, 255.463629, 233.780412, 241.859595},
  {62.275073, 57.724716, 13.286941, 231.685541, 255.753829, 234.903132, 242.128591},
  {62.312350, 57.687432, 12.908386, 232.899364, 256.036253, 236.026863, 242.390054},
  {62.349485, 57.650289, 12.523750, 234.115767, 256.310751, 237.151587, 242.643864},
  {62.386452, 57.613316, 12.133063, 235.334721, 256.577170, 238.277279, 242.889900},
  {62.423219, 57.576541, 11.736356, 236.556196, 256.835358, 239.403916, 243.128043},
  {62.459756, 57.539997, 11.333668, 237.780159, 257.085166, 240.531471, 243.358175},
  {62.496032, 57.503714, 10.925040, 239.006571, 257.326442, 241.659915, 243.580178},
  {62.532013, 57.467726, 10.510522, 240.235392, 257.559035, 242.789217, 243.793935},
  {62.567664, 57.432066, 10.090166, 241.466577, 257.782797, 243.919345, 243.999332},
  {62.602953, 57.396771, 9.664034, 242.700076, 257.997580, 245.050264, 244.196253},
  {62.637841, 57.361875, 9.232189, 243.935837, 258.203235, 246.181938, 244.384587},
  {62.672292, 57.327416, 8.794704, 245.173803, 258.399617, 247.314326, 244.564222},
  {62.706269, 57.293432, 8.351656, 246.413914, 258.586582, 248.447390, 244.735049},
  {62.739734, 57.259960, 7.903129, 247.656105, 258.763988, 249.581086, 244.896960},
  {62.772647, 57.227040, 7.449214, 248.900306, 258.931693, 250.715369, 245.049850},
  {62.804969, 57.194710, 6.990007, 250.146445, 259.089559, 251.850193, 245.193616},
  {62.836660, 57.163012, 6.525611, 251.394446, 259.237452, 252.985509, 245.328155},
  {62.867680, 57.131984, 6.056137, 252.644228, 259.375236, 254.121267, 245.453370},
  {62.897990, 57.101668, 5.581700, 253.895705, 259.502783, 255.257415, 245.569163},
  {62.927548, 57.072102, 5.102424, 255.148789, 259.619964, 256.393899, 245.675442},
  {62.956315, 57.043328, 4.618438, 256.403386, 259.726656, 257.530662, 245.772114},
  {62.984251, 57.015385, 4.129878, 257.659402, 259.822739, 258.667648, 245.859092},
  {63.011317, 56.988313, 3.636887, 258.916734, 259.908097, 259.804797, 245.936292},
  {63.037474, 56.962149, 3.139613, 260.175279, 259.982617, 260.942048, 246.003630},
  {63.062684, 56.936933, 2.638212, 261.434930, 260.046191, 262.079339, 246.061029},
  {63.086911, 56.912700, 2.132844, 262.695574, 260.098715, 263.216606, 246.108414},
  {63.110117, 56.889488, 1.623677, 263.957098, 260.140091, 264.353784, 246.145713},
  {63.132269, 56.867331, 1.110883, 265.219384, 260.170226, 265.490807, 246.172857},
  {63.153333, 56.846261, 0.594641, 266.482311, 260.189030, 266.627606, 246.189784},
  {63.173277, 56.826312, 0.075133, 267.745754, 260.196420, 267.764113, 246.196432},
  {63.192072, 56.807512, 359.552548, 269.009589, 260.192319, 268.900257, 246.192746},
  {63.209689, 56.789891, 359.027078, 270.273686, 260.176655, 270.035968, 246.178673},
  {63.226101, 56.773474, 358.498922, 271.537913, 260.149360, 271.171173, 246.154165},
  {63.241286, 56.758285, 357.968279, 272.802138, 260.110377, 272.305799, 246.119178},
  {63.255222, 56.744346, 357.435355, 274.066225, 260.059650, 273.439773, 246.073673},
  {63.267888, 56.731676, 356.900356, 275.330038, 259.997132, 274.573021, 246.017614},
  {63.279268, 56.720293, 356.363495, 276.593437, 259.922782, 275.705468, 245.950970},
  {63.289348, 56.710210, 355.824983, 277.856284, 259.836564, 276.837038, 245.873716},
  {63.298116, 56.701440, 355.285036, 279.118439, 259.738452, 277.967656, 245.785829},
  {63.305563, 56.693991, 354.743870, 280.379760, 259.628423, 279.097246, 245.687292},
  {63.311683, 56.687869, 354.201701, 281.640107, 259.506463, 280.225732, 245.578091},
  {63.316474, 56.683077, 353.658748, 282.899337, 259.372563, 281.353039, 245.458220},
  {63.319933, 56.679617, 353.115229, 284.157310, 259.226722, 282.479089, 245.327673},
  {63.322064, 56.677486, 352.571361, 285.413885, 259.068945, 283.603807, 245.186452},
  {63.322871, 56.676678, 352.027360, 286.668921, 258.899243, 284.727118, 245.034561},
  {63.322363, 56.677186, 351.483443, 287.922279, 258.717635, 285.848947, 244.872012},
  {63.320551, 56.678999, 350.939823, 289.173822, 258.524146, 286.969218, 244.698817},
  {63.317447, 56.682104, 350.396710, 290.423412, 258.318806, 288.087857, 244.514995},
  {63.313067, 56.686485, 349.854315, 291.670914, 258.101652, 289.204791, 244.320570},
  {63.307431, 56.692123, 349.312843, 292.916196, 257.872730, 290.319947, 244.115568},
  {63.300558, 56.698997, 348.772496, 294.159126, 257.632087, 291.433253, 243.900022},
  {63.292473, 56.707084, 348.233472, 295.399576, 257.379779, 292.544638, 243.673966},
  {63.283201, 56.716359, 347.695967, 296.637419, 257.115868, 293.654031, 243.437440},
  {63.272769, 56.726794, 347.160170, 297.872533, 256.840419, 294.761364, 243.190488},
  {63.261206, 56.738360, 346.626266, 299.104796, 256.553505, 295.866569, 242.933157},
  {63.248544, 56.751025, 346.094434, 300.334091, 256.255203, 296.969579, 242.665499},
  {63.234816, 56.764757, 345.564851, 301.560305, 255.945594, 298.070328, 242.387568},
  {63.220056, 56.779521, 345.037684, 302.783325, 255.624765, 299.168754, 242.099423},
  {63.204299, 56.795282, 344.513097, 304.003045, 255.292807, 300.264792, 241.801126},
  {63.187583, 56.812002, 343.991247, 305.219361, 254.949815, 301.358383, 241.492741},
  {63.169945, 56.829645, 343.472285, 306.432173, 254.595888, 302.449466, 241.174337},
  {63.151424, 56.848170, 342.956355, 307.641385, 254.231130, 303.537983, 240.845985},
  {63.132060, 56.867540, 342.443595, 308.846903, 253.855646, 304.623880, 240.507760},
  {63.111891, 56.887714, 341.934137, 310.048640, 253.469547, 305.707099, 240.159738},
  {63.090959, 56.908651, 341.428107, 311.246511, 253.072945, 306.787590, 239.801998},
  {63.069304, 56.930312, 340.925622, 312.440435, 252.665955, 307.865301, 239.434623},
  {63.046966, 56.952655, 340.426794, 313.630335, 252.248697, 308.940182, 239.057697},
  {63.023986, 56.975641, 339.931729, 314.816139, 251.821289, 310.012185, 238.671307},
  {63.000404, 56.999229, 339.440525, 315.997779, 251.383854, 311.081266, 238.275540},
  {62.976261, 57.023378, 338.953273, 317.175189, 250.936518, 312.147380, 237.870488},
  {62.951595, 57.048049, 338.470060, 318.348308, 250.479405, 313.210485, 237.456242},
  {62.926447, 57.073204, 337.990964, 319.517081, 250.012644, 314.270541, 237.032897},
  {62.900854, 57.098803, 337.516058, 320.681453, 249.536362, 315.327510, 236.600547},
  {62.836862, 57.090275, 337.045692, 321.840676, 249.050986, 316.380719, 236.159559},
  {62.525121, 56.856971, 336.581747, 322.990172, 248.558629, 317.426008, 235.711836},
  {61.794688, 56.242367, 336.127282, 324.122303, 248.062934, 318.456416, 235.260679},
  {60.646309, 55.245713, 335.685236, 325.229554, 247.567774, 319.465066, 234.809613},
  {59.080718, 53.866276, 335.258427, 326.304534, 247.077232, 320.445168, 234.362366},
  {699.639574, 700.517948, 153.674111, 320.532018, 234.330614, 326.740685, 246.878620},
  {79.514270, 25.734762, 149.271099, 319.133657, 235.087976, 326.287330, 247.122301},
  {88.605523, 12.620575, 143.048492, 317.659677, 236.071722, 326.075621, 247.259746},
  {104.689603, 7.525657, 133.854177, 316.088855, 237.456093, 326.184331, 247.155647},
  {129.980610, 36.932377, 120.159803, 314.517943, 239.527378, 326.622728, 246.561166},
  {154.643320, 65.745180, 102.045462, 313.399917, 242.411097, 327.091669, 245.332726},
  {150.471491, 65.542002, 84.293626, 313.236907, 245.416109, 327.167531, 244.024083},
  {118.479770, 37.376754, 71.510218, 313.739205, 247.731855, 327.016528, 243.291958},
  {87.165307, 9.955387, 63.554578, 314.409146, 249.341294, 326.944172, 243.106463},
  {65.592219, 7.662284, 58.811525, 315.043579, 250.489524, 327.020137, 243.239555},
  {51.136666, 18.131690, 56.109773, 315.594941, 251.350908, 327.216445, 243.544459},
  {40.766698, 24.506558, 54.778831, 316.058236, 252.021824, 327.495282, 243.947545},
  {32.668226, 28.606407, 54.446366, 316.437219, 252.554043, 327.827221, 244.413536},
  {25.845016, 31.429554, 54.903468, 316.736608, 252.975412, 328.191191, 244.926032},
  {19.749661, 33.524206, 56.030949, 316.960976, 253.300495, 328.571729, 245.478064},
  {14.089344, 35.183525, 57.757597, 317.115311, 253.536258, 328.956491, 246.067225},
  {8.736878, 36.534891, 60.033047, 317.205991, 253.685625, 329.334382, 246.692619},
  {3.694956, 37.575794, 62.806507, 317.241731, 253.750307, 329.694287, 247.352350},
  {0.919282, 38.188937, 66.007957, 317.234146, 253.733558, 330.024573, 248.041022},
  {4.892963, 38.162405, 69.532640, 317.197429, 253.642848, 330.313631, 248.747416},
  {7.966672, 37.236294, 73.233196, 317.146804, 253.491772, 330.551619, 249.453092},
  {9.911082, 35.181430, 76.924706, 317.095933, 253.300189, 330.732963, 250.132950},
  {10.614436, 31.885891, 80.403939, 317.054309, 253.092021, 330.858414, 250.758207},
  {10.130305, 27.402961, 83.476447, 317.025957, 252.891408, 330.935310, 251.300845},
  {8.656663, 21.930334, 85.980226, 317.010072, 252.719005, 330.975631, 251.737595},
  {6.515070, 15.861928, 87.811886, 317.003024, 252.588894, 330.992816, 252.054367},
  {4.270586, 10.162791, 88.993295, 317.000645, 252.503516, 330.998484, 252.257544},
  {2.396516, 5.633863, 89.650594, 317.000078, 252.455589, 330.999818, 252.370213},
  {1.033700, 2.416117, 89.932966, 317.000003, 252.434915, 330.999993, 252.418536},
  {0.245752, 0.573225, 90.000000, 317.000000, 252.430000, 331.000000, 252.430000},
  {0.000000, 0.000000, 90.000000, 317.000000, 252.430000, 331.000000, 252.430000}};
}}
