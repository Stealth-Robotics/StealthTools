/* Red25 Feed scale from side aud */
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
       kNumPoints =286;
       kPoints = new double[][]{
  {0.400000, 0.400000, 359.998977, 16.002885, 215.250000, 16.002634, 201.250000},
  {0.836240, 0.763760, 359.993044, 16.019609, 215.249999, 16.017910, 201.249999},
  {1.881486, 1.718514, 359.979705, 16.057239, 215.249990, 16.052280, 201.249991},
  {3.344677, 3.055323, 359.956021, 16.124133, 215.249952, 16.113386, 201.249956},
  {5.225578, 4.774422, 359.919093, 16.228644, 215.249838, 16.208875, 201.249852},
  {7.314826, 6.685173, 359.867555, 16.374940, 215.249566, 16.342578, 201.249603},
  {9.403091, 8.596908, 359.801568, 16.563001, 215.249023, 16.514515, 201.249107},
  {11.490091, 10.509907, 359.721339, 16.792801, 215.248066, 16.724712, 201.248232},
  {13.575543, 12.424454, 359.627121, 17.064308, 215.246522, 16.973197, 201.246818},
  {15.659164, 14.340832, 359.519214, 17.377482, 215.244189, 17.260006, 201.244681},
  {17.740668, 16.259325, 359.397965, 17.732280, 215.240835, 17.585177, 201.241608},
  {19.819772, 18.180219, 359.263765, 18.128648, 215.236205, 17.948757, 201.237361},
  {21.896190, 20.103798, 359.117056, 18.566528, 215.230017, 18.350793, 201.231679},
  {23.969636, 22.030349, 358.958323, 19.045853, 215.221964, 18.791338, 201.224278},
  {26.039824, 23.960158, 358.788100, 19.566549, 215.211721, 19.270448, 201.214852},
  {28.106468, 25.893509, 358.606967, 20.128533, 215.198941, 19.788184, 201.203079},
  {30.169285, 27.830688, 358.415550, 20.731715, 215.183262, 20.344610, 201.188615},
  {32.227989, 29.771979, 358.214523, 21.375996, 215.164306, 20.939792, 201.171103},
  {34.282299, 31.717664, 358.004605, 22.061269, 215.141684, 21.573800, 201.150173},
  {36.331935, 33.668023, 357.786560, 22.787417, 215.114996, 22.246706, 201.125442},
  {38.376621, 35.623331, 357.561200, 23.554317, 215.083839, 22.958586, 201.096519},
  {40.416083, 37.583864, 357.329380, 24.361835, 215.047802, 23.709516, 201.063008},
  {42.450052, 39.549889, 357.091998, 25.209830, 215.006479, 24.499576, 201.024507},
  {44.478263, 41.521673, 356.849998, 26.098152, 214.959465, 25.328849, 200.980617},
  {46.500457, 43.499474, 356.604364, 27.026643, 214.906360, 26.197419, 200.930939},
  {48.516379, 45.483547, 356.356123, 27.995140, 214.846777, 27.105373, 200.875080},
  {50.525782, 47.474140, 356.106342, 29.003469, 214.780343, 28.052802, 200.812658},
  {52.528424, 49.471495, 355.856129, 30.051454, 214.706701, 29.039798, 200.743301},
  {54.524068, 51.475849, 355.606629, 31.138909, 214.625517, 30.066457, 200.666654},
  {56.512483, 53.487431, 355.359025, 32.265646, 214.536483, 31.132882, 200.582385},
  {58.288338, 55.311576, 355.115373, 33.427386, 214.439668, 32.235292, 200.490513},
  {59.647656, 56.752261, 354.878382, 34.615792, 214.335609, 33.366010, 200.391505},
  {60.592469, 57.807453, 354.650425, 35.822583, 214.225003, 34.517334, 200.285982},
  {61.124901, 58.475028, 354.433530, 37.039537, 214.108701, 35.681530, 200.174720},
  {61.247126, 58.752811, 354.229367, 38.258487, 213.987685, 36.850838, 200.058632},
  {61.165081, 58.834863, 354.038636, 39.475380, 213.862638, 38.021371, 199.938348},
  {61.083330, 58.916622, 353.861288, 40.690240, 213.733855, 39.193138, 199.814132},
  {61.001865, 58.998094, 353.697278, 41.903091, 213.601630, 40.366149, 199.686249},
  {60.920673, 59.079292, 353.546559, 43.113961, 213.466255, 41.540419, 199.554965},
  {60.839742, 59.160229, 353.409088, 44.322878, 213.328019, 42.715964, 199.420545},
  {60.759056, 59.240920, 353.284827, 45.529873, 213.187209, 43.892801, 199.283253},
  {60.678598, 59.321383, 353.173738, 46.734979, 213.044110, 45.070951, 199.143354},
  {60.598347, 59.401638, 353.075786, 47.938228, 212.899006, 46.250438, 199.001115},
  {60.518284, 59.481705, 352.990941, 49.139654, 212.752176, 47.431287, 198.856800},
  {60.438386, 59.561606, 352.919175, 50.339294, 212.603901, 48.613522, 198.710675},
  {60.358629, 59.641366, 352.860467, 51.537180, 212.454457, 49.797174, 198.563007},
  {60.278989, 59.721008, 352.814795, 52.733350, 212.304121, 50.982271, 198.414062},
  {60.199441, 59.800557, 352.782146, 53.927838, 212.153167, 52.168844, 198.264108},
  {60.119959, 59.880041, 352.762509, 55.120680, 212.001867, 53.356926, 198.113413},
  {60.040517, 59.959483, 352.755876, 56.311911, 211.850495, 54.546550, 197.962245},
  {59.961088, 60.038912, 352.762246, 57.501566, 211.699320, 55.737749, 197.810873},
  {59.881646, 60.118354, 352.781621, 58.689679, 211.548612, 56.930558, 197.659570},
  {59.802164, 60.197835, 352.814007, 59.876282, 211.398640, 58.125013, 197.508606},
  {59.722616, 60.277381, 352.859415, 61.061409, 211.249673, 59.321148, 197.358255},
  {59.642976, 60.357019, 352.917860, 62.245089, 211.101976, 60.518999, 197.208790},
  {59.563219, 60.436773, 352.989362, 63.427354, 210.955818, 61.718603, 197.060489},
  {59.483320, 60.516669, 353.073942, 64.608229, 210.811463, 62.919993, 196.913627},
  {59.403257, 60.596728, 353.171629, 65.787744, 210.669179, 64.123205, 196.768485},
  {59.323007, 60.676975, 353.282453, 66.965921, 210.529231, 65.328273, 196.625343},
  {59.242548, 60.757429, 353.406448, 68.142785, 210.391884, 66.535230, 196.484484},
  {59.161861, 60.838110, 353.543651, 69.318355, 210.257403, 67.744108, 196.346193},
  {59.080930, 60.919036, 353.694102, 70.492650, 210.126053, 68.954937, 196.210757},
  {58.999737, 61.000222, 353.857844, 71.665687, 209.998099, 70.167748, 196.078466},
  {58.918270, 61.081682, 354.034921, 72.837479, 209.873806, 71.382567, 195.949611},
  {58.836518, 61.163427, 354.225382, 74.008037, 209.753440, 72.599419, 195.824485},
  {58.754471, 61.245466, 354.429272, 75.177370, 209.637265, 73.818328, 195.703385},
  {58.672125, 61.327803, 354.646643, 76.345482, 209.525546, 75.039312, 195.586610},
  {58.589477, 61.410441, 354.877542, 77.512376, 209.418548, 76.262390, 195.474462},
  {58.506529, 61.493380, 355.122019, 78.678052, 209.316536, 77.487575, 195.367243},
  {58.423285, 61.576614, 355.380123, 79.842504, 209.219775, 78.714878, 195.265262},
  {58.339753, 61.660134, 355.651900, 81.005726, 209.128531, 79.944304, 195.168825},
  {58.255947, 61.743929, 355.937395, 82.167706, 209.043067, 81.175856, 195.078246},
  {58.171885, 61.827979, 356.236651, 83.328430, 208.963649, 82.409531, 194.993838},
  {58.087588, 61.912262, 356.549705, 84.487879, 208.890540, 83.645323, 194.915916},
  {58.003085, 61.996752, 356.876592, 85.646031, 208.824004, 84.883216, 194.844801},
  {57.918408, 62.081415, 357.217339, 86.802860, 208.764303, 86.123195, 194.780811},
  {57.833596, 62.166212, 357.571969, 87.958335, 208.711701, 87.365233, 194.724270},
  {57.748694, 62.251099, 357.940497, 89.112422, 208.666458, 88.609299, 194.675502},
  {57.663752, 62.336026, 358.322928, 90.265084, 208.628835, 89.855356, 194.634832},
  {57.578827, 62.420934, 358.719261, 91.416276, 208.599090, 91.103358, 194.602587},
  {57.493984, 62.505760, 359.129481, 92.565953, 208.577480, 92.353253, 194.579096},
  {57.409293, 62.590433, 359.553564, 93.714062, 208.564261, 93.604979, 194.564686},
  {57.324832, 62.674876, 359.991472, 94.860550, 208.559685, 94.858466, 194.559685},
  {57.240687, 62.759003, 0.443154, 96.005355, 208.564003, 96.113637, 194.564421},
  {57.156949, 62.842722, 0.908542, 97.148415, 208.577463, 97.370404, 194.579223},
  {57.073717, 62.925934, 1.387554, 98.289661, 208.600309, 98.628671, 194.604415},
  {56.991099, 63.008532, 1.880089, 99.429020, 208.632784, 99.888330, 194.640321},
  {56.909207, 63.090403, 2.386028, 100.566416, 208.675124, 101.149265, 194.687262},
  {56.828163, 63.171426, 2.905233, 101.701769, 208.727564, 102.411348, 194.745557},
  {56.748094, 63.251475, 3.437544, 102.834994, 208.790330, 103.674441, 194.815520},
  {56.669132, 63.330415, 3.982780, 103.966003, 208.863648, 104.938396, 194.897459},
  {56.591417, 63.408109, 4.540736, 105.094703, 208.947736, 106.203053, 194.991678},
  {56.515094, 63.484410, 5.111184, 106.221000, 209.042806, 107.468242, 195.098474},
  {56.440312, 63.559171, 5.693873, 107.344794, 209.149065, 108.733781, 195.218138},
  {56.367223, 63.632239, 6.288526, 108.465983, 209.266712, 109.999476, 195.350951},
  {56.295984, 63.703456, 6.894838, 109.584462, 209.395939, 111.265125, 195.497185},
  {56.226754, 63.772665, 7.512483, 110.700123, 209.536933, 112.530514, 195.657103},
  {56.159691, 63.839707, 8.141104, 111.812856, 209.689869, 113.795416, 195.830957},
  {56.094957, 63.904421, 8.780320, 112.922548, 209.854918, 115.059598, 196.018986},
  {56.032708, 63.966650, 9.429726, 114.029085, 210.032238, 116.322814, 196.221416},
  {55.973102, 64.026236, 10.088887, 115.132349, 210.221981, 117.584810, 196.438461},
  {55.916291, 64.083029, 10.757348, 116.232224, 210.424289, 118.845324, 196.670318},
  {55.862423, 64.136878, 11.434625, 117.328590, 210.639292, 120.104086, 196.917170},
  {55.811640, 64.187644, 12.120214, 118.421326, 210.867113, 121.360816, 197.179184},
  {55.764075, 64.235193, 12.813589, 119.510313, 211.107862, 122.615230, 197.456507},
  {55.719854, 64.279398, 13.514202, 120.595430, 211.361640, 123.867039, 197.749272},
  {55.679093, 64.320145, 14.221486, 121.676555, 211.628536, 125.115947, 198.057590},
  {55.641894, 64.357331, 14.934859, 122.753567, 211.908629, 126.361657, 198.381556},
  {55.608349, 64.390864, 15.653722, 123.826348, 212.201984, 127.603867, 198.721244},
  {55.578537, 64.420665, 16.377465, 124.894777, 212.508657, 128.842275, 199.076708},
  {55.552521, 64.446672, 17.105466, 125.958737, 212.828693, 130.076578, 199.447983},
  {55.530349, 64.468835, 17.837096, 127.018111, 213.162122, 131.306474, 199.835085},
  {55.512056, 64.487122, 18.571720, 128.072783, 213.508966, 132.531664, 200.238006},
  {55.497659, 64.501514, 19.308701, 129.122642, 213.869233, 133.751850, 200.656722},
  {55.487158, 64.512011, 20.047400, 130.167577, 214.242919, 134.966741, 201.091188},
  {55.480539, 64.518628, 20.787183, 131.207479, 214.630011, 136.176048, 201.541340},
  {55.477770, 64.521395, 21.527419, 132.242243, 215.030483, 137.379493, 202.007093},
  {55.478805, 64.520361, 22.267485, 133.271766, 215.444296, 138.576800, 202.488347},
  {55.483582, 64.515585, 23.006770, 134.295949, 215.871403, 139.767707, 202.984981},
  {55.492024, 64.507146, 23.744673, 135.314695, 216.311745, 140.951957, 203.496860},
  {55.504041, 64.495134, 24.480609, 136.327913, 216.765252, 142.129307, 204.023830},
  {57.319039, 62.517196, 24.906086, 137.368511, 217.246236, 143.264361, 204.548246},
  {61.982424, 58.017416, 24.581545, 138.494343, 217.765109, 144.318174, 205.033927},
  {61.997954, 58.001883, 24.254462, 139.623391, 218.277701, 145.374449, 205.513480},
  {62.013490, 57.986345, 23.924835, 140.755641, 218.783940, 146.433171, 205.986845},
  {62.029025, 57.970807, 23.592665, 141.891081, 219.283757, 147.494325, 206.453962},
  {62.044554, 57.955275, 23.257952, 143.029695, 219.777082, 148.557894, 206.914772},
  {62.060071, 57.939756, 22.920699, 144.171467, 220.263843, 149.623861, 207.369216},
  {62.075568, 57.924257, 22.580909, 145.316382, 220.743969, 150.692210, 207.817234},
  {62.091038, 57.908784, 22.238587, 146.464420, 221.217389, 151.762920, 208.258766},
  {62.106475, 57.893344, 21.893737, 147.615563, 221.684033, 152.835972, 208.693754},
  {62.121871, 57.877946, 21.546366, 148.769789, 222.143828, 153.911346, 209.122139},
  {62.137218, 57.862595, 21.196483, 149.927078, 222.596704, 154.989021, 209.543860},
  {62.152510, 57.847301, 20.844096, 151.087406, 223.042588, 156.068975, 209.958859},
  {62.167737, 57.832071, 20.489217, 152.250749, 223.481410, 157.151184, 210.367076},
  {62.182893, 57.816913, 20.131856, 153.417081, 223.913097, 158.235626, 210.768455},
  {62.197968, 57.801835, 19.772027, 154.586375, 224.337580, 159.322275, 211.162935},
  {62.212954, 57.786846, 19.409744, 155.758604, 224.754786, 160.411106, 211.550460},
  {62.227844, 57.771954, 19.045024, 156.933737, 225.164645, 161.502092, 211.930971},
  {62.242627, 57.757168, 18.677884, 158.111743, 225.567086, 162.595206, 212.304411},
  {62.257295, 57.742497, 18.308342, 159.292591, 225.962039, 163.690420, 212.670722},
  {62.271838, 57.727951, 17.936419, 160.476245, 226.349433, 164.787705, 213.029849},
  {62.286249, 57.713538, 17.562137, 161.662672, 226.729199, 165.887031, 213.381735},
  {62.300516, 57.699268, 17.185519, 162.851834, 227.101268, 166.988367, 213.726325},
  {62.314632, 57.685149, 16.806590, 164.043694, 227.465570, 168.091681, 214.063563},
  {62.328585, 57.671193, 16.425376, 165.238211, 227.822038, 169.196940, 214.393394},
  {62.342367, 57.657409, 16.041907, 166.435345, 228.170604, 170.304110, 214.715766},
  {62.355968, 57.643805, 15.656210, 167.635054, 228.511201, 171.413158, 215.030625},
  {62.369377, 57.630393, 15.268318, 168.837293, 228.843763, 172.524048, 215.337918},
  {62.382586, 57.617183, 14.878264, 170.042017, 229.168224, 173.636743, 215.637594},
  {62.395583, 57.604183, 14.486082, 171.249179, 229.484519, 174.751206, 215.929601},
  {62.408359, 57.591404, 14.091808, 172.458731, 229.792586, 175.867400, 216.213890},
  {62.420904, 57.578857, 13.695480, 173.670623, 230.092360, 176.985285, 216.490410},
  {62.433207, 57.566551, 13.297138, 174.884805, 230.383780, 178.104821, 216.759115},
  {62.445260, 57.554496, 12.896823, 176.101223, 230.666785, 179.225968, 217.019955},
  {62.457051, 57.542702, 12.494577, 177.319823, 230.941316, 180.348684, 217.272885},
  {62.468571, 57.531180, 12.090445, 178.540550, 231.207313, 181.472927, 217.517859},
  {62.479811, 57.519938, 11.684473, 179.763347, 231.464721, 182.598654, 217.754832},
  {62.490759, 57.508987, 11.276709, 180.988155, 231.713482, 183.725820, 217.983763},
  {62.501407, 57.498337, 10.867201, 182.214916, 231.953542, 184.854382, 218.204607},
  {62.511746, 57.487997, 10.456001, 183.443567, 232.184849, 185.984293, 218.417325},
  {62.521765, 57.477976, 10.043160, 184.674047, 232.407349, 187.115507, 218.621876},
  {62.531455, 57.468283, 9.628733, 185.906293, 232.620994, 188.247978, 218.818222},
  {62.540808, 57.458929, 9.212775, 187.140239, 232.825734, 189.381657, 219.006326},
  {62.549814, 57.449920, 8.795342, 188.375819, 233.021523, 190.516496, 219.186152},
  {62.558466, 57.441267, 8.376492, 189.612966, 233.208316, 191.652446, 219.357665},
  {62.566754, 57.432977, 7.956286, 190.851612, 233.386068, 192.789458, 219.520832},
  {62.574670, 57.425059, 7.534784, 192.091687, 233.554739, 193.927480, 219.675622},
  {62.582208, 57.417520, 7.112047, 193.333121, 233.714288, 195.066463, 219.822005},
  {62.589358, 57.410368, 6.688140, 194.575841, 233.864677, 196.206353, 219.959951},
  {62.596114, 57.403610, 6.263127, 195.819776, 234.005872, 197.347101, 220.089433},
  {62.602470, 57.397254, 5.837073, 197.064851, 234.137837, 198.488651, 220.210426},
  {62.608418, 57.391304, 5.410046, 198.310993, 234.260541, 199.630953, 220.322905},
  {62.613953, 57.385768, 4.982112, 199.558126, 234.373955, 200.773952, 220.426849},
  {62.619068, 57.380652, 4.553341, 200.806174, 234.478050, 201.917594, 220.522236},
  {62.623759, 57.375960, 4.123802, 202.055060, 234.572801, 203.061825, 220.609047},
  {62.628020, 57.371698, 3.693565, 203.304706, 234.658185, 204.206590, 220.687265},
  {62.631847, 57.367871, 3.262702, 204.555036, 234.734180, 205.351834, 220.756873},
  {62.635235, 57.364481, 2.831284, 205.805970, 234.800769, 206.497501, 220.817858},
  {62.638182, 57.361534, 2.399383, 207.057428, 234.857933, 207.643537, 220.870208},
  {62.640684, 57.359031, 1.967073, 208.309333, 234.905660, 208.789885, 220.913910},
  {62.642739, 57.356976, 1.534426, 209.561603, 234.943938, 209.936489, 220.948958},
  {62.644343, 57.355371, 1.101517, 210.814158, 234.972756, 211.083293, 220.975343},
  {62.645496, 57.354218, 0.668419, 212.066918, 234.992107, 212.230240, 220.993060},
  {62.646196, 57.353518, 0.235207, 213.319803, 235.001987, 213.377275, 221.002105},
  {62.646442, 57.353272, 359.801954, 214.572732, 235.002394, 214.524340, 221.002477},
  {62.646235, 57.353479, 359.368735, 215.825624, 234.993326, 215.671380, 220.994176},
  {62.645574, 57.354140, 358.935624, 217.078398, 234.974787, 216.818337, 220.977202},
  {62.644460, 57.355255, 358.502695, 218.330974, 234.946780, 217.965156, 220.951561},
  {62.642894, 57.356821, 358.070023, 219.583272, 234.909314, 219.111779, 220.917256},
  {62.640878, 57.358837, 357.637682, 220.835211, 234.862397, 220.258151, 220.874295},
  {62.638414, 57.361301, 357.205743, 222.086711, 234.806041, 221.404216, 220.822687},
  {62.635505, 57.364211, 356.774281, 223.337693, 234.740261, 222.549917, 220.762442},
  {62.632154, 57.367563, 356.343367, 224.588077, 234.665071, 223.695199, 220.693573},
  {62.628365, 57.371353, 355.913074, 225.837786, 234.580492, 224.840008, 220.616093},
  {62.624141, 57.375578, 355.483472, 227.086740, 234.486544, 225.984286, 220.530018},
  {62.619487, 57.380233, 355.054632, 228.334863, 234.383249, 227.127981, 220.435366},
  {62.614408, 57.385313, 354.626624, 229.582077, 234.270634, 228.271037, 220.332156},
  {62.608909, 57.390813, 354.199516, 230.828307, 234.148725, 229.413401, 220.220407},
  {62.602996, 57.396727, 353.773376, 232.073476, 234.017553, 230.555018, 220.100144},
  {62.596675, 57.403049, 353.348271, 233.317512, 233.877149, 231.695836, 219.971389},
  {62.589953, 57.409773, 352.924267, 234.560339, 233.727546, 232.835803, 219.834168},
  {62.582836, 57.416892, 352.501427, 235.801886, 233.568781, 233.974865, 219.688508},
  {62.575332, 57.424398, 352.079817, 237.042080, 233.400892, 235.112972, 219.534438},
  {62.567447, 57.432284, 351.659497, 238.280851, 233.223917, 236.250072, 219.371988},
  {62.559191, 57.440542, 351.240529, 239.518130, 233.037898, 237.386115, 219.201189},
  {62.550570, 57.449164, 350.822972, 240.753847, 232.842879, 238.521052, 219.022075},
  {62.541594, 57.458142, 350.406885, 241.987936, 232.638905, 239.654832, 218.834680},
  {62.532271, 57.467468, 349.992324, 243.220330, 232.426022, 240.787408, 218.639039},
  {62.522609, 57.477131, 349.579345, 244.450964, 232.204279, 241.918732, 218.435190},
  {62.512618, 57.487125, 349.168002, 245.679775, 231.973726, 243.048757, 218.223171},
  {62.502306, 57.497438, 348.758347, 246.906700, 231.734414, 244.177436, 218.003022},
  {62.491685, 57.508062, 348.350432, 248.131678, 231.486397, 245.304724, 217.774784},
  {62.480761, 57.518987, 347.944304, 249.354650, 231.229729, 246.430576, 217.538499},
  {62.469547, 57.530204, 347.540012, 250.575557, 230.964466, 247.554948, 217.294209},
  {62.458050, 57.541703, 347.137603, 251.794341, 230.690665, 248.677797, 217.041960},
  {62.446282, 57.553474, 346.737121, 253.010949, 230.408384, 249.799080, 216.781796},
  {62.434251, 57.565507, 346.338608, 254.225324, 230.117683, 250.918756, 216.513765},
  {62.421969, 57.577792, 345.942106, 255.437415, 229.818623, 252.036784, 216.237913},
  {62.409444, 57.590319, 345.547654, 256.647170, 229.511266, 253.153124, 215.954288},
  {62.396688, 57.603078, 345.155291, 257.854539, 229.195674, 254.267737, 215.662941},
  {62.383709, 57.616059, 344.765053, 259.059474, 228.871911, 255.380585, 215.363921},
  {62.370518, 57.629252, 344.376974, 260.261927, 228.540041, 256.491631, 215.057280},
  {62.357126, 57.642647, 343.991088, 261.461855, 228.200131, 257.600839, 214.743068},
  {62.343541, 57.656235, 343.607426, 262.659211, 227.852246, 258.708172, 214.421338},
  {62.329774, 57.670004, 343.226018, 263.853955, 227.496453, 259.813596, 214.092144},
  {62.315835, 57.683946, 342.846892, 265.046044, 227.132820, 260.917078, 213.755539},
  {62.301733, 57.698051, 342.470075, 266.235439, 226.761414, 262.018584, 213.411578},
  {62.287478, 57.712308, 342.095591, 267.422102, 226.382306, 263.118084, 213.060315},
  {62.273080, 57.726709, 341.723465, 268.605995, 225.995563, 264.215545, 212.701807},
  {62.258547, 57.741245, 341.353718, 269.787085, 225.601256, 265.310938, 212.336110},
  {62.243889, 57.755905, 340.986371, 270.965336, 225.199455, 266.404234, 211.963279},
  {62.229116, 57.770681, 340.621443, 272.140717, 224.790229, 267.495404, 211.583373},
  {62.214235, 57.785564, 340.258951, 273.313196, 224.373651, 268.584420, 211.196447},
  {62.199257, 57.800546, 339.898911, 274.482743, 223.949789, 269.671257, 210.802561},
  {62.184189, 57.815616, 339.541337, 275.649330, 223.518717, 270.755889, 210.401772},
  {62.169040, 57.830768, 339.186244, 276.812931, 223.080504, 271.838291, 209.994139},
  {62.153818, 57.845992, 338.833644, 277.973518, 222.635223, 272.918439, 209.579719},
  {62.138532, 57.861281, 338.483545, 279.131068, 222.182944, 273.996311, 209.158572},
  {62.123189, 57.876627, 338.135959, 280.285559, 221.723739, 275.071883, 208.730757},
  {62.107797, 57.892022, 337.790893, 281.436967, 221.257680, 276.145135, 208.296333},
  {62.092363, 57.907458, 337.448353, 282.585272, 220.784838, 277.216047, 207.855360},
  {62.076896, 57.922928, 337.108345, 283.730455, 220.305285, 278.284598, 207.407896},
  {62.061401, 57.938426, 336.770875, 284.872498, 219.819090, 279.350771, 206.954001},
  {62.045886, 57.953943, 336.435944, 286.011383, 219.326326, 280.414546, 206.493734},
  {62.030358, 57.969474, 336.103556, 287.147096, 218.827063, 281.475908, 206.027156},
  {62.014822, 57.985012, 335.773711, 288.279620, 218.321372, 282.534839, 205.554324},
  {61.999286, 58.000551, 335.446410, 289.408943, 217.809322, 283.591325, 205.075300},
  {61.983756, 58.016084, 335.121651, 290.535052, 217.290985, 284.645349, 204.590142},
  {83.444546, 36.513941, 331.279244, 292.031486, 216.552147, 285.303909, 204.274537},
  {120.000982, 0.668135, 321.416914, 294.035642, 215.231749, 285.304558, 204.287884},
  {186.202908, 67.876085, 300.506111, 296.498629, 212.438488, 284.436578, 205.331664},
  {273.786833, 158.352301, 264.547920, 297.695738, 207.095210, 283.759074, 208.425395},
  {204.708376, 90.568510, 240.197994, 296.425657, 203.203025, 284.277185, 210.161086},
  {118.027528, 6.344444, 230.007871, 295.063812, 201.274924, 284.337953, 210.272477},
  {79.295761, 28.762811, 225.870712, 293.996925, 200.101518, 283.948138, 209.849435},
  {58.391526, 45.688453, 224.830931, 293.173606, 199.273283, 283.303366, 209.201946},
  {698.972553, 700.218990, 40.102621, 283.979332, 209.803732, 292.997552, 199.095245},
  {100.831937, 4.843996, 31.444848, 285.613948, 210.984775, 292.917434, 199.040777},
  {106.379837, 14.413575, 21.545488, 287.517322, 211.935477, 292.658679, 198.913708},
  {105.800445, 17.834190, 11.412656, 289.546399, 212.535761, 292.316633, 198.812576},
  {97.586902, 13.594522, 2.302756, 291.484335, 212.767457, 292.046853, 198.778762},
  {84.632423, 4.606423, 354.993582, 293.176468, 212.725670, 291.954725, 198.779081},
  {70.981533, 5.072394, 349.596797, 294.583053, 212.533657, 292.055015, 198.763797},
  {58.849869, 13.218944, 345.861182, 295.732872, 212.282183, 292.313064, 198.706289},
  {48.710519, 19.366011, 343.459122, 296.672103, 212.023470, 292.686312, 198.602834},
  {40.301445, 23.778570, 342.106674, 297.441780, 211.784139, 293.140339, 198.461317},
  {33.223817, 26.857477, 341.585580, 298.072988, 211.576522, 293.650558, 198.293370},
  {27.146726, 28.934761, 341.731933, 298.588185, 211.405202, 294.199699, 198.110797},
  {21.844088, 30.237104, 342.418915, 299.003735, 211.270357, 294.774963, 197.924291},
  {17.181382, 30.899355, 343.541764, 299.332215, 211.169456, 295.365786, 197.743085},
  {13.092378, 30.987928, 345.006575, 299.584187, 211.098221, 295.962272, 197.574844},
  {9.555848, 30.524162, 346.722920, 299.769463, 211.051334, 296.554217, 197.425543},
  {6.573702, 29.506194, 348.600054, 299.897864, 211.023074, 297.130674, 197.299275},
  {4.151226, 27.928740, 350.546364, 299.979495, 211.007926, 297.680003, 197.198063},
  {2.281233, 25.798955, 352.471406, 300.024607, 211.001105, 298.190313, 197.121791},
  {0.934534, 23.145965, 354.289513, 300.043167, 210.998897, 298.650140, 197.068373},
  {0.058089, 20.022757, 355.923701, 300.044321, 210.998767, 299.049133, 197.034183},
  {0.420248, 16.501363, 357.308787, 300.035930, 210.999247, 299.378584, 197.014688},
  {0.583051, 12.664377, 358.393120, 300.024277, 210.999678, 299.631693, 197.005183},
  {0.528493, 8.801788, 359.156819, 300.013709, 210.999902, 299.807689, 197.001418},
  {0.373697, 5.430737, 359.631920, 300.006236, 210.999980, 299.916298, 197.000269},
  {0.210186, 2.850937, 359.882477, 300.002032, 210.999998, 299.973316, 197.000027},
  {0.084293, 1.108745, 359.980128, 300.000346, 211.000000, 299.995491, 197.000001},
  {0.017314, 0.225465, 360.000000, 300.000000, 211.000000, 300.000000, 197.000000},
  {0.000000, 0.000000, 360.000000, 300.000000, 211.000000, 300.000000, 197.000000}};
}}