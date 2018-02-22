/* Red15 Feed scale from side aud */
//----------------------------------------------------------------------------
//
//  $Workfile: Red15Path60InPerSec.java$
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

public class Red15Path60InPerSec extends Path {
    public Red15Path60InPerSec() {
       kNumPoints =317;
       kPoints = new double[][]{
  {0.400000, 0.400000, 359.998722, 16.002157, 314.750000, 16.001845, 300.750000},
  {0.862444, 0.737556, 359.988499, 16.019406, 314.749998, 16.016596, 300.749998},
  {1.940564, 1.659436, 359.965489, 16.058218, 314.749982, 16.049785, 300.749985},
  {3.450113, 2.949887, 359.924545, 16.127220, 314.749916, 16.108783, 300.749928},
  {5.391368, 4.608632, 359.860477, 16.235047, 314.749714, 16.200955, 300.749755},
  {7.549075, 6.450924, 359.770592, 16.386028, 314.749228, 16.329973, 300.749340},
  {9.707942, 8.292055, 359.654700, 16.580184, 314.748254, 16.495812, 300.748509},
  {11.868301, 10.131694, 359.512557, 16.817544, 314.746530, 16.698440, 300.747036},
  {14.030483, 11.969508, 359.343863, 17.098140, 314.743729, 16.937819, 300.744647},
  {16.194819, 13.805166, 359.148268, 17.422008, 314.739468, 17.213898, 300.741015},
  {18.361640, 15.638338, 358.925362, 17.789189, 314.733295, 17.526621, 300.735758},
  {20.531273, 17.468696, 358.674687, 18.199724, 314.724697, 17.875918, 300.728442},
  {22.704044, 19.295915, 358.395727, 18.653657, 314.713091, 18.261710, 300.718578},
  {24.880271, 21.119674, 358.087918, 19.151028, 314.697825, 18.683905, 300.705620},
  {27.060268, 22.939660, 357.750641, 19.691877, 314.678177, 19.142396, 300.688965},
  {29.244338, 24.755569, 357.383229, 20.276236, 314.653350, 19.637061, 300.667949},
  {31.432772, 26.567112, 356.984969, 20.904133, 314.622470, 20.167762, 300.641849},
  {33.625841, 28.374013, 356.555100, 21.575581, 314.584581, 20.734341, 300.609879},
  {35.823798, 30.176023, 356.092822, 22.290583, 314.538649, 21.336620, 300.571189},
  {38.026863, 31.972918, 355.597298, 23.049122, 314.483551, 21.974398, 300.524863},
  {40.235225, 33.764512, 355.067661, 23.851159, 314.418075, 22.647449, 300.469918},
  {42.449024, 35.550660, 354.503020, 24.696626, 314.340921, 23.355520, 300.405304},
  {44.668349, 37.331276, 353.902470, 25.585425, 314.250692, 24.098328, 300.329896},
  {46.893223, 39.106334, 353.265101, 26.517416, 314.145895, 24.875557, 300.242503},
  {49.123589, 40.875890, 352.590014, 27.492414, 314.024939, 25.686855, 300.141858},
  {51.359299, 42.640094, 351.876333, 28.510178, 313.886135, 26.531836, 300.026620},
  {53.600093, 44.399202, 351.123224, 29.570406, 313.727692, 27.410067, 299.895377},
  {55.845585, 46.153600, 350.329918, 30.672723, 313.547720, 28.321078, 299.746641},
  {58.095244, 47.903820, 349.495731, 31.816670, 313.344231, 29.264347, 299.578852},
  {60.348372, 49.650557, 348.620095, 33.001697, 313.115144, 30.239308, 299.390378},
  {62.384327, 51.214468, 347.705820, 34.222855, 312.859223, 31.241819, 299.180282},
  {63.981269, 52.417407, 346.759294, 35.470845, 312.576449, 32.264250, 298.948619},
  {65.136530, 53.262050, 345.787343, 36.736372, 312.267338, 33.299070, 298.695862},
  {65.847630, 53.750882, 344.797198, 38.010176, 311.932992, 34.338867, 298.422940},
  {66.112408, 53.886067, 343.796446, 39.283077, 311.575132, 35.376367, 298.131262},
  {66.149682, 53.848774, 342.789590, 40.550226, 311.194810, 36.407883, 297.821666},
  {66.179220, 53.819221, 341.777897, 41.811042, 310.792046, 37.433223, 297.494125},
  {66.200693, 53.797737, 340.762688, 43.064943, 310.366904, 38.452201, 297.148636},
  {66.213827, 53.784596, 339.745328, 44.311352, 309.919499, 39.464642, 296.785215},
  {66.218404, 53.780018, 338.727219, 45.549703, 309.449990, 40.470383, 296.403899},
  {66.214264, 53.784159, 337.709788, 46.779441, 308.958588, 41.469268, 296.004745},
  {66.201314, 53.797116, 336.694477, 48.000029, 308.445548, 42.461153, 295.587833},
  {66.179524, 53.818917, 335.682735, 49.210952, 307.911172, 43.445906, 295.153263},
  {66.148932, 53.849524, 334.676001, 50.411718, 307.355804, 44.423407, 294.701156},
  {66.109641, 53.888835, 333.675702, 51.601865, 306.779832, 45.393546, 294.231654},
  {66.061814, 53.936685, 332.683234, 52.780961, 306.183680, 46.356228, 293.744919},
  {66.005679, 53.992848, 331.699959, 53.948610, 305.567811, 47.311367, 293.241132},
  {65.941517, 54.057042, 330.727190, 55.104451, 304.932716, 48.258892, 292.720496},
  {65.869658, 54.128935, 329.766187, 56.248162, 304.278919, 49.198743, 292.183230},
  {65.790481, 54.208150, 328.818150, 57.379460, 303.606967, 50.130876, 291.629570},
  {65.704398, 54.294273, 327.884208, 58.498104, 302.917428, 51.055255, 291.059772},
  {65.611856, 54.386858, 326.965419, 59.603892, 302.210889, 51.971861, 290.474105},
  {65.513323, 54.485436, 326.062765, 60.696667, 301.487949, 52.880685, 289.872854},
  {65.409286, 54.589519, 325.177147, 61.776309, 300.749218, 53.781734, 289.256317},
  {65.300240, 54.698613, 324.309383, 62.842739, 299.995312, 54.675024, 288.624805},
  {65.186685, 54.812216, 323.460214, 63.895920, 299.226851, 55.560588, 287.978640},
  {65.069118, 54.929833, 322.630295, 64.935848, 298.444453, 56.438468, 287.318154},
  {64.948027, 55.050973, 321.820203, 65.962557, 297.648736, 57.308719, 286.643687},
  {64.823887, 55.175163, 321.030438, 66.976116, 296.840310, 58.171412, 285.955587},
  {64.697158, 55.301942, 320.261423, 67.976624, 296.019778, 59.026624, 285.254208},
  {64.568275, 55.430873, 319.513511, 68.964211, 295.187734, 59.874449, 284.539907},
  {64.437654, 55.561542, 318.786987, 69.939035, 294.344761, 60.714990, 283.813047},
  {64.305682, 55.693562, 318.082071, 70.901278, 293.491428, 61.548362, 283.073993},
  {64.172720, 55.826570, 317.398925, 71.851145, 292.628290, 62.374688, 282.323109},
  {64.039101, 55.960233, 316.737657, 72.788864, 291.755888, 63.194106, 281.560761},
  {63.905128, 56.094249, 316.098325, 73.714680, 290.874747, 64.006760, 280.787315},
  {63.771078, 56.228342, 315.480940, 74.628856, 289.985374, 64.812805, 280.003133},
  {63.637196, 56.362264, 314.885476, 75.531668, 289.088264, 65.612406, 279.208576},
  {63.503701, 56.495798, 314.311869, 76.423406, 288.183891, 66.405734, 278.404001},
  {63.370786, 56.628750, 313.760024, 77.304372, 287.272714, 67.192970, 277.589762},
  {63.238618, 56.760954, 313.229818, 78.174875, 286.355178, 67.974303, 276.766208},
  {63.107337, 56.892269, 312.721106, 79.035234, 285.431708, 68.749928, 275.933683},
  {62.977064, 57.022574, 312.233723, 79.885775, 284.502716, 69.520047, 275.092525},
  {62.847897, 57.151772, 311.767488, 80.726826, 283.568599, 70.284869, 274.243068},
  {62.719912, 57.279786, 311.322206, 81.558723, 282.629737, 71.044608, 273.385638},
  {62.593169, 57.406557, 310.897675, 82.381804, 281.686498, 71.799483, 272.520556},
  {62.467710, 57.532041, 310.493684, 83.196407, 280.739235, 72.549722, 271.648135},
  {62.343561, 57.656215, 310.110019, 84.002875, 279.788289, 73.295552, 270.768685},
  {62.220735, 57.779063, 309.746463, 84.801548, 278.833988, 74.037210, 269.882507},
  {62.099231, 57.900589, 309.402799, 85.592768, 277.876650, 74.774932, 268.989895},
  {61.979036, 58.020804, 309.078813, 86.376877, 276.916581, 75.508963, 268.091137},
  {61.860128, 58.139730, 308.774293, 87.154214, 275.954075, 76.239548, 267.186518},
  {61.742475, 58.257401, 308.489036, 87.925119, 274.989420, 76.966937, 266.276312},
  {61.626035, 58.373857, 308.222841, 88.689927, 274.022892, 77.691383, 265.360790},
  {61.510761, 58.489146, 307.975519, 89.448975, 273.054762, 78.413142, 264.440216},
  {61.396598, 58.603323, 307.746886, 90.202593, 272.085290, 79.132473, 263.514850},
  {61.283485, 58.716448, 307.536771, 90.951113, 271.114731, 79.849638, 262.584945},
  {61.171357, 58.828587, 307.345012, 91.694861, 270.143334, 80.564901, 261.650750},
  {61.060144, 58.939810, 307.171461, 92.434162, 269.171341, 81.278529, 260.712509},
  {60.949771, 59.050192, 307.015978, 93.169338, 268.198990, 81.990791, 259.770462},
  {60.840162, 59.159809, 306.878439, 93.900708, 267.226515, 82.701961, 258.824845},
  {60.731236, 59.268742, 306.758732, 94.628588, 266.254144, 83.412311, 257.875890},
  {60.622910, 59.377074, 306.656759, 95.353290, 265.282104, 84.122120, 256.923826},
  {60.515100, 59.484889, 306.572435, 96.075125, 264.310618, 84.831666, 255.968878},
  {60.407720, 59.592273, 306.505690, 96.794400, 263.339907, 85.541231, 255.011270},
  {60.300681, 59.699315, 306.456468, 97.511420, 262.370190, 86.251100, 254.051224},
  {60.193895, 59.806103, 306.424727, 98.226486, 261.401684, 86.961559, 253.088958},
  {60.087273, 59.912727, 306.410440, 98.939896, 260.434608, 87.672897, 252.124690},
  {59.980724, 60.019276, 306.413595, 99.651948, 259.469177, 88.385407, 251.158639},
  {59.874158, 60.125841, 306.434196, 100.362935, 258.505608, 89.099382, 250.191020},
  {59.767486, 60.232512, 306.472259, 101.073147, 257.544120, 89.815120, 249.222051},
  {59.660616, 60.339379, 306.527816, 101.782873, 256.584930, 90.532921, 248.251949},
  {59.553460, 60.446531, 306.600915, 102.492398, 255.628259, 91.253087, 247.280932},
  {59.445930, 60.554057, 306.691617, 103.202007, 254.674329, 91.975924, 246.309219},
  {59.337938, 60.662044, 306.799996, 103.911980, 253.723364, 92.701740, 245.337034},
  {59.229399, 60.770577, 306.926143, 104.622595, 252.775590, 93.430846, 244.364599},
  {59.120227, 60.879741, 307.070162, 105.334128, 251.831238, 94.163556, 243.392142},
  {59.010343, 60.989617, 307.232167, 106.046853, 250.890541, 94.900188, 242.419894},
  {58.899665, 61.100285, 307.412291, 106.761042, 249.953736, 95.641061, 241.448089},
  {58.788119, 61.211821, 307.610674, 107.476962, 249.021065, 96.386499, 240.476967},
  {58.675631, 61.324298, 307.827470, 108.194880, 248.092774, 97.136825, 239.506772},
  {58.562132, 61.437783, 308.062845, 108.915060, 247.169112, 97.892370, 238.537756},
  {58.447560, 61.552342, 308.316975, 109.637761, 246.250336, 98.653464, 237.570175},
  {58.331855, 61.668032, 308.590046, 110.363243, 245.336707, 99.420439, 236.604293},
  {58.214964, 61.784905, 308.882249, 111.091760, 244.428491, 100.193632, 235.640384},
  {58.096843, 61.903009, 309.193789, 111.823563, 243.525960, 100.973381, 234.678726},
  {57.977454, 62.022379, 309.524871, 112.558901, 242.629394, 101.760024, 233.719610},
  {57.856767, 62.143045, 309.875708, 113.298020, 241.739076, 102.553901, 232.763335},
  {57.734764, 62.265027, 310.246516, 114.041160, 240.855297, 103.355355, 231.810212},
  {57.611435, 62.388332, 310.637511, 114.788559, 239.978356, 104.164727, 230.860560},
  {57.486785, 62.512957, 311.048910, 115.540449, 239.108555, 104.982359, 229.914713},
  {57.360831, 62.638885, 311.480925, 116.297058, 238.246205, 105.808591, 228.973016},
  {57.233605, 62.766083, 311.933766, 117.058611, 237.391623, 106.643762, 228.035827},
  {57.105155, 62.894503, 312.407632, 117.825325, 236.545130, 107.488208, 227.103520},
  {56.975546, 63.024081, 312.902713, 118.597413, 235.707058, 108.342264, 226.176480},
  {56.844863, 63.154731, 313.419184, 119.375080, 234.877739, 109.206256, 225.255109},
  {56.713212, 63.286347, 313.957204, 120.158527, 234.057516, 110.080509, 224.339824},
  {56.580719, 63.418804, 314.516912, 120.947947, 233.246734, 110.965337, 223.431057},
  {56.447536, 63.551949, 315.098418, 121.743524, 232.445743, 111.861048, 222.529258},
  {56.313838, 63.685607, 315.701808, 122.545437, 231.654898, 112.767939, 221.634892},
  {56.179828, 63.819577, 316.327133, 123.353855, 230.874559, 113.686296, 220.748440},
  {56.045733, 63.953629, 316.974407, 124.168939, 230.105087, 114.616390, 219.870401},
  {55.911811, 64.087507, 317.643601, 124.990841, 229.346845, 115.558477, 219.001290},
  {55.778348, 64.220925, 318.334639, 125.819701, 228.600200, 116.512798, 218.141637},
  {55.645658, 64.353568, 319.047396, 126.655653, 227.865516, 117.479570, 217.291987},
  {55.514084, 64.485095, 319.781688, 127.498817, 227.143159, 118.458992, 216.452903},
  {55.383997, 64.615133, 320.537272, 128.349304, 226.433492, 119.451238, 215.624957},
  {55.255796, 64.743286, 321.313840, 129.207213, 225.736876, 120.456456, 214.808737},
  {55.129902, 64.869130, 322.111013, 130.072632, 225.053669, 121.474763, 214.004839},
  {55.006762, 64.992220, 322.928341, 130.945637, 224.384221, 122.506249, 213.213870},
  {54.886841, 65.112092, 323.765297, 131.826291, 223.728879, 123.550971, 212.436444},
  {54.770620, 65.228264, 324.621274, 132.714646, 223.087978, 124.608948, 211.673178},
  {54.658590, 65.340245, 325.495588, 133.610741, 222.461848, 125.680166, 210.924692},
  {54.551252, 65.447536, 326.387470, 134.514602, 221.850807, 126.764571, 210.191605},
  {54.449104, 65.549639, 327.296071, 135.426243, 221.255162, 127.862070, 209.474529},
  {54.352639, 65.646059, 328.220460, 136.345663, 220.675204, 128.972531, 208.774073},
  {54.262340, 65.736317, 329.159628, 137.272851, 220.111216, 130.095779, 208.090831},
  {54.178666, 65.819951, 330.112491, 138.207781, 219.563459, 131.231598, 207.425384},
  {54.102054, 65.896526, 331.077893, 139.150415, 219.032184, 132.379733, 206.778292},
  {54.032906, 65.965641, 332.054613, 140.100703, 218.517621, 133.539886, 206.150095},
  {53.971583, 66.026934, 333.041369, 141.058580, 218.019982, 134.711721, 205.541305},
  {53.918398, 66.080093, 334.036830, 142.023970, 217.539463, 135.894864, 204.952404},
  {53.959056, 65.876127, 335.012267, 142.998330, 217.075507, 137.084391, 204.385931},
  {58.021341, 61.978499, 335.336165, 144.051518, 216.588285, 138.209408, 203.865481},
  {58.005811, 61.994027, 335.662606, 145.107172, 216.107177, 139.337644, 203.351294},
  {57.990274, 62.009562, 335.991589, 146.165277, 215.632243, 140.469086, 202.843442},
  {57.974735, 62.025098, 336.323117, 147.225817, 215.163541, 141.603721, 202.341995},
  {57.959201, 62.040629, 336.657187, 148.288777, 214.701130, 142.741534, 201.847022},
  {57.943678, 62.056150, 336.993797, 149.354140, 214.245071, 143.882509, 201.358596},
  {57.928173, 62.071652, 337.332946, 150.421888, 213.795423, 145.026631, 200.876785},
  {57.912693, 62.087130, 337.674629, 151.492003, 213.352245, 146.173881, 200.401663},
  {57.897244, 62.102576, 338.018841, 152.564465, 212.915597, 147.324241, 199.933299},
  {57.881834, 62.117983, 338.365575, 153.639254, 212.485538, 148.477690, 199.471766},
  {57.866471, 62.133343, 338.714823, 154.716349, 212.062126, 149.634207, 199.017134},
  {57.851162, 62.148649, 339.066578, 155.795729, 211.645423, 150.793769, 198.569475},
  {57.835915, 62.163894, 339.420829, 156.877370, 211.235485, 151.956352, 198.128861},
  {57.820738, 62.179068, 339.777563, 157.961250, 210.832372, 153.121930, 197.695363},
  {57.805639, 62.194165, 340.136770, 159.047343, 210.436142, 154.290479, 197.269052},
  {57.790626, 62.209174, 340.498433, 160.135625, 210.046853, 155.461968, 196.850000},
  {57.775709, 62.224089, 340.862539, 161.226069, 209.664563, 156.636370, 196.438277},
  {57.760896, 62.238900, 341.229069, 162.318648, 209.289331, 157.813653, 196.033954},
  {57.746195, 62.253598, 341.598006, 163.413334, 208.921212, 158.993785, 195.637101},
  {57.731616, 62.268174, 341.969328, 164.510098, 208.560263, 160.176733, 195.247790},
  {57.717169, 62.282619, 342.343016, 165.608911, 208.206541, 161.362462, 194.866088},
  {57.702861, 62.296923, 342.719046, 166.709741, 207.860102, 162.550937, 194.492067},
  {57.688704, 62.311078, 343.097393, 167.812558, 207.521000, 163.742118, 194.125795},
  {57.674706, 62.325074, 343.478032, 168.917328, 207.189290, 164.935967, 193.767340},
  {57.660877, 62.338900, 343.860934, 170.024019, 206.865027, 166.132443, 193.416770},
  {57.647227, 62.352547, 344.246070, 171.132596, 206.548264, 167.331505, 193.074152},
  {57.633766, 62.366006, 344.633410, 172.243023, 206.239053, 168.533109, 192.739552},
  {57.620503, 62.379266, 345.022921, 173.355265, 205.937447, 169.737209, 192.413037},
  {57.607449, 62.392317, 345.414568, 174.469285, 205.643496, 170.943759, 192.094671},
  {57.594614, 62.405150, 345.808317, 175.585045, 205.357252, 172.152712, 191.784518},
  {57.582007, 62.417755, 346.204129, 176.702506, 205.078763, 173.364017, 191.482642},
  {57.569639, 62.430120, 346.601966, 177.821628, 204.808078, 174.577625, 191.189104},
  {57.557519, 62.442237, 347.001786, 178.942372, 204.545245, 175.793482, 190.903966},
  {57.545659, 62.454095, 347.403548, 180.064695, 204.290311, 177.011536, 190.627287},
  {57.534067, 62.465685, 347.807207, 181.188556, 204.043321, 178.231730, 190.359127},
  {57.522753, 62.476996, 348.212719, 182.313912, 203.804321, 179.454009, 190.099542},
  {57.511728, 62.488019, 348.620034, 183.440719, 203.573353, 180.678315, 189.848590},
  {57.501001, 62.498744, 349.029106, 184.568932, 203.350460, 181.904587, 189.606325},
  {57.490581, 62.509162, 349.439883, 185.698506, 203.135684, 183.132767, 189.372800},
  {57.480479, 62.519262, 349.852314, 186.829395, 202.929065, 184.362791, 189.148068},
  {57.470702, 62.529037, 350.266346, 187.961553, 202.730641, 185.594596, 188.932180},
  {57.461261, 62.538476, 350.681922, 189.094931, 202.540450, 186.828118, 188.725184},
  {57.452165, 62.547570, 351.098988, 190.229481, 202.358528, 188.063292, 188.527128},
  {57.443421, 62.556312, 351.517484, 191.365155, 202.184910, 189.300049, 188.338057},
  {57.435038, 62.564693, 351.937353, 192.501903, 202.019629, 190.538323, 188.158015},
  {57.427025, 62.572704, 352.358534, 193.639675, 201.862718, 191.778043, 187.987044},
  {57.419390, 62.580338, 352.780964, 194.778419, 201.714208, 193.019140, 187.825186},
  {57.412140, 62.587587, 353.204581, 195.918086, 201.574126, 194.261542, 187.672477},
  {57.405281, 62.594444, 353.629321, 197.058621, 201.442502, 195.505176, 187.528954},
  {57.398823, 62.600901, 354.055118, 198.199974, 201.319360, 196.749970, 187.394652},
  {57.392770, 62.606953, 354.481906, 199.342091, 201.204726, 197.995849, 187.269604},
  {57.387129, 62.612593, 354.909617, 200.484918, 201.098623, 199.242739, 187.153839},
  {57.381906, 62.617815, 355.338183, 201.628403, 201.001070, 200.490563, 187.047386},
  {57.377106, 62.622613, 355.767534, 202.772490, 200.912089, 201.739244, 186.950270},
  {57.372735, 62.626983, 356.197601, 203.917125, 200.831698, 202.988706, 186.862516},
  {57.368797, 62.630920, 356.628313, 205.062253, 200.759911, 204.238870, 186.784145},
  {57.365297, 62.634420, 357.059598, 206.207819, 200.696745, 205.489658, 186.715177},
  {57.362237, 62.637479, 357.491383, 207.353767, 200.642211, 206.740992, 186.655628},
  {57.359622, 62.640094, 357.923597, 208.500041, 200.596321, 207.992791, 186.605514},
  {57.357453, 62.642262, 358.356165, 209.646586, 200.559085, 209.244976, 186.564846},
  {57.355735, 62.643980, 358.789015, 210.793344, 200.530509, 210.497467, 186.533636},
  {57.354467, 62.645247, 359.222072, 211.940261, 200.510600, 211.750183, 186.511891},
  {57.353652, 62.646062, 359.655263, 213.087279, 200.499363, 213.003044, 186.499616},
  {57.353291, 62.646423, 0.088513, 214.234342, 200.496798, 214.255970, 186.496814},
  {57.353384, 62.646330, 0.521747, 215.381393, 200.502906, 215.508878, 186.503487},
  {57.353930, 62.645784, 0.954892, 216.528377, 200.517688, 216.761690, 186.519632},
  {57.354930, 62.644784, 1.387874, 217.675236, 200.541138, 218.014324, 186.545245},
  {57.356382, 62.643332, 1.820618, 218.821914, 200.573253, 219.266699, 186.580321},
  {57.358285, 62.641430, 2.253050, 219.968354, 200.614026, 220.518736, 186.624849},
  {57.360636, 62.639079, 2.685097, 221.114502, 200.663449, 221.770355, 186.678820},
  {57.363434, 62.636283, 3.116687, 222.260301, 200.721512, 223.021475, 186.742219},
  {57.366674, 62.633043, 3.547746, 223.405694, 200.788202, 224.272018, 186.815032},
  {57.370354, 62.629364, 3.978203, 224.550627, 200.863506, 225.521905, 186.897239},
  {57.374469, 62.625250, 4.407986, 225.695045, 200.947410, 226.771057, 186.988821},
  {57.379016, 62.620704, 4.837025, 226.838893, 201.039895, 228.019398, 187.089755},
  {57.383989, 62.615732, 5.265250, 227.982115, 201.140944, 229.266849, 187.200016},
  {57.389383, 62.610339, 5.692592, 229.124659, 201.250536, 230.513334, 187.319578},
  {57.395193, 62.604530, 6.118983, 230.266470, 201.368649, 231.758779, 187.448411},
  {57.401413, 62.598312, 6.544356, 231.407495, 201.495259, 233.003109, 187.586484},
  {57.408036, 62.591690, 6.968645, 232.547682, 201.630342, 234.246248, 187.733765},
  {57.415055, 62.584672, 7.391785, 233.686978, 201.773871, 235.488126, 187.890217},
  {57.422464, 62.577265, 7.813712, 234.825331, 201.925818, 236.728669, 188.055803},
  {57.430254, 62.569476, 8.234364, 235.962691, 202.086152, 237.967806, 188.230485},
  {57.438419, 62.561313, 8.653680, 237.099006, 202.254843, 239.205468, 188.414221},
  {57.446950, 62.552784, 9.071599, 238.234227, 202.431858, 240.441587, 188.606969},
  {57.455839, 62.543897, 9.488063, 239.368304, 202.617163, 241.676094, 188.808684},
  {57.465078, 62.534660, 9.903015, 240.501189, 202.810723, 242.908922, 189.019319},
  {57.474657, 62.525083, 10.316398, 241.632834, 203.012500, 244.140007, 189.238827},
  {57.484568, 62.515174, 10.728160, 242.763192, 203.222457, 245.369285, 189.467157},
  {57.494801, 62.504943, 11.138247, 243.892215, 203.440554, 246.596693, 189.704259},
  {57.505347, 62.494399, 11.546607, 245.019859, 203.666751, 247.822169, 189.950081},
  {57.516197, 62.483551, 11.953191, 246.146078, 203.901007, 249.045654, 190.204567},
  {57.527342, 62.472409, 12.357951, 247.270829, 204.143277, 250.267088, 190.467662},
  {57.538770, 62.460983, 12.760841, 248.394067, 204.393518, 251.486414, 190.739310},
  {57.550473, 62.449282, 13.161815, 249.515750, 204.651685, 252.703577, 191.019453},
  {57.562441, 62.437317, 13.560829, 250.635836, 204.917732, 253.918522, 191.308030},
  {57.574663, 62.425097, 13.957844, 251.754284, 205.191611, 255.131195, 191.604983},
  {57.587130, 62.412633, 14.352817, 252.871055, 205.473275, 256.341545, 191.910248},
  {57.599831, 62.399934, 14.745712, 253.986108, 205.762674, 257.549522, 192.223764},
  {57.612757, 62.387011, 15.136491, 255.099406, 206.059759, 258.755077, 192.545468},
  {57.625897, 62.373873, 15.525118, 256.210911, 206.364479, 259.958162, 192.875294},
  {57.639242, 62.360530, 15.911562, 257.320587, 206.676782, 261.158733, 193.213178},
  {57.652782, 62.346993, 16.295789, 258.428397, 206.996616, 262.356743, 193.559053},
  {57.666506, 62.333272, 16.677770, 259.534307, 207.323928, 263.552151, 193.912853},
  {57.680405, 62.319375, 17.057475, 260.638283, 207.658664, 264.744915, 194.274511},
  {57.694470, 62.305313, 17.434879, 261.740292, 208.000771, 265.934995, 194.643957},
  {57.708690, 62.291096, 17.809954, 262.840302, 208.350193, 267.122352, 195.021125},
  {57.723055, 62.276733, 18.182678, 263.938281, 208.706874, 268.306949, 195.405944},
  {57.737558, 62.262233, 18.553029, 265.034200, 209.070759, 269.488751, 195.798345},
  {57.752187, 62.247606, 18.920985, 266.128029, 209.441792, 270.667723, 196.198259},
  {57.766935, 62.232861, 19.286526, 267.219739, 209.819915, 271.843833, 196.605614},
  {57.781792, 62.218007, 19.649636, 268.309302, 210.205071, 273.017048, 197.020340},
  {57.796749, 62.203052, 20.010297, 269.396693, 210.597203, 274.187339, 197.442367},
  {57.811798, 62.188006, 20.368495, 270.481884, 210.996252, 275.354677, 197.871623},
  {57.826930, 62.172877, 20.724217, 271.564852, 211.402161, 276.519035, 198.308037},
  {57.842137, 62.157673, 21.077449, 272.645573, 211.814871, 277.680387, 198.751538},
  {57.857410, 62.142402, 21.428180, 273.724022, 212.234322, 278.838707, 199.202055},
  {57.872742, 62.127073, 21.776403, 274.800177, 212.660457, 279.993973, 199.659515},
  {57.888125, 62.111693, 22.122107, 275.874018, 213.093216, 281.146162, 200.123848},
  {57.903552, 62.096269, 22.465286, 276.945523, 213.532539, 282.295254, 200.594982},
  {57.919014, 62.080809, 22.805934, 278.014673, 213.978368, 283.441228, 201.072846},
  {57.934505, 62.065320, 23.144046, 279.081449, 214.430643, 284.584066, 201.557368},
  {57.950019, 62.049810, 23.479619, 280.145832, 214.889304, 285.723752, 202.048478},
  {57.965547, 62.034284, 23.812650, 281.207806, 215.354292, 286.860268, 202.546104},
  {57.981084, 62.018750, 24.143138, 282.267353, 215.825548, 287.993600, 203.050177},
  {57.996623, 62.003214, 24.471083, 283.324458, 216.303012, 289.123733, 203.560626},
  {58.012158, 61.987681, 24.796484, 284.379106, 216.786625, 290.250655, 204.077380},
  {700.966459, 698.638444, 208.521431, 290.945247, 204.400044, 284.260423, 216.700984},
  {60.721750, 179.747874, 228.298864, 291.872565, 205.184222, 281.419815, 214.497654},
  {242.953259, 357.357744, 279.080028, 293.205002, 209.857029, 279.380438, 207.647635},
  {100.915078, 216.532445, 305.285576, 292.500677, 211.748448, 281.072714, 203.661318},
  {33.131014, 81.014033, 309.206406, 292.918046, 211.233794, 282.069813, 202.384171},
  {107.445661, 3.386584, 300.663125, 294.168517, 209.486181, 282.125987, 202.346329},
  {255.987936, 150.904394, 266.874649, 295.443102, 204.527617, 281.463925, 205.290905},
  {298.630051, 198.248594, 225.299985, 292.984046, 199.084731, 283.032856, 208.932259},
  {700.003853, 700.071769, 37.499914, 284.470145, 210.198490, 292.992788, 199.091531},
  {103.039143, 8.332365, 28.374470, 286.198356, 211.321038, 292.851607, 199.002992},
  {107.033962, 16.344113, 18.262730, 288.163695, 212.169536, 292.550942, 198.874722},
  {103.945162, 17.246033, 8.330667, 290.186978, 212.647200, 292.215380, 198.794923},
  {93.748584, 11.018385, 359.747377, 292.057406, 212.777651, 291.995679, 198.777787},
  {80.189776, 1.429418, 353.063215, 293.657929, 212.675248, 291.967091, 198.777728},
  {66.886667, 7.899730, 348.233610, 294.977624, 212.456304, 292.122719, 198.750484},
  {55.393499, 15.405194, 344.960070, 296.055031, 212.198293, 292.422141, 198.677859},
  {45.849743, 20.955021, 342.922300, 296.935982, 211.943724, 292.824626, 198.561021},
  {37.908313, 24.899061, 341.857462, 297.658398, 211.713658, 293.299050, 198.409670},
  {31.181395, 27.626820, 341.566516, 298.250347, 211.517421, 293.823498, 198.235742},
  {25.371364, 29.436838, 341.899280, 298.732056, 211.357927, 294.382419, 198.050762},
  {20.283763, 30.524065, 342.737468, 299.118450, 211.234347, 294.963944, 197.864976},
  {15.808571, 30.998793, 343.980829, 299.421289, 211.143502, 295.557864, 197.687130},
  {11.896493, 30.910475, 345.537193, 299.650783, 211.080706, 296.154262, 197.524367},
  {8.536077, 30.270649, 347.316264, 299.816689, 211.040442, 296.742719, 197.382085},
  {5.732398, 29.074275, 349.226911, 299.928912, 211.016986, 297.312033, 197.263734},
  {3.488271, 27.318527, 351.177539, 299.997640, 211.005004, 297.850415, 197.170648},
  {1.790169, 25.016887, 353.078759, 300.033096, 211.000027, 298.346028, 197.102048},
  {0.601096, 22.206286, 354.847238, 300.045046, 210.998717, 298.787686, 197.055294},
  {0.138920, 18.946615, 356.409458, 300.042274, 210.998903, 299.165514, 197.026384},
  {0.506359, 15.314327, 357.704427, 300.032160, 210.999411, 299.471395, 197.010646},
  {0.585414, 11.393565, 358.684932, 300.020457, 210.999775, 299.699154, 197.003462},
  {0.483408, 7.610823, 359.347457, 300.010790, 210.999940, 299.851348, 197.000848},
  {0.317684, 4.483474, 359.740438, 300.004437, 210.999990, 299.941014, 197.000134},
  {0.161953, 2.166094, 359.930991, 300.001198, 210.999999, 299.984336, 197.000009},
  {0.053163, 0.695648, 359.992282, 300.000135, 211.000000, 299.998249, 197.000000},
  {0.006731, 0.087559, 360.000000, 300.000000, 211.000000, 300.000000, 197.000000},
  {0.000000, 0.000000, 360.000000, 300.000000, 211.000000, 300.000000, 197.000000}};
}}