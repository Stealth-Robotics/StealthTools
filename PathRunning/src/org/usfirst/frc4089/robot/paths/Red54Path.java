/* Red54 Feed Scale from front cont */
package org.usfirst.frc4089.robot.paths;
public class Red54Path extends Path {
    public Red54Path() {
       kNumPoints =345;
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
  {108.389096, 179.335546, 29.093472, 137.171619, 237.160821, 143.978921, 224.927234},
  {55.484806, 231.757555, 43.560884, 138.050511, 237.838297, 147.698261, 227.693303},
  {86.372637, 201.305535, 52.979060, 139.187777, 239.138574, 150.365594, 230.709077},
  {105.519165, 182.336933, 59.269879, 140.355863, 240.896212, 152.390035, 233.742284},
  {116.890955, 171.037479, 63.702948, 141.466600, 242.953312, 154.017729, 236.750961},
  {123.930386, 164.030333, 66.985625, 142.497117, 245.207537, 155.382812, 239.734069},
  {128.529242, 159.447393, 69.516517, 143.447238, 247.596089, 156.562061, 242.696966},
  {131.685164, 156.300021, 71.531375, 144.323647, 250.079695, 157.602609, 245.644701},
  {133.942929, 154.047185, 73.176987, 145.134440, 252.632908, 158.535287, 248.581080},
  {135.615070, 152.378054, 74.549089, 145.887330, 255.238620, 159.381357, 251.508843},
  {136.889758, 151.105298, 75.712667, 146.589140, 257.884936, 160.156125, 254.429949},
  {137.885306, 150.111035, 76.713369, 147.245749, 260.563332, 160.871004, 257.345815},
  {138.678997, 149.318233, 77.584211, 147.862174, 263.267545, 161.534756, 260.257482},
  {139.322998, 148.674864, 78.349677, 148.442694, 265.992862, 162.154269, 263.165728},
  {139.853555, 148.144762, 79.028326, 148.990971, 268.735671, 162.735070, 266.071139},
  {140.296498, 147.702159, 79.634490, 149.510154, 271.493150, 163.281673, 268.974172},
  {140.670640, 147.328275, 80.179427, 150.002964, 274.263065, 163.797818, 271.875179},
  {140.989956, 147.009158, 80.672107, 150.471768, 277.043621, 164.286645, 274.774442},
  {141.265008, 146.734261, 81.119772, 150.918637, 279.833357, 164.750821, 277.672185},
  {141.503906, 146.495482, 81.528339, 151.345391, 282.631075, 165.192635, 280.568592},
  {141.712968, 146.286523, 81.902691, 151.753642, 285.435778, 165.614065, 283.463811},
  {141.897163, 146.102402, 82.246895, 152.144820, 288.246632, 166.016841, 286.357967},
  {142.060469, 145.939164, 82.564371, 152.520207, 291.062934, 166.402479, 289.251162},
  {142.206079, 145.793605, 82.858014, 152.880950, 293.884084, 166.772325, 292.143484},
  {142.336599, 145.663131, 83.130295, 153.228083, 296.709572, 167.127573, 295.035005},
  {142.454162, 145.545603, 83.383333, 153.562544, 299.538956, 167.469294, 297.925790},
  {142.560538, 145.439260, 83.618959, 153.885185, 302.371853, 167.798451, 300.815892},
  {142.657199, 145.342625, 83.838765, 154.196781, 305.207931, 168.115915, 303.705357},
  {142.745380, 145.254464, 84.044136, 154.498044, 308.046899, 168.422474, 306.594226},
  {142.826130, 145.173735, 84.236290, 154.789626, 310.888500, 168.718850, 309.482535},
  {142.900334, 145.099548, 84.416298, 155.072129, 313.732511, 169.005701, 312.370314},
  {142.968751, 145.031145, 84.585108, 155.346108, 316.578729, 169.283633, 315.257590},
  {143.032035, 144.967874, 84.743558, 155.612079, 319.426979, 169.553204, 318.144389},
  {143.090747, 144.909173, 84.892399, 155.870520, 322.277100, 169.814929, 321.030730},
  {143.145378, 144.854550, 85.032296, 156.121877, 325.128952, 170.069288, 323.916634},
  {143.196353, 144.803583, 85.163850, 156.366567, 327.982407, 170.316725, 326.802116},
  {143.244047, 144.755899, 85.287597, 156.604981, 330.837351, 170.557655, 329.687191},
  {143.288784, 144.711166, 85.404020, 156.837485, 333.693679, 170.792468, 332.571873},
  {143.330857, 144.669099, 85.513557, 157.064425, 336.551299, 171.021527, 335.456174},
  {143.370521, 144.629441, 85.616601, 157.286126, 339.410126, 171.245175, 338.340104},
  {143.408003, 144.591962, 85.713509, 157.502896, 342.270083, 171.463735, 341.223672},
  {143.443508, 144.556462, 85.804606, 157.715028, 345.131099, 171.677513, 344.106887},
  {143.477217, 144.522756, 85.890184, 157.922798, 347.993112, 171.886797, 346.989755},
  {143.509294, 144.490683, 85.970512, 158.126471, 350.856062, 172.091863, 349.872284},
  {143.539886, 144.460094, 86.045832, 158.326299, 353.719897, 172.292972, 352.754478},
  {143.569126, 144.430856, 86.116366, 158.522522, 356.584567, 172.490374, 355.636342},
  {143.597136, 144.402849, 86.182314, 158.715372, 359.450027, 172.684306, 358.517881},
  {143.624025, 144.375962, 86.243861, 158.905070, 362.316237, 172.874997, 361.399097},
  {143.649894, 144.350094, 86.301173, 159.091829, 365.183158, 173.062666, 364.279992},
  {143.674836, 144.325154, 86.354403, 159.275855, 368.050756, 173.247525, 367.160570},
  {143.698935, 144.301056, 86.403687, 159.457346, 370.918999, 173.429777, 370.040831},
  {143.722270, 144.277723, 86.449151, 159.636496, 373.787856, 173.609619, 372.920775},
  {143.744913, 144.255081, 86.490909, 159.813490, 376.657301, 173.787241, 375.800404},
  {143.766932, 144.233063, 86.529063, 159.988510, 379.527308, 173.962829, 378.679716},
  {143.788390, 144.211606, 86.563703, 160.161733, 382.397854, 174.136562, 381.558711},
  {143.809346, 144.190650, 86.594913, 160.333332, 385.268917, 174.308616, 384.437387},
  {143.829856, 144.170141, 86.622766, 160.503475, 388.140478, 174.479162, 387.315742},
  {143.849974, 144.150024, 86.647326, 160.672329, 391.012518, 174.648368, 390.193773},
  {143.869748, 144.130250, 86.668648, 160.840057, 393.885020, 174.816399, 393.071476},
  {143.889228, 144.110771, 86.686782, 161.006818, 396.757969, 174.983417, 395.948848},
  {143.908459, 144.091541, 86.701767, 161.172771, 399.631350, 175.149581, 398.825885},
  {143.927486, 144.072514, 86.713638, 161.338073, 402.505150, 175.315049, 401.702580},
  {143.946352, 144.053648, 86.722420, 161.502878, 405.379356, 175.479978, 404.578928},
  {143.965100, 144.034900, 86.728133, 161.667341, 408.253957, 175.644521, 407.454923},
  {143.983771, 144.016229, 86.730790, 161.831616, 411.128943, 175.808832, 410.330557},
  {144.002406, 143.997594, 86.730396, 161.995855, 414.004304, 175.973066, 413.205823},
  {144.021047, 143.978953, 86.726951, 162.160212, 416.880032, 176.137375, 416.080710},
  {144.039733, 143.960266, 86.720446, 162.324839, 419.756119, 176.301912, 418.955210},
  {144.058508, 143.941492, 86.710868, 162.489892, 422.632557, 176.466830, 421.829312},
  {144.077411, 143.922588, 86.698196, 162.655525, 425.509341, 176.632285, 424.703005},
  {144.096486, 143.903513, 86.682401, 162.821895, 428.386465, 176.798432, 427.576276},
  {144.115777, 143.884222, 86.663448, 162.989159, 431.263923, 176.965427, 430.449110},
  {144.135328, 143.864671, 86.641294, 163.157478, 434.141711, 177.133430, 433.321495},
  {144.155185, 143.844812, 86.615890, 163.327014, 437.019826, 177.302602, 436.193413},
  {144.175399, 143.824599, 86.587177, 163.497934, 439.898264, 177.473105, 439.064847},
  {144.196018, 143.803978, 86.555088, 163.670405, 442.777023, 177.645107, 441.935779},
  {144.217098, 143.782898, 86.519548, 163.844600, 445.656100, 177.818778, 444.806188},
  {144.238694, 143.761301, 86.480473, 164.020696, 448.535494, 177.994291, 447.676052},
  {144.260867, 143.739127, 86.437768, 164.198873, 451.415204, 178.171824, 450.545347},
  {144.283680, 143.716312, 86.391328, 164.379319, 454.295230, 178.351560, 453.414048},
  {144.307204, 143.692787, 86.341037, 164.562225, 457.175573, 178.533687, 456.282127},
  {144.331512, 143.668478, 86.286767, 164.747790, 460.056233, 178.718400, 459.149554},
  {144.356682, 143.643305, 86.228377, 164.936220, 462.937211, 178.905898, 462.016295},
  {144.382804, 143.617181, 86.165710, 165.127726, 465.818510, 179.096389, 464.882315},
  {144.409971, 143.590013, 86.098595, 165.322532, 468.700132, 179.290089, 467.747575},
  {144.438285, 143.561696, 86.026845, 165.520868, 471.582081, 179.487220, 470.612034},
  {144.467860, 143.532119, 85.950254, 165.722974, 474.464361, 179.688018, 473.475645},
  {144.498819, 143.501157, 85.868594, 165.929105, 477.346977, 179.892725, 476.338358},
  {144.531299, 143.468673, 85.781617, 166.139523, 480.229934, 180.101596, 479.200119},
  {144.565452, 143.434517, 85.689049, 166.354508, 483.113239, 180.314898, 482.060869},
  {144.601446, 143.398520, 85.590588, 166.574352, 485.996900, 180.532914, 484.920541},
  {144.639466, 143.360494, 85.485903, 166.799366, 488.880925, 180.755938, 487.779063},
  {144.679724, 143.320231, 85.374626, 167.029877, 491.765323, 180.984283, 490.636358},
  {144.722454, 143.277495, 85.256355, 167.266234, 494.650106, 181.218279, 493.492338},
  {144.767919, 143.232023, 85.130640, 167.508807, 497.535285, 181.458278, 496.346907},
  {144.816419, 143.183516, 84.996985, 167.757990, 500.420874, 181.704652, 499.199960},
  {144.868292, 143.131634, 84.854838, 168.014208, 503.306889, 181.957797, 502.051378},
  {144.923924, 143.075992, 84.703582, 168.277912, 506.193346, 182.218139, 504.901030},
  {144.983757, 143.016148, 84.542531, 168.549592, 509.080266, 182.486131, 507.748771},
  {145.048297, 142.951597, 84.370914, 168.829772, 511.967670, 182.762261, 510.594437},
  {145.118126, 142.881750, 84.187864, 169.119024, 514.855583, 183.047053, 513.437845},
  {145.193927, 142.805935, 83.992404, 169.417964, 517.744034, 183.341076, 516.278789},
  {145.276484, 142.723356, 83.783428, 169.727268, 520.633053, 183.644944, 519.117037},
  {145.366725, 142.633090, 83.559676, 170.047673, 523.522679, 183.959322, 521.952323},
  {145.465742, 142.534049, 83.319714, 170.379988, 526.412952, 184.284938, 524.784346},
  {145.574822, 142.424934, 83.061892, 170.725107, 529.303921, 184.622588, 527.612762},
  {145.695512, 142.304208, 82.784310, 171.084018, 532.195643, 184.973143, 530.437174},
  {145.829661, 142.170011, 82.484763, 171.457823, 535.088183, 185.337564, 533.257125},
  {145.979515, 142.020101, 82.160680, 171.847752, 537.981617, 185.716915, 536.072081},
  {146.147814, 141.851735, 81.809041, 172.255193, 540.876037, 186.112375, 538.881419},
  {146.337933, 141.661532, 81.426271, 172.681713, 543.771550, 186.525261, 541.684403},
  {146.554072, 141.445289, 81.008111, 173.129102, 546.668287, 186.957048, 544.480162},
  {146.801512, 141.197719, 80.549433, 173.599412, 549.566404, 187.409399, 547.267651},
  {147.086969, 140.912096, 80.044011, 174.095022, 552.466094, 187.884194, 550.045610},
  {147.419105, 140.579749, 79.484200, 174.618711, 555.367594, 188.383576, 552.812501},
  {147.809239, 140.189342, 78.860500, 175.173761, 558.271204, 188.909997, 555.566426},
  {148.272405, 139.725806, 78.160947, 175.764093, 561.177299, 189.466281, 558.305014},
  {148.828948, 139.168768, 77.370244, 176.394455, 564.086366, 190.055701, 561.025265},
  {149.506929, 138.490099, 76.468495, 177.070688, 566.999034, 190.682067, 563.723314},
  {150.345976, 137.650079, 75.429307, 177.800106, 569.916141, 191.349838, 566.394100},
  {151.403499, 136.591134, 74.216876, 178.592067, 572.838812, 192.064241, 569.030857},
  {152.765190, 135.227286, 72.781341, 179.458846, 575.768586, 192.831394, 571.624318},
  {154.563404, 133.425668, 71.051128, 180.417044, 578.707599, 193.658366, 574.161459},
  {157.010721, 130.972701, 68.919764, 181.489960, 581.658835, 194.553047, 576.623386},
  {160.464120, 127.509342, 66.222128, 182.711810, 584.626423, 195.523427, 578.981736},
  {165.552947, 122.401573, 62.689576, 184.135711, 587.615673, 196.575184, 581.192316},
  {173.440294, 114.474857, 57.861748, 185.849750, 590.631412, 197.704488, 583.183916},
  {186.325845, 101.498728, 50.914239, 188.010453, 593.667577, 198.877296, 584.840816},
  {207.999549, 79.598823, 40.389418, 190.904700, 596.655696, 199.976409, 585.992484},
  {241.154368, 45.914941, 24.355365, 194.964241, 599.259981, 200.737769, 586.505908},
  {270.248484, 16.140773, 3.439827, 200.208569, 600.567925, 201.048573, 586.593148},
  {260.699802, 25.943579, 344.131928, 205.389580, 599.982367, 201.561655, 586.515853},
  {225.759903, 61.583185, 330.662187, 209.548681, 598.224841, 202.689271, 586.020396},
  {197.375276, 90.345597, 321.892988, 212.823439, 596.020582, 204.183589, 585.004549},
  {180.058523, 107.814176, 315.977049, 215.532656, 593.648121, 205.803406, 583.581260},
  {169.701085, 118.234248, 311.763474, 217.880832, 591.197519, 207.438222, 581.872719},
  {163.220270, 124.743557, 308.613713, 219.984660, 588.701471, 209.045464, 579.964538},
  {158.944364, 129.033769, 306.165310, 221.913066, 586.174304, 210.610617, 577.912666},
  {155.985872, 132.000062, 304.201949, 223.709252, 583.623549, 212.130392, 575.753988},
  {153.855200, 134.135282, 302.587798, 225.401866, 581.053796, 213.605926, 573.513517},
  {152.268687, 135.724618, 301.233616, 227.010840, 578.468162, 215.039997, 571.208759},
  {151.054012, 136.941112, 300.078440, 228.550606, 575.868923, 216.435845, 568.852331},
  {150.102012, 137.894342, 299.079215, 230.031970, 573.257830, 217.796690, 566.453573},
  {149.340907, 138.656299, 298.204660, 231.463244, 570.636277, 219.125534, 564.019563},
  {148.721970, 139.275844, 297.431478, 232.850973, 568.005403, 220.425099, 561.555778},
  {148.211183, 139.787082, 296.741952, 234.200414, 565.366155, 221.697824, 559.066533},
  {147.784197, 140.214405, 296.122353, 235.515860, 562.719333, 222.945878, 556.555280},
  {147.423205, 140.575640, 295.561870, 236.800869, 560.065619, 224.171190, 554.024823},
  {147.114960, 140.884092, 295.051865, 238.058425, 557.405605, 225.375477, 551.477466},
  {146.849388, 141.149817, 294.585347, 239.291063, 554.739803, 226.560267, 548.915127},
  {146.618754, 141.380571, 294.156595, 240.500952, 552.068663, 227.726927, 546.339416},
  {146.417025, 141.582407, 293.760875, 241.689970, 549.392581, 228.876680, 543.751695},
  {146.239416, 141.760091, 293.394237, 242.859752, 546.711909, 230.010628, 541.153131},
  {146.082126, 141.917450, 293.053353, 244.011731, 544.026963, 231.129762, 538.544729},
  {145.942074, 142.057558, 292.735401, 245.147175, 541.338022, 232.234982, 535.927358},
  {145.816748, 142.182923, 292.437968, 246.267208, 538.645340, 233.327101, 533.301779},
  {145.704101, 142.295616, 292.158979, 247.372834, 535.949146, 234.406862, 530.668656},
  {145.602413, 142.397335, 291.896640, 248.464956, 533.249646, 235.474942, 528.028579},
  {145.510270, 142.489513, 291.649387, 249.544384, 530.547031, 236.531960, 525.382069},
  {145.426464, 142.573336, 291.415855, 250.611853, 527.841471, 237.578485, 522.729589},
  {145.349994, 142.649826, 291.194843, 251.668030, 525.133125, 238.615041, 520.071556},
  {145.279996, 142.719839, 290.985292, 252.713524, 522.422138, 239.642111, 517.408342},
  {145.215740, 142.784117, 290.786260, 253.748890, 519.708642, 240.660139, 514.740283},
  {145.156587, 142.843283, 290.596914, 254.774639, 516.992760, 241.669540, 512.067683},
  {145.101992, 142.897886, 290.416505, 255.791240, 514.274607, 242.670698, 509.390818},
  {145.051491, 142.948407, 290.244365, 256.799124, 511.554286, 243.663969, 506.709939},
  {145.004653, 142.995241, 290.079893, 257.798693, 508.831897, 244.649686, 504.025276},
  {144.961139, 143.038770, 289.922545, 258.790316, 506.107530, 245.628159, 501.337037},
  {144.920619, 143.079297, 289.771830, 259.774339, 503.381270, 246.599678, 498.645416},
  {144.882819, 143.117106, 289.627305, 260.751081, 500.653196, 247.564516, 495.950589},
  {144.847491, 143.152443, 289.488563, 261.720841, 497.923382, 248.522928, 493.252721},
  {144.814412, 143.185522, 289.355237, 262.683899, 495.191898, 249.475153, 490.551961},
  {144.783393, 143.216543, 289.226988, 263.640516, 492.458809, 250.421417, 487.848449},
  {144.754264, 143.245684, 289.103509, 264.590936, 489.724176, 251.361932, 485.142315},
  {144.726857, 143.273088, 288.984517, 265.535390, 486.988056, 252.296899, 482.433679},
  {144.701046, 143.298906, 288.869750, 266.474094, 484.250504, 253.226507, 479.722653},
  {144.676698, 143.323257, 288.758969, 267.407252, 481.511570, 254.150935, 477.009343},
  {144.653702, 143.346254, 288.651953, 268.335056, 478.771304, 255.070352, 474.293844},
  {144.631957, 143.368004, 288.548497, 269.257687, 476.029751, 255.984921, 471.576250},
  {144.611370, 143.388594, 288.448412, 270.175317, 473.286955, 256.894791, 468.856645},
  {144.591856, 143.408110, 288.351521, 271.088109, 470.542955, 257.800110, 466.135110},
  {144.573339, 143.426628, 288.257661, 271.996216, 467.797792, 258.701014, 463.411721},
  {144.555751, 143.444218, 288.166681, 272.899784, 465.051501, 259.597635, 460.686547},
  {144.539028, 143.460943, 288.078439, 273.798953, 462.304119, 260.490097, 457.959657},
  {144.523111, 143.476861, 287.992802, 274.693853, 459.555679, 261.378519, 455.231114},
  {144.507949, 143.492026, 287.909647, 275.584612, 456.806212, 262.263015, 452.500976},
  {144.493491, 143.506484, 287.828860, 276.471347, 454.055749, 263.143693, 449.769301},
  {144.479696, 143.520283, 287.750331, 277.354174, 451.304318, 264.020657, 447.036141},
  {144.466520, 143.533464, 287.673959, 278.233201, 448.551948, 264.894007, 444.301548},
  {144.453918, 143.546055, 287.599649, 279.108532, 445.798665, 265.763837, 441.565568},
  {144.441872, 143.558110, 287.527312, 279.980267, 443.044493, 266.630238, 438.828248},
  {144.430337, 143.569648, 287.456864, 280.848500, 440.289458, 267.493297, 436.089630},
  {144.419284, 143.580700, 287.388225, 281.713322, 437.533582, 268.353097, 433.349756},
  {144.408686, 143.591294, 287.321321, 282.574821, 434.776887, 269.209720, 430.608665},
  {144.398522, 143.601461, 287.256080, 283.433081, 432.019394, 270.063242, 427.866394},
  {144.388764, 143.611221, 287.192438, 284.288181, 429.261125, 270.913737, 425.122978},
  {144.379390, 143.620596, 287.130329, 285.140198, 426.502098, 271.761277, 422.378451},
  {144.370379, 143.629607, 287.069696, 285.989207, 423.742333, 272.605930, 419.632846},
  {144.361713, 143.638274, 287.010482, 286.835279, 420.981846, 273.447762, 416.886193},
  {144.353373, 143.646615, 286.952633, 287.678482, 418.220657, 274.286836, 414.138523},
  {144.345344, 143.654646, 286.896099, 288.518882, 415.458781, 275.123214, 411.389862},
  {144.337605, 143.662381, 286.840831, 289.356541, 412.696234, 275.956955, 408.640239},
  {144.330151, 143.669843, 286.786784, 290.191523, 409.933032, 276.788117, 405.889679},
  {144.322954, 143.677031, 286.733914, 291.023884, 407.169191, 277.616753, 403.138207},
  {144.316017, 143.683974, 286.682181, 291.853682, 404.404723, 278.442917, 400.385847},
  {144.309320, 143.690675, 286.631544, 292.680972, 401.639644, 279.266660, 397.632621},
  {144.302848, 143.697146, 286.581967, 293.505807, 398.873967, 280.088033, 394.878552},
  {144.296593, 143.703397, 286.533413, 294.328238, 396.107704, 280.907083, 392.123661},
  {144.290546, 143.709439, 286.485849, 295.148314, 393.340868, 281.723856, 389.367969},
  {144.284706, 143.715291, 286.439241, 295.966082, 390.573471, 282.538397, 386.611493},
  {144.279049, 143.720947, 286.393560, 296.781590, 387.805525, 283.350750, 383.854254},
  {144.273570, 143.726419, 286.348775, 297.594882, 385.037041, 284.160957, 381.096270},
  {144.268268, 143.731721, 286.304858, 298.406000, 382.268031, 284.969059, 378.337558},
  {144.263137, 143.736863, 286.261782, 299.214988, 379.498504, 285.775096, 375.578134},
  {144.258152, 143.741834, 286.219521, 300.021885, 376.728471, 286.579105, 372.818015},
  {144.253331, 143.746665, 286.178050, 300.826731, 373.957942, 287.381124, 370.057217},
  {144.248652, 143.751347, 286.137345, 301.629564, 371.186926, 288.181189, 367.295754},
  {144.244109, 143.755885, 286.097383, 302.430421, 368.415433, 288.979336, 364.533642},
  {144.239701, 143.760292, 286.058143, 303.229338, 365.643472, 289.775598, 361.770894},
  {144.235423, 143.764571, 286.019603, 304.026350, 362.871051, 290.570008, 359.007524},
  {144.231268, 143.768727, 285.981744, 304.821491, 360.098180, 291.362598, 356.243545},
  {144.227231, 143.772765, 285.944545, 305.614792, 357.324866, 292.153400, 353.478970},
  {144.223307, 143.776688, 285.907989, 306.406288, 354.551118, 292.942444, 350.713811},
  {144.219493, 143.780502, 285.872057, 307.196007, 351.776943, 293.729760, 347.948081},
  {144.215783, 143.784210, 285.836732, 307.983982, 349.002348, 294.515376, 345.181789},
  {144.212175, 143.787818, 285.801998, 308.770240, 346.227342, 295.299321, 342.414949},
  {144.208665, 143.791329, 285.767839, 309.554810, 343.451932, 296.081621, 339.647570},
  {144.205252, 143.794749, 285.734238, 310.337721, 340.676123, 296.862303, 336.879664},
  {144.201922, 143.798071, 285.701183, 311.118999, 337.899924, 297.641393, 334.111239},
  {144.198685, 143.801313, 285.668657, 311.898671, 335.123340, 298.418916, 331.342307},
  {144.195527, 143.804466, 285.636649, 312.676762, 332.346379, 299.194897, 328.572877},
  {144.192455, 143.807543, 285.605143, 313.453297, 329.569046, 299.969359, 325.802958},
  {144.189458, 143.810539, 285.574128, 314.228301, 326.791347, 300.742326, 323.032559},
  {144.186536, 143.813459, 285.543591, 315.001797, 324.013289, 301.513821, 320.261689},
  {144.183691, 143.816310, 285.513521, 315.773808, 321.234877, 302.283865, 317.490356},
  {144.180909, 143.819084, 285.483905, 316.544357, 318.456117, 303.052480, 314.718570},
  {144.178203, 143.821799, 285.454733, 317.313466, 315.677014, 303.819688, 311.946337},
  {144.175556, 143.824441, 285.425994, 318.081157, 312.897574, 304.585509, 309.173665},
  {144.064058, 143.718366, 285.397699, 318.846871, 310.119902, 305.349386, 306.402658},
  {143.643483, 143.303804, 285.369896, 319.608994, 307.349965, 306.109707, 303.639272},
  {142.822523, 142.489625, 285.342648, 320.365439, 304.595496, 306.864388, 300.891223},
  {141.601202, 141.275803, 285.316013, 321.114130, 301.864229, 307.611360, 298.166232},
  {139.979548, 139.662327, 285.290048, 321.853008, 299.163901, 308.348563, 295.472024},
  {137.957566, 137.649161, 285.264805, 322.580024, 296.502254, 309.073954, 292.816327},
  {135.535289, 135.236303, 285.240333, 323.293141, 293.887032, 309.785497, 290.206874},
  {132.712721, 132.423726, 285.216678, 323.990330, 291.325978, 310.481169, 287.651397},
  {129.489879, 129.211418, 285.193886, 324.669577, 288.826843, 311.158954, 285.157637},
  {125.866789, 125.599382, 285.171998, 325.328871, 286.397376, 311.816847, 282.733331},
  {121.843442, 121.587588, 285.151056, 325.966211, 284.045329, 312.452850, 280.386223},
  {117.419855, 117.176035, 285.131099, 326.579604, 281.778455, 313.064969, 278.124056},
  {112.704927, 112.473388, 285.112148, 327.167628, 279.602406, 313.651784, 275.952477},
  {107.898870, 107.679450, 285.094188, 327.729905, 277.518969, 314.212918, 273.873277},
  {103.092972, 102.885345, 285.077193, 328.266531, 275.528166, 314.748464, 271.886483},
  {98.287229, 98.091091, 285.061139, 328.777595, 273.630018, 315.258507, 269.992123},
  {93.481625, 93.296693, 285.046002, 329.263179, 271.824544, 315.743131, 268.190221},
  {88.676155, 88.502165, 285.031761, 329.723363, 270.111765, 316.202412, 266.480802},
  {83.870804, 83.707512, 285.018396, 330.158219, 268.491695, 316.636421, 264.863887},
  {79.065571, 78.912751, 285.005887, 330.567816, 266.964352, 317.045227, 263.339496},
  {94.445892, 53.895150, 281.685926, 331.024616, 265.131501, 317.314800, 262.295846},
  {103.824877, 34.897246, 276.041835, 331.342757, 263.079519, 317.420523, 261.605955},
  {90.962296, 38.185386, 271.720957, 331.464047, 261.264321, 317.470362, 260.843875},
  {78.585995, 40.977927, 268.642321, 331.467442, 259.692605, 317.471373, 260.024318},
  {67.324406, 42.648208, 266.622441, 331.410572, 258.347318, 317.434890, 259.172134},
  {57.310367, 43.066213, 265.456520, 331.330467, 257.203914, 317.374462, 258.312932},
  {48.464175, 42.313848, 264.953107, 331.248756, 256.238080, 317.303033, 257.469675},
  {40.650558, 40.527759, 264.943056, 331.176675, 255.428271, 317.231169, 256.662312},
  {33.738391, 37.839825, 265.278764, 331.118831, 254.755987, 317.166334, 255.908298},
  {27.612744, 34.365334, 265.831473, 331.075812, 254.205410, 317.112848, 255.223075},
  {22.259711, 30.335887, 266.492521, 331.045878, 253.761223, 317.072102, 254.617727},
  {17.722554, 26.108213, 267.178900, 331.026234, 253.407317, 317.043201, 254.096363},
  {13.939924, 21.926052, 267.832577, 331.014057, 253.128785, 317.024073, 253.658260},
  {10.790034, 17.911152, 268.415451, 331.006971, 252.913100, 317.012324, 253.300229},
  {8.172571, 14.163810, 268.905842, 331.003140, 252.749694, 317.005692, 253.017031},
  {6.007383, 10.764174, 269.295192, 331.001249, 252.629561, 317.002308, 252.801774},
  {4.232787, 7.773930, 269.585038, 331.000420, 252.544909, 317.000788, 252.646303},
  {2.803422, 5.238444, 269.784348, 331.000111, 252.488842, 317.000210, 252.541535},
  {1.687806, 3.189203, 269.907239, 331.000020, 252.455086, 317.000039, 252.477752},
  {0.865759, 1.646390, 269.971134, 331.000002, 252.437771, 317.000004, 252.444824},
  {0.325823, 0.621467, 269.995333, 331.000000, 252.431254, 317.000000, 252.432394},
  {0.062707, 0.119723, 270.000000, 331.000000, 252.430000, 317.000000, 252.430000},
  {0.000000, 0.000000, 270.000000, 331.000000, 252.430000, 317.000000, 252.430000}};
}}
