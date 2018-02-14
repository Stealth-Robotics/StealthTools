/* Red17 Feed cross switch from side */
//----------------------------------------------------------------------------////  $Workfile: Red17Path.java$////  $Revision: X$////  Project:    Paths////                            Copyright (c) 2018//                               Cedarcrest High School//                            All Rights Reserved////  Modification History://  $Log://  $////----------------------------------------------------------------------------//////----------------------------------------------------------------------------//    Parameters Used//----------------------------------------------------------------------------//   Time Slice= 0.020000//   Max Vel   = 144.000000//   Max Accel = 600.000000//   Max Jerk  = 1000.000000package org.usfirst.frc4089.paths;
public class Red17Path extends Path {
    public Red17Path() {
       kNumPoints =222;
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
  {152.660297, 135.229515, 335.291087, 143.513264, 63.337095, 137.661147, 50.618891},
  {135.931340, 152.062287, 336.611452, 145.995843, 62.229063, 140.438340, 49.379387},
  {135.682634, 152.310594, 337.972501, 148.499078, 61.181340, 143.248356, 48.203285},
  {135.435695, 152.557125, 339.373944, 151.022242, 60.196060, 146.090500, 47.093468},
  {135.192428, 152.799979, 340.815180, 153.564495, 59.275329, 148.963865, 46.052841},
  {134.954932, 153.037060, 342.295263, 156.124888, 58.421208, 151.867322, 45.084299},
  {134.725486, 153.266095, 343.812877, 158.702358, 57.635694, 154.799504, 44.190705},
  {134.506507, 153.484671, 345.366308, 161.295731, 56.920702, 157.758794, 43.374851},
  {134.300516, 153.690276, 346.953433, 163.903726, 56.278040, 160.743325, 42.639423},
  {134.110076, 153.880350, 348.571705, 166.524955, 55.709390, 163.750975, 41.986962},
  {133.937732, 154.052358, 350.218166, 169.157934, 55.216287, 166.779375, 41.419822},
  {133.785934, 154.203855, 351.889455, 171.801087, 54.800099, 169.825919, 40.940132},
  {133.656957, 154.332572, 353.581839, 174.452759, 54.462008, 172.887784, 40.549753},
  {133.552828, 154.436489, 355.291253, 177.111228, 54.202992, 175.961958, 40.250244},
  {133.475242, 154.513916, 357.013357, 179.774712, 54.023813, 179.045268, 40.042829},
  {133.425499, 154.563557, 358.743596, 182.441392, 53.925003, 182.134419, 39.928369},
  {133.404449, 154.584564, 0.477278, 185.109419, 53.906858, 185.226039, 39.907344},
  {133.412454, 154.576575, 2.209651, 187.776935, 53.969432, 188.316721, 39.979842},
  {133.449377, 154.539728, 3.935985, 190.442083, 54.112539, 191.403069, 40.145560},
  {133.514583, 154.474656, 5.651655, 193.103029, 54.335750, 194.481751, 40.403804},
  {133.606969, 154.382459, 7.352214, 195.757973, 54.638406, 197.549532, 40.753511},
  {133.725009, 154.264658, 9.033468, 198.405165, 55.019625, 200.603324, 41.193270},
  {133.866818, 154.123132, 10.691527, 201.042917, 55.478314, 203.640215, 41.721350},
  {134.030222, 153.960050, 12.322861, 203.669616, 56.013186, 206.657499, 42.335740},
  {134.212838, 153.777786, 13.924326, 206.283737, 56.622781, 209.652699, 43.034179},
  {134.412157, 153.578845, 15.493190, 208.883848, 57.305479, 212.623582, 43.814208},
  {134.625614, 153.365785, 17.027139, 211.468618, 58.059525, 215.568163, 44.673199},
  {134.850662, 153.141145, 18.524278, 214.036825, 58.883050, 218.484715, 45.608403},
  {135.084830, 152.907390, 19.983114, 216.587356, 59.774093, 221.371760, 46.616986},
  {135.325774, 152.666861, 21.402538, 219.119210, 60.730617, 224.228063, 47.696062},
  {135.571310, 152.421736, 22.781797, 221.631500, 61.750535, 227.052618, 48.842728},
  {135.819445, 152.174005, 24.120467, 224.123447, 62.831724, 229.844638, 50.054088},
  {137.805007, 149.781596, 25.100777, 226.626879, 63.984511, 232.565843, 51.306628},
  {142.057349, 145.942281, 25.418764, 229.196382, 65.196851, 235.205615, 52.552124},
  {141.986811, 146.012792, 25.748295, 231.757723, 66.423079, 237.839581, 53.813123},
  {141.912110, 146.087463, 26.090053, 234.310493, 67.663635, 240.467459, 55.090180},
  {141.832910, 146.166630, 26.444773, 236.854258, 68.918980, 243.088948, 56.383884},
  {141.748838, 146.250665, 26.813254, 239.388548, 70.189597, 245.703724, 57.694856},
  {141.659484, 146.339979, 27.196358, 241.912859, 71.475994, 248.311439, 59.023758},
  {141.564394, 146.435025, 27.595025, 244.426649, 72.778703, 250.911717, 60.371290},
  {141.463064, 146.536305, 28.010277, 246.929332, 74.098285, 253.504151, 61.738198},
  {141.354937, 146.644378, 28.443224, 249.420277, 75.435329, 256.088304, 63.125275},
  {141.239391, 146.759862, 28.895082, 251.898798, 76.790452, 258.663699, 64.533368},
  {141.115735, 146.883451, 29.367178, 254.364154, 78.164307, 261.229819, 65.963378},
  {140.983196, 147.015913, 29.860964, 256.815542, 79.557577, 263.786100, 67.416270},
  {140.840912, 147.158110, 30.378035, 259.252086, 80.970983, 266.331929, 68.893077},
  {140.687917, 147.311009, 30.920145, 261.672834, 82.405284, 268.866634, 70.394904},
  {140.523125, 147.475692, 31.489222, 264.076745, 83.861276, 271.389479, 71.922937},
  {140.345318, 147.653374, 32.087398, 266.462682, 85.339797, 273.899654, 73.478454},
  {140.153125, 147.845426, 32.717024, 268.829402, 86.841728, 276.396266, 75.062825},
  {139.944998, 148.053393, 33.380709, 271.175537, 88.367996, 278.878331, 76.677531},
  {139.719190, 148.279016, 34.081345, 273.499585, 89.919568, 281.344756, 78.324169},
  {139.473727, 148.524267, 34.822147, 275.799894, 91.497463, 283.794326, 80.004463},
  {139.206376, 148.791374, 35.606695, 278.074635, 93.102740, 286.225687, 81.720282},
  {138.914608, 149.082860, 36.438986, 280.321793, 94.736506, 288.637323, 83.473649},
  {138.595560, 149.401581, 37.323479, 282.539130, 96.399909, 291.027530, 85.266757},
  {138.245994, 149.750765, 38.265168, 284.724167, 98.094133, 293.394392, 87.101991},
  {137.862252, 150.134061, 39.269642, 286.874148, 99.820396, 295.735738, 88.981936},
  {137.440211, 150.555577, 40.343165, 288.986006, 101.579935, 298.049104, 90.909403},
  {136.975247, 151.019922, 41.492757, 291.056327, 103.373997, 300.331682, 92.887444},
  {136.462205, 151.532235, 42.726279, 293.081304, 105.203817, 302.580257, 94.919369},
  {135.895389, 152.098183, 44.052526, 295.056692, 107.070596, 304.791138, 97.008758},
  {135.268597, 152.723942, 45.481302, 296.977763, 108.975464, 306.960067, 99.159476},
  {134.575208, 153.416100, 47.023497, 298.839253, 110.919442, 309.082120, 101.375665},
  {133.808370, 154.181467, 48.691116, 300.635315, 112.903386, 311.151580, 103.661732},
  {132.961330, 155.026748, 50.497270, 302.359482, 114.927919, 313.161802, 106.022309},
  {132.027980, 155.957997, 52.456064, 304.004635, 116.993356, 315.105043, 108.462181},
  {131.003697, 156.979779, 54.582358, 305.562998, 119.099607, 316.972289, 110.986157},
  {129.886574, 158.093941, 56.891318, 307.026165, 121.246080, 318.753069, 113.598876},
  {128.679138, 159.297901, 59.397698, 308.385175, 123.431583, 320.435277, 116.304519},
  {127.390593, 160.582423, 62.114741, 309.630639, 125.654230, 322.005043, 119.106397},
  {126.039489, 161.928959, 65.052656, 310.752956, 127.911394, 323.446697, 122.006402},
  {124.656522, 163.306882, 68.216633, 311.742585, 130.199698, 324.742895, 125.004322},
  {123.286741, 164.671298, 71.604499, 312.590396, 132.515095, 325.875008, 128.097051},
  {121.990091, 165.962530, 75.204284, 313.288052, 134.853023, 326.823847, 131.277795},
  {120.838907, 167.108628, 78.992202, 313.828374, 137.208627, 327.570790, 134.535431},
  {119.911281, 168.031967, 82.931710, 314.205646, 139.576992, 328.099249, 137.854260},
  {119.280321, 168.659915, 86.974323, 314.415830, 141.953321, 328.396314, 141.214352},
  {119.001224, 168.937653, 91.062543, 314.456664, 144.332995, 328.454257, 144.592608},
  {119.099869, 168.839491, 95.134643, 314.327710, 146.711499, 328.271530, 147.964451},
  {119.567004, 168.374610, 99.130410, 314.030355, 149.084280, 327.852971, 151.305830},
  {120.360361, 167.584983, 102.996531, 313.567792, 151.446626, 327.209163, 154.595115},
  {121.413820, 166.536287, 106.690495, 312.944949, 153.793666, 326.355131, 157.814488},
  {122.650140, 165.305280, 110.182407, 312.168334, 156.120486, 325.308720, 160.950626},
  {123.993118, 163.967734, 113.454819, 311.245772, 158.422354, 324.089011, 163.994715},
  {125.376303, 162.589774, 116.501141, 310.186059, 160.694951, 322.715016, 166.941970},
  {126.747399, 161.223487, 119.323333, 308.998585, 162.934564, 321.204764, 169.790890},
  {128.069014, 159.906160, 121.929464, 307.692962, 165.138201, 319.574760, 172.542449},
  {129.317095, 158.661816, 124.331538, 306.278711, 167.303624, 317.839742, 175.199354},
  {130.478296, 157.503818, 126.543744, 304.765019, 169.429317, 316.012654, 177.765426},
  {131.547193, 156.437636, 128.581162, 303.160572, 171.514411, 314.104730, 180.245128},
  {132.523807, 155.463307, 130.458870, 301.473456, 173.558589, 312.125663, 182.643217},
  {133.411638, 154.577392, 132.191377, 299.711104, 175.561982, 310.083784, 184.964509},
  {134.216208, 153.774425, 133.792291, 297.880296, 177.525075, 307.986243, 187.213720},
  {134.944042, 153.047932, 135.274156, 295.987172, 179.448624, 305.839185, 189.395374},
  {135.601999, 152.391100, 136.648395, 294.037269, 181.333575, 303.647899, 191.513742},
  {136.196839, 151.797201, 137.925329, 292.035569, 183.181014, 301.416949, 193.572824},
  {136.734985, 151.259848, 139.114226, 289.986543, 184.992111, 299.150286, 195.576335},
  {137.222382, 150.773122, 140.223387, 287.894201, 186.768088, 296.851345, 197.527715},
  {137.664445, 150.331626, 141.260224, 285.762140, 188.510190, 294.523121, 199.430137},
  {138.066051, 149.930502, 142.231354, 283.593592, 190.219661, 292.168236, 201.286525},
  {138.431560, 149.565404, 143.142681, 281.391454, 191.897728, 289.788995, 203.099572},
  {138.764848, 149.232469, 143.999475, 279.158337, 193.545594, 287.387434, 204.871756},
  {139.069349, 148.928270, 144.806445, 276.896589, 195.164424, 284.965355, 206.605360},
  {139.348102, 148.649780, 145.567804, 274.608331, 196.755345, 282.524359, 208.302488},
  {139.603789, 148.394319, 146.287323, 272.295477, 198.319440, 280.065876, 209.965079},
  {139.838783, 148.159522, 146.968389, 269.959761, 199.857747, 277.591185, 211.594926},
  {140.055177, 147.943299, 147.614044, 267.602758, 201.371258, 275.101435, 213.193687},
  {140.254824, 147.743802, 148.227028, 265.225895, 202.860919, 272.597662, 214.762895},
  {140.439365, 147.559394, 148.809813, 262.830475, 204.327632, 270.080802, 216.303974},
  {140.610251, 147.388623, 149.364632, 260.417685, 205.772257, 267.551702, 217.818244},
  {140.768774, 147.230204, 149.893509, 257.988609, 207.195609, 265.011132, 219.306934},
  {140.916079, 147.082990, 150.398280, 255.544242, 208.598466, 262.459794, 220.771188},
  {141.053188, 146.945962, 150.880611, 253.085495, 209.981566, 259.898330, 222.212072},
  {141.181012, 146.818210, 151.342023, 250.613206, 211.345609, 257.327326, 223.630583},
  {141.268552, 146.665919, 151.783805, 248.128705, 212.690963, 254.747903, 225.027341},
  {141.119962, 146.284939, 152.206565, 245.636751, 214.016144, 252.164745, 226.401025},
  {140.570494, 145.504834, 152.610447, 243.145045, 215.318228, 249.585575, 227.748818},
  {139.619759, 144.325994, 152.995658, 240.661346, 216.594439, 247.018158, 229.068048},
  {138.267471, 142.748706, 153.362453, 238.193465, 217.842124, 244.470294, 230.356173},
  {136.513430, 140.773167, 153.711118, 235.749249, 219.058749, 241.949810, 231.610762},
  {134.357511, 138.399503, 154.041960, 233.336578, 220.241878, 239.464557, 232.829485},
  {131.799652, 135.627776, 154.355296, 230.963353, 221.389163, 237.022403, 234.010095},
  {128.839845, 132.457993, 154.651446, 228.637493, 222.498333, 234.631227, 235.150414},
  {125.478127, 128.890118, 154.930722, 226.366926, 223.567177, 232.298919, 236.248323},
  {95.482458, 151.037219, 159.479367, 224.615966, 224.329346, 229.523592, 237.440990},
  {83.261925, 154.743588, 165.332766, 223.028488, 224.832270, 226.573355, 238.376048},
  {81.640753, 147.120044, 170.694276, 221.431001, 225.170084, 223.694835, 238.985838},
  {81.122134, 138.061735, 175.356133, 219.820350, 225.365337, 220.953819, 239.319377},
  {81.119560, 128.484813, 179.233773, 218.199679, 225.440027, 218.386898, 239.438775},
  {81.072948, 118.947045, 182.334190, 216.578397, 225.416083, 216.008204, 239.404466},
  {80.652065, 109.778601, 184.718402, 214.968505, 225.315332, 213.816884, 239.267887},
  {79.717540, 101.119826, 186.470272, 213.381875, 225.158612, 211.804248, 239.069438},
  {78.246333, 92.994911, 187.677482, 211.828994, 224.964819, 209.958640, 238.839320},
  {76.270779, 85.372495, 188.422473, 210.318735, 224.750321, 208.268140, 238.599331},
  {73.841543, 78.202625, 188.779433, 208.858622, 224.528735, 206.721787, 238.364700},
  {71.009170, 71.435242, 188.814307, 207.455237, 224.310951, 205.309980, 238.145613},
  {67.816843, 65.027485, 188.585995, 206.114580, 224.105305, 204.024469, 237.948406},
  {64.298980, 58.945136, 188.147776, 204.842345, 223.917793, 202.858171, 237.776474},
  {60.482553, 53.161346, 187.548524, 203.644067, 223.752308, 201.804945, 237.630984},
  {56.389409, 47.654333, 186.833544, 202.525180, 223.610894, 200.859386, 237.511438},
  {52.038700, 42.404972, 186.045006, 201.490992, 223.493988, 200.016657, 237.416141},
  {47.448957, 37.394731, 185.222050, 200.546610, 223.400698, 199.272388, 237.342590},
  {42.639557, 32.604216, 184.400639, 199.696832, 223.329076, 198.622610, 237.287802},
  {37.631532, 28.012369, 183.613293, 198.946045, 223.276421, 198.063736, 237.248590},
  {32.485466, 23.623393, 182.887918, 198.297383, 223.239540, 197.592031, 237.221761},
  {27.461374, 19.577243, 182.242589, 197.748707, 223.214935, 197.200879, 237.204213},
  {22.788678, 15.979666, 181.685262, 197.293202, 223.199303, 196.881474, 237.193247},
  {18.499683, 12.798360, 181.218601, 196.923327, 223.189921, 196.625589, 237.186755},
  {14.620888, 10.006828, 180.840934, 196.630957, 223.184663, 196.425485, 237.183155},
  {11.173410, 7.583962, 180.547134, 196.407505, 223.181954, 196.273817, 237.181316},
  {8.173540, 5.513477, 180.329405, 196.244039, 223.180703, 196.163551, 237.180472},
  {5.633372, 3.783281, 180.177972, 196.131373, 223.180204, 196.087886, 237.180137},
  {3.561461, 2.384827, 180.081664, 196.060144, 223.180043, 196.040189, 237.180029},
  {1.963456, 1.312464, 180.028379, 196.020875, 223.180005, 196.013940, 237.180003},
  {0.842733, 0.562820, 180.005468, 196.004020, 223.180000, 196.002684, 237.180000},
  {0.200994, 0.134190, 180.000000, 196.000000, 223.180000, 196.000000, 237.180000},
  {0.000000, 0.000000, 180.000000, 196.000000, 223.180000, 196.000000, 237.180000}};
}}
