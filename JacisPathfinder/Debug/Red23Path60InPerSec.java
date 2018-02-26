/* Red23 Feed Switch from Front Start 4 aud */
//----------------------------------------------------------------------------
//
//  $Workfile: Red23Path60InPerSec.java$
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

public class Red23Path60InPerSec extends Path {
    public Red23Path60InPerSec() {
       kNumPoints =178;
       kPoints = new double[][]{
  {0.400000, 0.400000, 359.999627, 16.002755, 242.750000, 16.002664, 228.750000},
  {0.813452, 0.786548, 359.997425, 16.019024, 242.750000, 16.018395, 228.750000},
  {1.830273, 1.769727, 359.992469, 16.055630, 242.749996, 16.053790, 228.749996},
  {3.253839, 3.146161, 359.983656, 16.120706, 242.749983, 16.116713, 228.749983},
  {5.084173, 4.915827, 359.969876, 16.222390, 242.749942, 16.215029, 228.749943},
  {7.117945, 6.882055, 359.950568, 16.364749, 242.749843, 16.352670, 228.749848},
  {9.151820, 8.848180, 359.925715, 16.547785, 242.749645, 16.529634, 228.749657},
  {11.185827, 10.814173, 359.895295, 16.771501, 242.749296, 16.745917, 228.749319},
  {13.219996, 12.780004, 359.859281, 17.035901, 242.748729, 17.001517, 228.748772},
  {15.254355, 14.745644, 359.817642, 17.340987, 242.747869, 17.296428, 228.747940},
  {17.288935, 16.711064, 359.770343, 17.686763, 242.746626, 17.630647, 228.746739},
  {19.323766, 18.676232, 359.717341, 18.073234, 242.744898, 18.004168, 228.745069},
  {21.358877, 20.641121, 359.658592, 18.500406, 242.742572, 18.416985, 228.742821},
  {23.394298, 22.605699, 359.594045, 18.968282, 242.739521, 18.869089, 228.739872},
  {25.430059, 24.569938, 359.523643, 19.476868, 242.735605, 19.360473, 228.736089},
  {27.466190, 26.533806, 359.447326, 20.026169, 242.730672, 19.891128, 228.731324},
  {29.502721, 28.497274, 359.365029, 20.616192, 242.724557, 20.461043, 228.725417},
  {31.539683, 30.460311, 359.276681, 21.246942, 242.717081, 21.070206, 228.718196},
  {33.577105, 32.422887, 359.182207, 21.918423, 242.708050, 21.718605, 228.709476},
  {35.615019, 34.384972, 359.081527, 22.630642, 242.697258, 22.406226, 228.699057},
  {37.653455, 36.346535, 358.974554, 23.383602, 242.684485, 23.133052, 228.686727},
  {39.692443, 38.307545, 358.861198, 24.177310, 242.669493, 23.899066, 228.672258},
  {41.732014, 40.267971, 358.741365, 25.011767, 242.652033, 24.704249, 228.655411},
  {43.772200, 42.227783, 358.614952, 25.886978, 242.631839, 25.548580, 228.635929},
  {45.813030, 44.186950, 358.481856, 26.802945, 242.608629, 26.432036, 228.613543},
  {47.854536, 46.145440, 358.341964, 27.759668, 242.582106, 27.354590, 228.587967},
  {49.896749, 48.103224, 358.195162, 28.757147, 242.551955, 28.316215, 228.558900},
  {51.939699, 50.060270, 358.041329, 29.795381, 242.517845, 29.316881, 228.526025},
  {53.983417, 52.016548, 357.880339, 30.874366, 242.479430, 30.356553, 228.489009},
  {56.027934, 53.972027, 357.712060, 31.994096, 242.436342, 31.435195, 228.447502},
  {57.869500, 55.730456, 357.536977, 33.150491, 242.388372, 32.548847, 228.401306},
  {59.304219, 57.095733, 357.356210, 34.335398, 242.335534, 33.689628, 228.350435},
  {60.331875, 58.068073, 357.170915, 35.540660, 242.277929, 34.849665, 228.294992},
  {60.952243, 58.647703, 356.982286, 36.758118, 242.215760, 36.021093, 228.235173},
  {61.165084, 58.834861, 356.791554, 37.979615, 242.149329, 37.196054, 228.171273},
  {61.174088, 58.825856, 356.599349, 39.201063, 242.078806, 38.370615, 228.103458},
  {61.183091, 58.816852, 356.405670, 40.422445, 242.004159, 39.544761, 228.031698},
  {61.192092, 58.807850, 356.210517, 41.643748, 241.925357, 40.718477, 227.955966},
  {61.201089, 58.798853, 356.013891, 42.864953, 241.842367, 41.891748, 227.876234},
  {61.210080, 58.789860, 355.815793, 44.086044, 241.755159, 43.064558, 227.792474},
  {61.219065, 58.780875, 355.616225, 45.307005, 241.663700, 44.236891, 227.704658},
  {61.228040, 58.771898, 355.415186, 46.527817, 241.567959, 45.408731, 227.612758},
  {61.237005, 58.762932, 355.212681, 47.748464, 241.467905, 46.580061, 227.516746},
  {61.245958, 58.753978, 355.008709, 48.968926, 241.363507, 47.750865, 227.416596},
  {61.254897, 58.745039, 354.803274, 50.189185, 241.254733, 48.921126, 227.312279},
  {61.263820, 58.736115, 354.596379, 51.409223, 241.141552, 50.090826, 227.203768},
  {61.272725, 58.727209, 354.388026, 52.629020, 241.023934, 51.259948, 227.091036},
  {61.281610, 58.718323, 354.178218, 53.848557, 240.901848, 52.428473, 226.974057},
  {61.290473, 58.709459, 353.966959, 55.067813, 240.775263, 53.596385, 226.852803},
  {61.299312, 58.700619, 353.754252, 56.286768, 240.644149, 54.763664, 226.727247},
  {61.308126, 58.691804, 353.540103, 57.505401, 240.508476, 55.930292, 226.597364},
  {61.316912, 58.683017, 353.324516, 58.723692, 240.368213, 57.096251, 226.463127},
  {61.325668, 58.674261, 353.107495, 59.941618, 240.223332, 58.261520, 226.324509},
  {61.334391, 58.665536, 352.889047, 61.159158, 240.073802, 59.426081, 226.181486},
  {61.343080, 58.656846, 352.669175, 62.376289, 239.919594, 60.589914, 226.034031},
  {61.351733, 58.648193, 352.447887, 63.592990, 239.760679, 61.752999, 225.882119},
  {61.360346, 58.639578, 352.225189, 64.809236, 239.597029, 62.915316, 225.725725},
  {61.368919, 58.631005, 352.001088, 66.025005, 239.428613, 64.076845, 225.564823},
  {61.377448, 58.622475, 351.775590, 67.240273, 239.255405, 65.237564, 225.399390},
  {61.385931, 58.613991, 351.548704, 68.455015, 239.077376, 66.397454, 225.229400},
  {61.394366, 58.605555, 351.320436, 69.669207, 238.894499, 67.556492, 225.054830},
  {61.402750, 58.597170, 351.090796, 70.882825, 238.706745, 68.714658, 224.875655},
  {61.411081, 58.588838, 350.859792, 72.095843, 238.514088, 69.871929, 224.691852},
  {61.419357, 58.580561, 350.627433, 73.308235, 238.316501, 71.028285, 224.503398},
  {61.427574, 58.572342, 350.393729, 74.519975, 238.113958, 72.183702, 224.310269},
  {61.435732, 58.564184, 350.158690, 75.731038, 237.906432, 73.338159, 224.112443},
  {61.443826, 58.556089, 349.922325, 76.941396, 237.693898, 74.491632, 223.909897},
  {61.451854, 58.548060, 349.684646, 78.151022, 237.476330, 75.644100, 223.702610},
  {61.459815, 58.540098, 349.445664, 79.359890, 237.253703, 76.795539, 223.490560},
  {61.467704, 58.532208, 349.205390, 80.567971, 237.025994, 77.945926, 223.273725},
  {61.475520, 58.524391, 348.963837, 81.775237, 236.793177, 79.095238, 223.052085},
  {61.483261, 58.516649, 348.721016, 82.981661, 236.555229, 80.243451, 222.825619},
  {61.490923, 58.508987, 348.476941, 84.187212, 236.312127, 81.390540, 222.594306},
  {61.498503, 58.501405, 348.231625, 85.391863, 236.063848, 82.536483, 222.358126},
  {61.506000, 58.493907, 347.985082, 86.595584, 235.810370, 83.681255, 222.117062},
  {61.513410, 58.486496, 347.737325, 87.798345, 235.551670, 84.824832, 221.871092},
  {61.520732, 58.479174, 347.488370, 89.000117, 235.287728, 85.967188, 221.620199},
  {61.527961, 58.471944, 347.238232, 90.200869, 235.018523, 87.108300, 221.364365},
  {61.535096, 58.464807, 346.986925, 91.400570, 234.744034, 88.248142, 221.103572},
  {61.542135, 58.457768, 346.734466, 92.599190, 234.464241, 89.386690, 220.837802},
  {61.549073, 58.450829, 346.480871, 93.796697, 234.179126, 90.523918, 220.567039},
  {61.555910, 58.443992, 346.226157, 94.993062, 233.888670, 91.659800, 220.291267},
  {61.562641, 58.437259, 345.970341, 96.188250, 233.592854, 92.794312, 220.010468},
  {61.569266, 58.430634, 345.713440, 97.382232, 233.291660, 93.927428, 219.724629},
  {61.575780, 58.424119, 345.455473, 98.574975, 232.985073, 95.059122, 219.433734},
  {61.582182, 58.417716, 345.196457, 99.766446, 232.673075, 96.189368, 219.137769},
  {61.588469, 58.411428, 344.936413, 100.956613, 232.355651, 97.318141, 218.836720},
  {61.594638, 58.405258, 344.675358, 102.145444, 232.032786, 98.445413, 218.530573},
  {61.600687, 58.399208, 344.413314, 103.332904, 231.704465, 99.571160, 218.219315},
  {61.606614, 58.393280, 344.150298, 104.518962, 231.370675, 100.695355, 217.902935},
  {61.612416, 58.387477, 343.886333, 105.703585, 231.031401, 101.817971, 217.581420},
  {61.618092, 58.381802, 343.621439, 106.886737, 230.686633, 102.938982, 217.254759},
  {61.623637, 58.376255, 343.355637, 108.068387, 230.336357, 104.058363, 216.922941},
  {61.629051, 58.370841, 343.088949, 109.248500, 229.980562, 105.176085, 216.585957},
  {61.634331, 58.365560, 342.821396, 110.427042, 229.619239, 106.292124, 216.243796},
  {61.639474, 58.360416, 342.553001, 111.603979, 229.252376, 107.406451, 215.896450},
  {61.644480, 58.355410, 342.283787, 112.779278, 228.879966, 108.519041, 215.543911},
  {61.649344, 58.350545, 342.013776, 113.952904, 228.502000, 109.629868, 215.186169},
  {61.654067, 58.345821, 341.742992, 115.124823, 228.118470, 110.738904, 214.823218},
  {61.658645, 58.341243, 341.471459, 116.295001, 227.729369, 111.846123, 214.455052},
  {61.663076, 58.336811, 341.199200, 117.463403, 227.334691, 112.951498, 214.081664},
  {61.667360, 58.332527, 340.926240, 118.629995, 226.934430, 114.055003, 213.703049},
  {61.671493, 58.328393, 340.652603, 119.794742, 226.528582, 115.156612, 213.319201},
  {61.675475, 58.324410, 340.378315, 120.957611, 226.117143, 116.256298, 212.930117},
  {61.679303, 58.320582, 340.103399, 122.118566, 225.700109, 117.354034, 212.535793},
  {61.682977, 58.316908, 339.827883, 123.277574, 225.277479, 118.449794, 212.136226},
  {61.686494, 58.313390, 339.551790, 124.434600, 224.849250, 119.543552, 211.731413},
  {61.689853, 58.310031, 339.275148, 125.589610, 224.415421, 120.635282, 211.321352},
  {61.693052, 58.306831, 338.997981, 126.742570, 223.975992, 121.724958, 210.906043},
  {61.696092, 58.303791, 338.720317, 127.893445, 223.530964, 122.812553, 210.485485},
  {61.698969, 58.300913, 338.442182, 129.042201, 223.080339, 123.898042, 210.059677},
  {61.701684, 58.298198, 338.163603, 130.188805, 222.624117, 124.981399, 209.628621},
  {61.704235, 58.295647, 337.884606, 131.333224, 222.162303, 126.062599, 209.192318},
  {61.706621, 58.293260, 337.605218, 132.475422, 221.694900, 127.141615, 208.750770},
  {61.708841, 58.291039, 337.325467, 133.615367, 221.221912, 128.218424, 208.303978},
  {61.710895, 58.288985, 337.045380, 134.753025, 220.743344, 129.292998, 207.851948},
  {61.712782, 58.287098, 336.764983, 135.888364, 220.259203, 130.365314, 207.394681},
  {61.714502, 58.285378, 336.484306, 137.021351, 219.769495, 131.435347, 206.932184},
  {61.716053, 58.283827, 336.203374, 138.151952, 219.274228, 132.503072, 206.464460},
  {61.717435, 58.282445, 335.922216, 139.280135, 218.773409, 133.568464, 205.991515},
  {61.718649, 58.281231, 335.640859, 140.405868, 218.267049, 134.631500, 205.513357},
  {61.719693, 58.280186, 335.359331, 141.529120, 217.755156, 135.692155, 205.029991},
  {61.720568, 58.279311, 335.077660, 142.649858, 217.237742, 136.750406, 204.541425},
  {52.802992, 67.163143, 336.253083, 143.610331, 216.798705, 137.972567, 203.984041},
  {49.053054, 70.942056, 338.044795, 144.514348, 216.417609, 139.280006, 203.432939},
  {48.622981, 71.371737, 339.906888, 145.422027, 216.068614, 140.612372, 202.920716},
  {48.202404, 71.791917, 341.837807, 146.332812, 215.752609, 141.968900, 202.450118},
  {47.797908, 72.196017, 343.834921, 147.246164, 215.470396, 143.348483, 202.023907},
  {47.416713, 72.576826, 345.894415, 148.161575, 215.222685, 144.749641, 201.644810},
  {47.066462, 72.926712, 348.011226, 149.078582, 215.010085, 146.170501, 201.315448},
  {46.754928, 73.237914, 350.179017, 149.996779, 214.833102, 147.608794, 201.038265},
  {46.489653, 73.502899, 352.390218, 150.915826, 214.692147, 149.061867, 200.815445},
  {46.277545, 73.714771, 354.636129, 151.835445, 214.587531, 150.526718, 200.648835},
  {46.124457, 73.867687, 356.907092, 152.755421, 214.519479, 152.000048, 200.539872},
  {46.034799, 73.957243, 359.192727, 153.675583, 214.488132, 153.478336, 200.489522},
  {46.011229, 73.980786, 1.482219, 154.595792, 214.493555, 154.957926, 200.498239},
  {46.054453, 73.937612, 3.764637, 155.515914, 214.535736, 156.435127, 200.565946},
  {46.163176, 73.829011, 6.029264, 156.435804, 214.614590, 157.906314, 200.692032},
  {46.334200, 73.658180, 8.265904, 157.355280, 214.729950, 159.368023, 200.875389},
  {46.562651, 73.429982, 10.465160, 158.274108, 214.881567, 160.817034, 201.114449},
  {46.842323, 73.150613, 12.618650, 159.191993, 215.069100, 162.250445, 201.407260},
  {47.166077, 72.827202, 14.719159, 160.108574, 215.292113, 163.665713, 201.751553},
  {47.492670, 72.416506, 16.759311, 161.022784, 215.549873, 165.059711, 202.144529},
  {47.652835, 71.687834, 18.726699, 161.930473, 215.840422, 166.425234, 202.581571},
  {47.504680, 70.467584, 20.606322, 162.825107, 216.160278, 167.752336, 203.055988},
  {47.029743, 68.774177, 22.386200, 163.700235, 216.505052, 169.032103, 203.560123},
  {46.210630, 66.624969, 24.057196, 164.549533, 216.869555, 170.256610, 204.085609},
  {40.022256, 69.097181, 26.437206, 165.275529, 217.206662, 171.508564, 204.670743},
  {25.417377, 79.711576, 30.882378, 165.722764, 217.448319, 172.908647, 205.433200},
  {12.143092, 88.962431, 37.173390, 165.926133, 217.581071, 174.385341, 206.425722},
  {8.260996, 105.303269, 46.478513, 165.807011, 217.466583, 175.958637, 207.825811},
  {37.900807, 130.824817, 60.322335, 165.361317, 216.853439, 177.524861, 209.921759},
  {66.285376, 155.061150, 78.516090, 164.899586, 215.610738, 178.619315, 212.823440},
  {64.967247, 149.781622, 96.163166, 164.835704, 214.312965, 178.754787, 215.816008},
  {36.454194, 117.442102, 108.785134, 164.987549, 213.599868, 178.241809, 218.108149},
  {9.300261, 86.392412, 116.623605, 165.055667, 213.426785, 177.571243, 219.700569},
  {8.053930, 65.081569, 121.292735, 164.975612, 213.566562, 176.938958, 220.838312},
  {18.365205, 50.783765, 123.946475, 164.776376, 213.875135, 176.390211, 221.692989},
  {24.652046, 40.501667, 125.243814, 164.495618, 214.280430, 175.929472, 222.359228},
  {28.702706, 32.452322, 125.550725, 164.162488, 214.747936, 175.552903, 222.887865},
  {31.497892, 25.657045, 125.072643, 163.797849, 215.261635, 175.255788, 223.306238},
  {33.576174, 19.578049, 123.926861, 163.417042, 215.814744, 175.033552, 223.628622},
  {35.224896, 13.928323, 122.183645, 163.032349, 216.404937, 174.881182, 223.861823},
  {36.567262, 8.584857, 119.893099, 162.654850, 217.031324, 174.792245, 224.008690},
  {37.597212, 3.553895, 117.106336, 162.295669, 217.691936, 174.757942, 224.070943},
  {38.194327, 1.044281, 113.894207, 161.966450, 218.381239, 174.766578, 224.051927},
  {38.144522, 4.994699, 110.362657, 161.678778, 219.087813, 174.803903, 223.959268},
  {37.187639, 8.037619, 106.660276, 161.442402, 219.793004, 174.854703, 223.806753},
  {35.096878, 9.946119, 102.972821, 161.262733, 220.471558, 174.905406, 223.614401},
  {31.764836, 10.612962, 99.503621, 161.138819, 221.094653, 174.946671, 223.406192},
  {27.249651, 10.096578, 96.446428, 161.063153, 221.634367, 174.974635, 223.206206},
  {21.752228, 8.598149, 93.962021, 161.023670, 222.067617, 174.990211, 223.034950},
  {15.687820, 6.448607, 92.150054, 161.006935, 222.380926, 174.997079, 222.906160},
  {10.021408, 4.212931, 90.984938, 161.001451, 222.581279, 174.999383, 222.821933},
  {5.530593, 2.353055, 90.339649, 161.000172, 222.691884, 174.999926, 222.774875},
  {2.353526, 1.006994, 90.064586, 161.000006, 222.738954, 174.999997, 222.754736},
  {0.552292, 0.236781, 90.000000, 161.000000, 222.750000, 175.000000, 222.750000},
  {0.000000, 0.000000, 90.000000, 161.000000, 222.750000, 175.000000, 222.750000}};
}}
