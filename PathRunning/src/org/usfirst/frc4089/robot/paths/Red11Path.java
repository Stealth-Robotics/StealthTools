/* Red11 Feed Switch from Side start 1 aud */
package org.usfirst.frc4089.robot.paths;
public class Red11Path extends Path {
    public Red11Path() {
       kNumPoints =84;
       kPoints = new double[][]{
  {0.376720, 0.376720, 0.002274, 16.002462, 50.750000, 16.003017, 36.750000},
  {0.677049, 0.829831, 0.014779, 16.016003, 50.750002, 16.019614, 36.750003},
  {1.523394, 1.867086, 0.042910, 16.046471, 50.750017, 16.056956, 36.750021},
  {2.708371, 3.319149, 0.092903, 16.100638, 50.750082, 16.123338, 36.750100},
  {4.232127, 5.185872, 0.170968, 16.185280, 50.750277, 16.227056, 36.750339},
  {6.094907, 7.467010, 0.283277, 16.307177, 50.750760, 16.376395, 36.750931},
  {8.297093, 10.162180, 0.435936, 16.473116, 50.751801, 16.579634, 36.752207},
  {10.839240, 13.270826, 0.634964, 16.689891, 50.753827, 16.845039, 36.754687},
  {13.722137, 16.792157, 0.886249, 16.964310, 50.757471, 17.180853, 36.759146},
  {16.946874, 20.725078, 1.195499, 17.303191, 50.763628, 17.595286, 36.766676},
  {20.514938, 25.068099, 1.568181, 17.713371, 50.773524, 18.096502, 36.778768},
  {24.428327, 29.819217, 2.009433, 18.201699, 50.788777, 18.692596, 36.797386},
  {28.519865, 34.768882, 2.520924, 18.771651, 50.811325, 19.387430, 36.824874},
  {32.622393, 39.707535, 3.100853, 19.423313, 50.843326, 20.180625, 36.863824},
  {36.738625, 44.632460, 3.746975, 20.156774, 50.887217, 21.071680, 36.917144},
  {40.871672, 49.540546, 4.456537, 20.972113, 50.945698, 22.059952, 36.988026},
  {45.025038, 54.428289, 5.226209, 21.869399, 51.021715, 23.144633, 37.079915},
  {49.202608, 59.291807, 6.052029, 22.848687, 51.118434, 24.324728, 37.196462},
  {53.408599, 64.126887, 6.929341, 23.910008, 51.239213, 25.599041, 37.341474},
  {57.647493, 68.929051, 7.852759, 25.053373, 51.387571, 26.966163, 37.518857},
  {61.923946, 73.693650, 8.816134, 26.278764, 51.567144, 28.424462, 37.732550},
  {66.242672, 78.415978, 9.812545, 27.586139, 51.781641, 29.972092, 37.986452},
  {70.608308, 83.091408, 10.834314, 28.975429, 52.034793, 31.607003, 38.284345},
  {75.025280, 87.715523, 11.873039, 30.446550, 52.330294, 33.326962, 38.629812},
  {79.497664, 92.284260, 12.919651, 31.999407, 52.671741, 35.129589, 39.026157},
  {84.029066, 96.794020, 13.964491, 33.633913, 53.062564, 37.012400, 39.476327},
  {88.622533, 101.241766, 14.997403, 35.350009, 53.505956, 38.972863, 39.982831},
  {93.280506, 105.625067, 16.007832, 37.147688, 54.004807, 41.008450, 40.547671},
  {98.004815, 109.942095, 16.984924, 39.027032, 54.561621, 43.116713, 41.172278},
  {102.796736, 114.191578, 17.917615, 40.988253, 55.178453, 45.295341, 41.857454},
  {107.477184, 118.175889, 18.793324, 43.028331, 55.855668, 47.538506, 42.602052},
  {111.861719, 121.702746, 19.598830, 45.141058, 56.591603, 49.837111, 43.402703},
  {115.942293, 124.780176, 20.322225, 47.320477, 57.383549, 52.182669, 44.254989},
  {119.710540, 127.416520, 20.952971, 49.560905, 58.227782, 54.567326, 45.153543},
  {123.158109, 129.620101, 21.481894, 51.856943, 59.119622, 56.983843, 46.092155},
  {126.276911, 131.398982, 21.901143, 54.203474, 60.053504, 59.425562, 47.063901},
  {129.059266, 132.760816, 22.204119, 56.595641, 61.023067, 61.886344, 48.061259},
  {131.497958, 133.712798, 22.385406, 59.028811, 62.021249, 64.360500, 49.076246},
  {133.586186, 134.261711, 22.440699, 61.498520, 63.040388, 66.842699, 50.100537},
  {135.317440, 134.414048, 22.366755, 64.000403, 64.072333, 69.327877, 51.125595},
  {136.685315, 134.176206, 22.161381, 66.530108, 65.108548, 71.811141, 52.142797},
  {137.549328, 133.425565, 21.823846, 69.080715, 66.139241, 74.285275, 53.142604},
  {137.899391, 132.172217, 21.355069, 71.645023, 67.154537, 76.743074, 54.115753},
  {137.860276, 130.554505, 20.757081, 74.217942, 68.145632, 79.179634, 55.054352},
  {137.423547, 128.580881, 20.033294, 76.793938, 69.103931, 81.589864, 55.951019},
  {136.579821, 126.260753, 19.188659, 79.366917, 70.021208, 83.968433, 56.799028},
  {135.318743, 123.604506, 18.229825, 81.930131, 70.889770, 86.309742, 57.592440},
  {133.629105, 120.623386, 17.165277, 84.476094, 71.702645, 88.607901, 58.326241},
  {131.499113, 117.329224, 16.005436, 86.996537, 72.453760, 90.856737, 58.996462},
  {128.916821, 113.734005, 14.762681, 89.482387, 73.138135, 93.049811, 59.600281},
  {125.870700, 109.849296, 13.451283, 91.923797, 73.752050, 95.180456, 60.136097},
  {122.350297, 105.685576, 12.087225, 94.310216, 74.293199, 97.241823, 60.603579},
  {118.346924, 101.251554, 10.687915, 96.630507, 74.760796, 99.226938, 61.003669},
  {113.996890, 96.674009, 9.269981, 98.875914, 75.156095, 101.131129, 61.338931},
  {109.503579, 92.126239, 7.847590, 101.041570, 75.482209, 102.953108, 61.613322},
  {104.932335, 87.656493, 6.433507, 103.123922, 75.743223, 104.692623, 61.831387},
  {100.288653, 83.259257, 5.039598, 105.119638, 75.943823, 106.349457, 61.997944},
  {95.579039, 78.928003, 3.676660, 107.025685, 76.089173, 107.923446, 62.117987},
  {90.810714, 74.655491, 2.354308, 108.839381, 76.184782, 109.414485, 62.196599},
  {85.991287, 70.434092, 1.080908, 110.558433, 76.236378, 110.822533, 62.238869},
  {80.289152, 67.083574, 0.000000, 112.164158, 76.250000, 112.164158, 62.250000},
  {69.173019, 69.173019, 0.000000, 113.547618, 76.250000, 113.547618, 62.250000},
  {64.652380, 64.652380, 0.000000, 114.840666, 76.250000, 114.840666, 62.250000},
  {60.131740, 60.131740, 0.000000, 116.043301, 76.250000, 116.043301, 62.250000},
  {55.611100, 55.611100, 0.000000, 117.155523, 76.250000, 117.155523, 62.250000},
  {51.090460, 51.090460, 0.000000, 118.177332, 76.250000, 118.177332, 62.250000},
  {46.569820, 46.569820, 0.000000, 119.108728, 76.250000, 119.108728, 62.250000},
  {42.049181, 42.049181, 0.000000, 119.949712, 76.250000, 119.949712, 62.250000},
  {37.528541, 37.528541, 0.000000, 120.700283, 76.250000, 120.700283, 62.250000},
  {33.007901, 33.007901, 0.000000, 121.360441, 76.250000, 121.360441, 62.250000},
  {28.487261, 28.487261, 0.000000, 121.930186, 76.250000, 121.930186, 62.250000},
  {24.098172, 24.098172, 0.000000, 122.412150, 76.250000, 122.412150, 62.250000},
  {20.028994, 20.028994, 0.000000, 122.812730, 76.250000, 122.812730, 62.250000},
  {16.336536, 16.336536, 0.000000, 123.139460, 76.250000, 123.139460, 62.250000},
  {13.020797, 13.020797, 0.000000, 123.399876, 76.250000, 123.399876, 62.250000},
  {10.081779, 10.081779, 0.000000, 123.601512, 76.250000, 123.601512, 62.250000},
  {7.519480, 7.519480, 0.000000, 123.751901, 76.250000, 123.751901, 62.250000},
  {5.333902, 5.333902, 0.000000, 123.858579, 76.250000, 123.858579, 62.250000},
  {3.525043, 3.525043, 0.000000, 123.929080, 76.250000, 123.929080, 62.250000},
  {2.092905, 2.092905, 0.000000, 123.970938, 76.250000, 123.970938, 62.250000},
  {1.037486, 1.037486, 0.000000, 123.991688, 76.250000, 123.991688, 62.250000},
  {0.358788, 0.358788, 0.000000, 123.998864, 76.250000, 123.998864, 62.250000},
  {0.056809, 0.056809, 0.000000, 124.000000, 76.250000, 124.000000, 62.250000},
  {0.000000, 0.000000, 0.000000, 124.000000, 76.250000, 124.000000, 62.250000}};
}}
