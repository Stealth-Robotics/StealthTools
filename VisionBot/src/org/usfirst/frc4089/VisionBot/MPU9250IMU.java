package org.usfirst.frc4089.VisionBot;

import edu.wpi.first.wpilibj.I2C;

public class MPU9250IMU {	
	static final int MPU9250_ADDRESS  = 0x68;  // Device address when ADO = 0
	static final int AK8963_ADDRESS   = 0x0C;
	static final int WHO_AM_I_AK8963  = 0x00; // should return = 0x48
	static final int INFO             = 0x01;
	static final int AK8963_ST1       = 0x02;  // data ready status bit 0
	static final int AK8963_XOUT_L    = 0x03;  // data
	static final int AK8963_XOUT_H    = 0x04;
	static final int AK8963_YOUT_L    = 0x05;
	static final int AK8963_YOUT_H    = 0x06;
	static final int AK8963_ZOUT_L    = 0x07;
	static final int AK8963_ZOUT_H    = 0x08;
	static final int AK8963_ST2       = 0x09 ; // Data overflow bit 3 and data read error status bit 2
	static final int AK8963_CNTL      = 0x0A;  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
	static final int AK8963_ASTC      = 0x0C;  // Self test control
	static final int AK8963_I2CDIS    = 0x0F;  // I2C disable
	static final int AK8963_ASAX      = 0x10;  // Fuse ROM x-axis sensitivity adjustment value
	static final int AK8963_ASAY      = 0x11;  // Fuse ROM y-axis sensitivity adjustment value
	static final int AK8963_ASAZ      = 0x12;  // Fuse ROM z-axis sensitivity adjustment value

	static final int SELF_TEST_X_GYRO = 0x00;
	static final int SELF_TEST_Y_GYRO = 0x01;
	static final int SELF_TEST_Z_GYRO = 0x02;

	/*static final int X_FINE_GAIN      = 0x03; // [7:0] fine gain
	static final int Y_FINE_GAIN      = 0x04;
	static final int Z_FINE_GAIN      = 0x05;
	static final int XA_OFFSET_H      = 0x06; // User-defined trim values for accelerometer
	static final int XA_OFFSET_L_TC   = 0x07;
	static final int YA_OFFSET_H      = 0x08;
	static final int YA_OFFSET_L_TC   = 0x09;
	static final int ZA_OFFSET_H      = 0x0A;
	static final int ZA_OFFSET_L_TC   = 0x0B;*/

	static final int SELF_TEST_X_ACCEL = 0x0D;
	static final int SELF_TEST_Y_ACCEL = 0x0E;
	static final int SELF_TEST_Z_ACCEL = 0x0F;

	static final int SELF_TEST_A       = 0x10;

	static final int XG_OFFSET_H       = 0x13;  // User-defined trim values for gyroscope
	static final int XG_OFFSET_L       = 0x14;
	static final int YG_OFFSET_H       = 0x15;
	static final int YG_OFFSET_L       = 0x16;
	static final int ZG_OFFSET_H       = 0x17;
	static final int ZG_OFFSET_L       = 0x18;
	static final int SMPLRT_DIV        = 0x19;
	static final int CONFIG            = 0x1A;
	static final int GYRO_CONFIG       = 0x1B;
	static final int ACCEL_CONFIG      = 0x1C;
	static final int ACCEL_CONFIG2     = 0x1D;
	static final int LP_ACCEL_ODR      = 0x1E;
	static final int WOM_THR           = 0x1F;

	// Duration counter threshold for motion interrupt generation, 1 kHz rate,
	// LSB = 1 ms
	static final int MOT_DUR           = 0x20;
	// Zero-motion detection threshold bits [7:0]
	static final int ZMOT_THR          = 0x21;
	// Duration counter threshold for zero motion interrupt generation, 16 Hz rate,
	// LSB = 64 ms
	static final int ZRMOT_DUR         = 0x22;

	static final int FIFO_EN            = 0x23;
	static final int I2C_MST_CTRL       = 0x24;
	static final int I2C_SLV0_ADDR      = 0x25;
	static final int I2C_SLV0_REG       = 0x26;
	static final int I2C_SLV0_CTRL      = 0x27;
	static final int I2C_SLV1_ADDR      = 0x28;
	static final int I2C_SLV1_REG       = 0x29;
	static final int I2C_SLV1_CTRL      = 0x2A;
	static final int I2C_SLV2_ADDR      = 0x2B;
	static final int I2C_SLV2_REG       = 0x2C;
	static final int I2C_SLV2_CTRL      = 0x2D;
	static final int I2C_SLV3_ADDR      = 0x2E;
	static final int I2C_SLV3_REG       = 0x2F;
	static final int I2C_SLV3_CTRL      = 0x30;
	static final int I2C_SLV4_ADDR      = 0x31;
	static final int I2C_SLV4_REG       = 0x32;
	static final int I2C_SLV4_DO        = 0x33;
	static final int I2C_SLV4_CTRL      = 0x34;
	static final int I2C_SLV4_DI        = 0x35;
	static final int I2C_MST_STATUS     = 0x36;
	static final int INT_PIN_CFG        = 0x37;
	static final int INT_ENABLE         = 0x38;
	static final int DMP_INT_STATUS     = 0x39;  // Check DMP interrupt
	static final int INT_STATUS         = 0x3A;
	static final int ACCEL_XOUT_H       = 0x3B;
	static final int ACCEL_XOUT_L       = 0x3C;
	static final int ACCEL_YOUT_H       = 0x3D;
	static final int ACCEL_YOUT_L       = 0x3E;
	static final int ACCEL_ZOUT_H       = 0x3F;
	static final int ACCEL_ZOUT_L       = 0x40;
	static final int TEMP_OUT_H         = 0x41;
	static final int TEMP_OUT_L         = 0x42;
	static final int GYRO_XOUT_H        = 0x43;
	static final int GYRO_XOUT_L        = 0x44;
	static final int GYRO_YOUT_H        = 0x45;
	static final int GYRO_YOUT_L        = 0x46;
	static final int GYRO_ZOUT_H        = 0x47;
	static final int GYRO_ZOUT_L        = 0x48;
	static final int EXT_SENS_DATA_00   = 0x49;
	static final int EXT_SENS_DATA_01   = 0x4A;
	static final int EXT_SENS_DATA_02   = 0x4B;
	static final int EXT_SENS_DATA_03   = 0x4C;
	static final int EXT_SENS_DATA_04   = 0x4D;
	static final int EXT_SENS_DATA_05   = 0x4E;
	static final int EXT_SENS_DATA_06   = 0x4F;
	static final int EXT_SENS_DATA_07   = 0x50;
	static final int EXT_SENS_DATA_08   = 0x51;
	static final int EXT_SENS_DATA_09   = 0x52;
	static final int EXT_SENS_DATA_10   = 0x53;
	static final int EXT_SENS_DATA_11   = 0x54;
	static final int EXT_SENS_DATA_12   = 0x55;
	static final int EXT_SENS_DATA_13   = 0x56;
	static final int EXT_SENS_DATA_14   = 0x57;
	static final int EXT_SENS_DATA_15   = 0x58;
	static final int EXT_SENS_DATA_16   = 0x59;
	static final int EXT_SENS_DATA_17   = 0x5A;
	static final int EXT_SENS_DATA_18   = 0x5B;
	static final int EXT_SENS_DATA_19   = 0x5C;
	static final int EXT_SENS_DATA_20   = 0x5D;
	static final int EXT_SENS_DATA_21   = 0x5E;
	static final int EXT_SENS_DATA_22   = 0x5F;
	static final int EXT_SENS_DATA_23   = 0x60;
	static final int MOT_DETECT_STATUS  = 0x61;
	static final int I2C_SLV0_DO        = 0x63;
	static final int I2C_SLV1_DO        = 0x64;
	static final int I2C_SLV2_DO        = 0x65;
	static final int I2C_SLV3_DO        = 0x66;
	static final int I2C_MST_DELAY_CTRL = 0x67;
	static final int SIGNAL_PATH_RESET  = 0x68;
	static final int MOT_DETECT_CTRL    = 0x69;
	static final int USER_CTRL          = 0x6A; // Bit 7 enable DMP, bit 3 reset DMP
	static final int PWR_MGMT_1         = 0x6B; // Device defaults to the SLEEP mode
	static final int PWR_MGMT_2         = 0x6C;
	static final int DMP_BANK           = 0x6D;  // Activates a specific bank in the DMP
	static final int DMP_RW_PNT         = 0x6E;  // Set read/write pointer to a specific start address in specified DMP bank
	static final int DMP_REG            = 0x6F;  // Register in DMP from which to read or to which to write
	static final int DMP_REG_1          = 0x70;
	static final int DMP_REG_2          = 0x71;
	static final int FIFO_COUNTH        = 0x72;
	static final int FIFO_COUNTL        = 0x73;
	static final int FIFO_R_W           = 0x74;
	static final int WHO_AM_I_MPU9250   = 0x75; // Should return = 0x71
	static final int XA_OFFSET_H        = 0x77;
	static final int XA_OFFSET_L        = 0x78;
	static final int YA_OFFSET_H        = 0x7A;
	static final int YA_OFFSET_L        = 0x7B;
	static final int ZA_OFFSET_H        = 0x7D;
	static final int ZA_OFFSET_L        = 0x7E;	
		
    static final  byte AFS_2G = 0;
    static final  byte AFS_4G = 1;
    static final  byte AFS_8G = 2;
    static final  byte AFS_16G = 3;

    static final  byte GFS_250DPS = 0;
    static final  byte GFS_500DPS = 1;
    static final  byte GFS_1000DPS = 2;
    static final  byte GFS_2000DPS = 3;
    
    static final  byte MFS_14BITS = 0; // 0.6 mG per LSB
    static final  byte MFS_16BITS = 1;      // 0.15 mG per LSB
   
      // Specify sensor full scale
      byte Gscale = GFS_250DPS;
      byte Ascale = AFS_2G;
      // Choose either 14-bit or 16-bit magnetometer resolution
      byte Mscale = MFS_16BITS;
      // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
      byte Mmode = 0x02;
	
	
    double pitch, yaw, roll;
    double temperature;   // Stores the real internal chip temperature in Celsius
    int tempCount;   // Temperature raw count output
    long delt_t = 0; // Used to control display output rate

    long count = 0, sumCount = 0; // used to control display output rate
    double deltat = 0.0f, sum = 0.0f;  // integration interval for both filter schemes
    long lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
    long Now = 0;        // used to calculate integration interval

    int[] gyroCount = new int[3];   // Stores the 16-bit signed gyro sensor output
    int[] magCount = new int[3];    // Stores the 16-bit signed magnetometer sensor output
    // Scale resolutions per LSB for the sensors
    double aRes, gRes, mRes;
    // Variables to hold latest sensor data values
    public double ax, ay, az, gx, gy, gz, mx, my, mz;
    // Factory mag calibration and mag bias
    double[] magCalibration = new double[3];
    double[] magbias = new double[3];
    // Bias corrections for gyro and accelerometer
    double[] gyroBias = new double[3];
    double[] accelBias = new double[3];
    double[] SelfTest = new double[6];
    // Stores the 16-bit signed accelerometer sensor output
    int[] accelCount = new int[3];	
	
    I2C i2cMPU9250;
    I2C i2cAK8963;
    
	public MPU9250IMU() {
		gyroBias[0] = 0;
		gyroBias[1] = 0;
		gyroBias[2] = 0;
		
		System.out.println("constructor");
		
		i2cMPU9250 = new I2C(I2C.Port.kOnboard, MPU9250_ADDRESS);
		i2cAK8963 = new I2C(I2C.Port.kOnboard, AK8963_ADDRESS);
		
		byte c = readByte(i2cMPU9250, WHO_AM_I_MPU9250);
		System.out.println("MPU9250 is(113):"+c);
		c = readByte(i2cAK8963, WHO_AM_I_AK8963);
		System.out.println("AK8963 is(72):"+c);
	}
	
	void readGyroData(int[] destination)
	{
	  byte[] rawData = new byte[6];  // x/y/z gyro register data stored here
	  readBytes(i2cMPU9250, GYRO_XOUT_H, 6, rawData);  // Read the six raw data registers sequentially into data array
	  destination[0] = ((int)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
	  destination[1] = ((int)rawData[2] << 8) | rawData[3] ;  
	  destination[2] = ((int)rawData[4] << 8) | rawData[5] ; 
	}
	
	public void calcGyroData()
	{
        readGyroData(gyroCount);  // Read the x/y/z adc values
        getGres();
        gx = (float)gyroCount[0]*gRes;
        gy = (float)gyroCount[1]*gRes;
        gz = (float)gyroCount[2]*gRes;
	}
	
	public void calcMagData()
	{
	    readMagData(magCount);  // Read the x/y/z adc values
	    getMres();
	    // User environmental x-axis correction in milliGauss, should be
	    // automatically calculated
	    magbias[0] = +470.;
	    // User environmental x-axis correction in milliGauss TODO axis??
	    magbias[1] = +120.;
	    // User environmental x-axis correction in milliGauss
	    magbias[2] = +125.;

	    // Calculate the magnetometer values in milliGauss
	    // Include factory calibration per data sheet and user environmental
	    // corrections
	    // Get actual magnetometer value, this depends on scale being set
	    mx = (float)magCount[0]*mRes*magCalibration[0] -
	               magbias[0];
	    my = (float)magCount[1]*mRes*magCalibration[1] -
	               magbias[1];
	    mz = (float)magCount[2]*mRes*magCalibration[2] -
	               magbias[2];
	}
	
	// Calculate the time the last update took for use in the quaternion filters
	public void updateTime()
	{
	  //Now = micros();
	  Now = 0;
		
	  // Set integration time by time elapsed since last filter update
	  deltat = ((Now - lastUpdate) / 1000000.0f);
	  lastUpdate = Now;

	  sum += deltat; // sum for averaging filter update rate
	  sumCount++;
	}

	void initAK8963(double[] destination)
	{
	  // First extract the factory calibration for each magnetometer axis
	  byte[] rawData = new byte[3];  // x/y/z gyro calibration data stored here
	  writeByte(i2cAK8963, AK8963_CNTL, 0x00); // Power down magnetometer  
	  Threadsleep(10);
	  writeByte(i2cAK8963, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	  Threadsleep(10);
	  readBytes(i2cAK8963, AK8963_ASAX, 3, rawData);  // Read the x-, y-, and z-axis calibration values
	  destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
	  destination[1] =  (float)(rawData[1] - 128)/256. + 1.;  
	  destination[2] =  (float)(rawData[2] - 128)/256. + 1.; 
	  writeByte(i2cAK8963, AK8963_CNTL, 0x00); // Power down magnetometer  
	  Threadsleep(10);
	  // Configure the magnetometer for continuous read and highest resolution
	  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates

	  writeByte(i2cAK8963, AK8963_CNTL, (byte)(Mscale << 4 | Mmode)); // Set magnetometer data resolution and sample ODR
	  Threadsleep(10);
	}

	void initMPU9250()
	{  
	 // wake up device
	  writeByte(i2cMPU9250, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
	  Threadsleep(100);

	 // get stable time source
	  writeByte(i2cMPU9250, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
	  Threadsleep(200);
	  
	 // Configure Gyro and Thermometer
	 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
	 // minimum Threadsleep time for this setting is 5.9 ms, which means sensor fusion update rates cannot
	 // be higher than 1 / 0.0059 = 170 Hz
	 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
	  writeByte(i2cMPU9250, CONFIG, 0x03);  

	 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	  writeByte(i2cMPU9250, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
	                                    // determined inset in CONFIG above
	 
	 // Set gyroscope full scale range
	 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	  int c = readByte(i2cMPU9250, GYRO_CONFIG); // get current GYRO_CONFIG register value
	 // c = c & ~0xE0; // Clear self-test bits [7:5] 
	  c = c & ~0x02; // Clear Fchoice bits [1:0] 
	  c = c & ~0x18; // Clear AFS bits [4:3]
	  c = c | Gscale << 3; // Set full scale range for the gyro
	 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
	  writeByte(i2cMPU9250, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register
	  
	 // Set accelerometer full-scale range configuration
	  c = readByte(i2cMPU9250, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
	 // c = c & ~0xE0; // Clear self-test bits [7:5] 
	  c = c & ~0x18;  // Clear AFS bits [4:3]
	  c = c | Ascale << 3; // Set full scale range for the accelerometer 
	  writeByte(i2cMPU9250, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

	 // Set accelerometer sample rate configuration
	 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	  c = readByte(i2cMPU9250, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
	  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
	  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	  writeByte(i2cMPU9250, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
	 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
	 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	  // Configure Interrupts and Bypass Enable
	  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
	  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
	  // can join the I2C bus and all can be controlled by the Arduino as master
	   writeByte(i2cMPU9250, INT_PIN_CFG, 0x22);    
	   writeByte(i2cMPU9250, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
	   Threadsleep(100);
	   calibrateMPU9250(gyroBias, accelBias);
	   initAK8963(magCalibration);
	}	
	
	void getMres() {
		  switch (Mscale)
		  {
		  // Possible magnetometer scales (and their register bit settings) are:
		  // 14 bit resolution (0) and 16 bit resolution (1)
		    case MFS_14BITS:
		          mRes = 10.*4912./8190.; // Proper scale to return milliGauss
		          break;
		    case MFS_16BITS:
		          mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
		          break;
		  }
		}

		void getGres() {
		  switch (Gscale)
		  {
		  // Possible gyro scales (and their register bit settings) are:
		  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
		        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		    case GFS_250DPS:
		          gRes = 250.0/32768.0;
		          break;
		    case GFS_500DPS:
		          gRes = 500.0/32768.0;
		          break;
		    case GFS_1000DPS:
		          gRes = 1000.0/32768.0;
		          break;
		    case GFS_2000DPS:
		          gRes = 2000.0/32768.0;
		          break;
		  }
		}

		void getAres() {
		  switch (Ascale)
		  {
		  // Possible accelerometer scales (and their register bit settings) are:
		  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
		        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		    case AFS_2G:
		          aRes = 2.0/32768.0;
		          break;
		    case AFS_4G:
		          aRes = 4.0/32768.0;
		          break;
		    case AFS_8G:
		          aRes = 8.0/32768.0;
		          break;
		    case AFS_16G:
		          aRes = 16.0/32768.0;
		          break;
		  }
		}


		void readAccelData(int[] destination)
		{
		  byte[] rawData = new byte[6];  // x/y/z accel register data stored here
		  readBytes(i2cMPU9250, ACCEL_XOUT_H, 6, rawData);  // Read the six raw data registers into data array
		  destination[0] = ((int)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
		  destination[1] = ((int)rawData[2] << 8) | rawData[3] ;  
		  destination[2] = ((int)rawData[4] << 8) | rawData[5] ; 
		}

		void readMagData(int[] destination)
		{
		  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of
		  // data acquisition
		  byte[] rawData = new byte[7];
		  // Wait for magnetometer data ready bit to be set
		  if(0x01 == (readByte( i2cAK8963, AK8963_ST1) & 0x01))
		  {
		    // Read the six raw data and ST2 registers sequentially into data array
		    readBytes( i2cAK8963, AK8963_XOUT_L, 7, rawData);
		    byte c = rawData[6]; // End data read by reading ST2 register
		    // Check if magnetic sensor overflow set, if not then report data
		    if(0x08 !=(c & 0x08))
		    {
		      // Turn the MSB and LSB into a signed 16-bit value
		      destination[0] = ((int)rawData[1] << 8) | rawData[0];
		      // Data stored as little Endian 
		      destination[1] = ((int)rawData[3] << 8) | rawData[2];
		      destination[2] = ((int)rawData[5] << 8) | rawData[4];
		    }
		  }
		}

		int readTempData()
		{
		  byte[] rawData = new byte[2];  // x/y/z gyro register data stored here
		  readBytes(i2cMPU9250, TEMP_OUT_H, 2, rawData);  // Read the two raw data registers sequentially into data array 
		  return ((int)rawData[0] << 8) | rawData[1];  // Turn the MSB and LSB into a 16-bit value
		}

		// Function which accumulates gyro and accelerometer data after device
		// initialization. It calculates the average of the at-rest readings and then
		// loads the resulting offsets into accelerometer and gyro bias registers.
		void calibrateMPU9250(double[] gyroBias, double[] accelBias)
		{  
			  byte[] data = new byte[12]; // data array to hold accelerometer and gyro x, y, z, data
			  byte[] data2 = new byte[2]; // data array to hold accelerometer and gyro x, y, z, data
		  int ii, packet_count, fifo_count;
		  long[] gyro_bias = {0, 0, 0}, accel_bias = {0, 0, 0};
		  
		  // reset device
		  // Write a one to bit 7 reset bit; toggle reset device
		  writeByte(i2cMPU9250, PWR_MGMT_1, 0x80);
		  Threadsleep(100);
		   
		 // get stable time source; Auto select clock source to be PLL gyroscope
		 // reference if ready else use the internal oscillator, bits 2:0 = 001
		  writeByte(i2cMPU9250, PWR_MGMT_1, 0x01);  
		  writeByte(i2cMPU9250, PWR_MGMT_2, 0x00);
		  Threadsleep(200);                                    

		  // Configure device for bias calculation
		  writeByte(i2cMPU9250, INT_ENABLE, 0x00);   // Disable all interrupts
		  writeByte(i2cMPU9250, FIFO_EN, 0x00);      // Disable FIFO
		  writeByte(i2cMPU9250, PWR_MGMT_1, 0x00);   // Turn on internal clock source
		  writeByte(i2cMPU9250, I2C_MST_CTRL, 0x00); // Disable I2C master
		  writeByte(i2cMPU9250, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
		  writeByte(i2cMPU9250, USER_CTRL, 0x0C);    // Reset FIFO and DMP
		  Threadsleep(15);
		  
		// Configure MPU6050 gyro and accelerometer for bias calculation
		  writeByte(i2cMPU9250, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
		  writeByte(i2cMPU9250, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
		  writeByte(i2cMPU9250, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
		  writeByte(i2cMPU9250, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
		 
		  int  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
		  int  accelsensitivity = 16384;  // = 16384 LSB/g

		    // Configure FIFO to capture accelerometer and gyro data for bias calculation
		  writeByte(i2cMPU9250, USER_CTRL, 0x40);   // Enable FIFO  
		  writeByte(i2cMPU9250, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
		  Threadsleep(40);

		// At end of sample accumulation, turn off FIFO sensor read
		  writeByte(i2cMPU9250, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
		  readBytes(i2cMPU9250, FIFO_COUNTH, 2, data2); // read FIFO sample count
		  fifo_count = ((int)data2[0] << 8) | data2[1];
		  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
		  
		  for (ii = 0; ii < packet_count; ii++)
		  {
		    int[] accel_temp = {0, 0, 0}, gyro_temp = {0, 0, 0};
		    readBytes(i2cMPU9250, FIFO_R_W, 12, data); // read data for averaging
		    accel_temp[0] = (int) (((int)data[0] << 8) | data[1]  );  // Form signed 16-bit integer for each sample in FIFO
		    accel_temp[1] = (int) (((int)data[2] << 8) | data[3]  );
		    accel_temp[2] = (int) (((int)data[4] << 8) | data[5]  );
		    gyro_temp[0]  = (int) (((int)data[6] << 8) | data[7]  );
		    gyro_temp[1]  = (int) (((int)data[8] << 8) | data[9]  );
		    gyro_temp[2]  = (int) (((int)data[10] << 8) | data[11]);
		    
		    accel_bias[0] += (int) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		    accel_bias[1] += (int) accel_temp[1];
		    accel_bias[2] += (int) accel_temp[2];
		    gyro_bias[0]  += (int) gyro_temp[0];
		    gyro_bias[1]  += (int) gyro_temp[1];
		    gyro_bias[2]  += (int) gyro_temp[2];
		  }
		  accel_bias[0] /= (int) packet_count; // Normalize sums to get average count biases
		  accel_bias[1] /= (int) packet_count;
		  accel_bias[2] /= (int) packet_count;
		  gyro_bias[0]  /= (int) packet_count;
		  gyro_bias[1]  /= (int) packet_count;
		  gyro_bias[2]  /= (int) packet_count;
		    
		  if(accel_bias[2] > 0L) {accel_bias[2] -= (int) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
		  else {accel_bias[2] += (int) accelsensitivity;}
		   
		// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
		  data[0] = (byte)((-gyro_bias[0]/4  >> 8) & 0xFF); // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
		  data[1] = (byte)((-gyro_bias[0]/4)       & 0xFF); // Biases are additive, so change sign on calculated average gyro biases
		  data[2] = (byte)((-gyro_bias[1]/4  >> 8) & 0xFF);
		  data[3] = (byte)((-gyro_bias[1]/4)       & 0xFF);
		  data[4] = (byte)((-gyro_bias[2]/4  >> 8) & 0xFF);
		  data[5] = (byte)((-gyro_bias[2]/4)       & 0xFF);
		  
		// Push gyro biases to hardware registers
		  writeByte(i2cMPU9250, XG_OFFSET_H, data[0]);
		  writeByte(i2cMPU9250, XG_OFFSET_L, data[1]);
		  writeByte(i2cMPU9250, YG_OFFSET_H, data[2]);
		  writeByte(i2cMPU9250, YG_OFFSET_L, data[3]);
		  writeByte(i2cMPU9250, ZG_OFFSET_H, data[4]);
		  writeByte(i2cMPU9250, ZG_OFFSET_L, data[5]);
		  
		// Output scaled gyro biases for display in the main program
		  gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
		  gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
		  gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

		// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
		// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
		// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
		// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
		// the accelerometer biases calculated above must be divided by 8.

		  int[] accel_bias_reg = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
		  readBytes(i2cMPU9250, XA_OFFSET_H, 2, data2); // Read factory accelerometer trim values
		  accel_bias_reg[0] = (int) (((int)data2[0] << 8) | data2[1]);
		  readBytes(i2cMPU9250, YA_OFFSET_H, 2, data2);
		  accel_bias_reg[1] = (int) (((int)data2[0] << 8) | data2[1]);
		  readBytes(i2cMPU9250, ZA_OFFSET_H, 2, data2);
		  accel_bias_reg[2] = (int) (((int)data2[0] << 8) | data2[1]);
		  
		  long mask = 1; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
		  byte[] mask_bit  = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
		  
		  for(ii = 0; ii < 3; ii++) {
		    if(mask == (accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
		  }
		  
		  // Construct total accelerometer bias, including calculated average accelerometer bias from above
		  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
		  accel_bias_reg[1] -= (accel_bias[1]/8);
		  accel_bias_reg[2] -= (accel_bias[2]/8);
		  
		  data[0] = (byte)((accel_bias_reg[0] >> 8) & 0xFF);
		  data[1] = (byte)((accel_bias_reg[0])      & 0xFF);
		  data[1] = (byte)(data[1] | mask_bit[0]); // preserve temperature compensation bit when writing back to accelerometer bias registers
		  data[2] = (byte)((accel_bias_reg[1] >> 8) & 0xFF);
		  data[3] = (byte)((accel_bias_reg[1])      & 0xFF);
		  data[3] = (byte)(data[3] | mask_bit[1]); // preserve temperature compensation bit when writing back to accelerometer bias registers
		  data[4] = (byte)((accel_bias_reg[2] >> 8) & 0xFF);
		  data[5] = (byte)((accel_bias_reg[2])      & 0xFF);
		  data[5] = (byte)(data[5] | mask_bit[2]); // preserve temperature compensation bit when writing back to accelerometer bias registers
		 
		// Apparently this is not working for the acceleration biases in the MPU-9250
		// Are we handling the temperature correction bit properly?
		// Push accelerometer biases to hardware registers
		  writeByte(i2cMPU9250, XA_OFFSET_H, data[0]);
		  writeByte(i2cMPU9250, XA_OFFSET_L, data[1]);
		  writeByte(i2cMPU9250, YA_OFFSET_H, data[2]);
		  writeByte(i2cMPU9250, YA_OFFSET_L, data[3]);
		  writeByte(i2cMPU9250, ZA_OFFSET_H, data[4]);
		  writeByte(i2cMPU9250, ZA_OFFSET_L, data[5]);

		// Output scaled accelerometer biases for display in the main program
		   accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity; 
		   accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
		   accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;
		}

		// Accelerometer and gyroscope self test; check calibration wrt factory settings
		void MPU9250SelfTest(double[] destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
		{
		  byte[] rawData = new byte[6];
		  byte[] selfTest = new byte[6];
		  int[] gAvg = new int[3];
		  int[] aAvg = new int[3];
		  int[] aSTAvg = new int[3];
		  int[] gSTAvg = new int[3];
		  double[] factoryTrim = new double[6];
		  byte FS = 0;
		   
		  writeByte(i2cMPU9250, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
		  writeByte(i2cMPU9250, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
		  writeByte(i2cMPU9250, GYRO_CONFIG, 1<<FS);  // Set full scale range for the gyro to 250 dps
		  writeByte(i2cMPU9250, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
		  writeByte(i2cMPU9250, ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

		  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
		  
		    readBytes(i2cMPU9250, ACCEL_XOUT_H, 6, rawData);        // Read the six raw data registers into data array
		    aAvg[0] += (int)(((int)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		    aAvg[1] += (int)(((int)rawData[2] << 8) | rawData[3]) ;  
		    aAvg[2] += (int)(((int)rawData[4] << 8) | rawData[5]) ; 
		  
		    readBytes(i2cMPU9250, GYRO_XOUT_H, 6, rawData);       // Read the six raw data registers sequentially into data array
		    gAvg[0] += (int)(((int)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		    gAvg[1] += (int)(((int)rawData[2] << 8) | rawData[3]) ;  
		    gAvg[2] += (int)(((int)rawData[4] << 8) | rawData[5]) ; 
		  }
		  
		  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
		    aAvg[ii] /= 200;
		    gAvg[ii] /= 200;
		  }
		  
		// Configure the accelerometer for self-test
		  writeByte(i2cMPU9250, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
		  writeByte(i2cMPU9250, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
		  Threadsleep(25);

		  for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
		  
		    readBytes(i2cMPU9250, ACCEL_XOUT_H, 6, rawData);  // Read the six raw data registers into data array
		    aSTAvg[0] += (int)(((int)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		    aSTAvg[1] += (int)(((int)rawData[2] << 8) | rawData[3]) ;  
		    aSTAvg[2] += (int)(((int)rawData[4] << 8) | rawData[5]) ; 
		  
		    readBytes(i2cMPU9250, GYRO_XOUT_H, 6, rawData);  // Read the six raw data registers sequentially into data array
		    gSTAvg[0] += (int)(((int)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		    gSTAvg[1] += (int)(((int)rawData[2] << 8) | rawData[3]) ;  
		    gSTAvg[2] += (int)(((int)rawData[4] << 8) | rawData[5]) ; 
		  }
		  
		  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
		    aSTAvg[ii] /= 200;
		    gSTAvg[ii] /= 200;
		  }   
		  
		  // Configure the gyro and accelerometer for normal operation
		  writeByte(i2cMPU9250, ACCEL_CONFIG, 0x00);  
		  writeByte(i2cMPU9250, GYRO_CONFIG,  0x00);  
		  Threadsleep(25);
		   
		  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
		  selfTest[0] = readByte(i2cMPU9250, SELF_TEST_X_ACCEL); // X-axis accel self-test results
		  selfTest[1] = readByte(i2cMPU9250, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
		  selfTest[2] = readByte(i2cMPU9250, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
		  selfTest[3] = readByte(i2cMPU9250, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
		  selfTest[4] = readByte(i2cMPU9250, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
		  selfTest[5] = readByte(i2cMPU9250, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results
		  
		  // Retrieve factory self-test value from self-test code reads
		  factoryTrim[0] = (2620/1<<FS)*(Math.pow( 1.01 , ((double)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
		  factoryTrim[1] = (2620/1<<FS)*(Math.pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
		  factoryTrim[2] = (2620/1<<FS)*(Math.pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
		  factoryTrim[3] = (2620/1<<FS)*(Math.pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
		  factoryTrim[4] = (2620/1<<FS)*(Math.pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
		  factoryTrim[5] = (2620/1<<FS)*(Math.pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation
		 
		 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
		 // To get percent, must multiply by 100
		  for (int i = 0; i < 3; i++) {
		    destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i];   // Report percent differences
		    destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // Report percent differences
		  }
		}
	
	// Wire.h read and write protocols
	void writeByte(I2C device, int subAddress, int data)
	{
		device.write(subAddress, data);
	}

	byte readByte(I2C device, int subAddress)
	{
	  int count = 1;
	  byte[] data = new byte[1]; 
	  
	  device.read(subAddress, count, data);
	  
	  return data[0];
	}

	void readBytes(I2C device, int subAddress, int count, byte[] dest)
	{  
		device.read(subAddress, count, dest);
	}	
	
	void Threadsleep(int millis)
	{
		try
		{
			Thread.sleep(millis);
		}
		catch(Exception e)
		{
		}
	}
}

