package org.usfirst.frc4089.VisionBot;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * Use a rate gyro to return the robots heading relative to a starting position.
 * The Gyro class tracks the robots heading based on the starting position. As
 * the robot rotates the new heading is computed by integrating the rate of
 * rotation returned by the sensor. When the class is instantiated, it does a
 * short calibration routine where it samples the gyro while at rest to
 * determine the default offset. This is subtracted from each sample to
 * determine the heading.
 *
 * This class is for the digital ADXRS453 gyro sensor that connects via SPI. A
 * datasheet can be found here:
 * http://www.analog.com/media/en/technical-documentation/data-sheets/ADXRS453.
 * pdf
 */
public class GyroHelper extends GyroBase implements Gyro, PIDSource {
    public static final double kCalibrationSampleTime = 5.0;

    private static final double kSamplePeriod = 0.001;
    private static final double kDegreePerSecondPerLSB = -0.0125;
    MPU9250IMU mGyro;
    
    private boolean m_is_calibrating;
    private double m_last_center;

    /**
     * Constructor. Uses the onboard CS0.
     */
    public GyroHelper() {
        mGyro = RobotMap.mImu;
    }

    /**
     * This is a blocking calibration call. There are also non-blocking options
     * available in this class!
     * 
     * {@inheritDoc}
     */
    @Override
    public synchronized void calibrate() {
        Timer.delay(0.1);
        startCalibrate();
        Timer.delay(kCalibrationSampleTime);
        endCalibrate();
    }

    public synchronized void startCalibrate() {
        if (mGyro == null)
            return;

        if (!m_is_calibrating) {
            m_is_calibrating = true;
            mGyro.calcGyroData();
            m_last_center = mGyro.gz;
        }
    }

    public synchronized void endCalibrate() {
        if (m_is_calibrating) {
            m_is_calibrating = false;
            mGyro.calcGyroData();
            m_last_center = mGyro.gz;
        }
    }

    public synchronized void cancelCalibrate() {
        if (m_is_calibrating) {
            m_is_calibrating = false;
            mGyro.calcGyroData();
            m_last_center = mGyro.gz;
        }
    }

    public synchronized double getCenter() {
        return m_last_center;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public synchronized void reset() {
    	endCalibrate();
    }

    /**
     * Delete (free) the spi port used for the gyro and stop accumulating.
     */
    @Override
    public void free() {
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public synchronized double getAngle() {
        if (mGyro == null)
            return 0.0;
        if (m_is_calibrating) {
            return 0.0;
        }
        mGyro.calcGyroData();
        
        return (mGyro.gz-m_last_center) * kDegreePerSecondPerLSB * kSamplePeriod;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public synchronized double getRate() {
        if (mGyro == null)
            return 0.0;
        if (m_is_calibrating) {
            return 0.0;
        }
        mGyro.calcGyroData();
        return (mGyro.gz-m_last_center) * kDegreePerSecondPerLSB;
    }
}

