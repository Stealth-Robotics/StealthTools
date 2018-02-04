package org.usfirst.frc4444.TerrifiedCameliansLittleSistem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import stealth_libraries.Encoder;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static WPI_TalonSRX leftMotor1SpeedControler;
    public static WPI_TalonSRX rightMotor1SpeedControler;
    public static WPI_TalonSRX auxMotor1SpeedControler;
    public static DifferentialDrive driveBaseDifferentialDrive1;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private static final double kVoltsPerDegreePerSecond = 0.0128;
    /*
     * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For now
     * we just want the primary one.
     */
    public static final int kPIDLoopIdx = 0;

    /*
     * set to zero to skip waiting for confirmation, set to nonzero to wait and
     * report to DS if action fails.
     */
    public static final int kTimeoutMs = 10;
    public static PigeonIMU pigeonIMU;

    public static NetworkTable netTable;
    public static NetworkTable piTable;

    public static Encoder leftEncoder;
    public static Encoder rightEncoder;
    
    public static void init() {
        leftMotor1SpeedControler = new WPI_TalonSRX(1);
        rightMotor1SpeedControler = new WPI_TalonSRX(2);
        auxMotor1SpeedControler = new WPI_TalonSRX(3);
        
        leftEncoder = new Encoder(leftMotor1SpeedControler);
        rightEncoder = new Encoder(rightMotor1SpeedControler);
        leftEncoder.SetTicksPerRev(Constants.kEncoderPulsePerRev);
        leftEncoder.SetRevToInches(Constants.kWheelCirc);
        rightEncoder.SetTicksPerRev(Constants.kEncoderPulsePerRev);
        rightEncoder.SetRevToInches(Constants.kWheelCirc);
        
        pigeonIMU = new PigeonIMU(auxMotor1SpeedControler);
        pigeonIMU.setFusedHeading(0.0, 10);

        netTable = NetworkTable.getTable("FRCRobot");
        piTable = NetworkTable.getTable("piBlob0");
        
        
    }
    
    public static void SetUpTalonsForTele()
    {
      System.out.println("SetUpTalonsForTele");
      SetUpTalonForTele(leftMotor1SpeedControler);
      SetUpTalonForTele(rightMotor1SpeedControler);
      leftMotor1SpeedControler.setInverted(true);
      
    }
    
    private static void SetUpTalonForTele(WPI_TalonSRX talon) {
      talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, kTimeoutMs);
      talon.setSensorPhase(true);

      /* set the peak and nominal outputs, 12V means full */
      talon.configNominalOutputForward(0, kTimeoutMs);
      talon.configNominalOutputReverse(0, kTimeoutMs);
      talon.configPeakOutputForward(1, kTimeoutMs);
      talon.configPeakOutputReverse(-1, kTimeoutMs);

      /* set closed loop gains in slot0 */
      talon.config_kF(kPIDLoopIdx, 0.34, kTimeoutMs);
      talon.config_kP(kPIDLoopIdx, 0.2, kTimeoutMs);
      talon.config_kI(kPIDLoopIdx, 0, kTimeoutMs);
      talon.config_kD(kPIDLoopIdx, 0, kTimeoutMs);

      talon.setSafetyEnabled(true);
      talon.setExpiration(kTimeoutMs);
    }

    public static void SetUpTalonsForAuto()
    {
      System.out.println("SetUpTalonsForAuto");
      SetUpTalonForAuto(leftMotor1SpeedControler);
      SetUpTalonForAuto(rightMotor1SpeedControler);
      leftMotor1SpeedControler.setInverted(true);
      
    }
    
    private static void SetUpTalonForAuto(WPI_TalonSRX talon) {
      talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
      talon.setSensorPhase(true); /* keep sensor and motor in phase */
      talon.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

      talon.config_kF(0, 0.076, Constants.kTimeoutMs);
      talon.config_kP(0, 2.000, Constants.kTimeoutMs);
      talon.config_kI(0, 0.0, Constants.kTimeoutMs);
      talon.config_kD(0,20.0, Constants.kTimeoutMs);

      talon.configMotionProfileTrajectoryPeriod(20, Constants.kTimeoutMs); //Our profile uses 10 ms timing
      /* status 10 provides the trajectory target for motion profile AND motion magic */
      talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 20, Constants.kTimeoutMs);
      
    }

    static public double returnRawGyro() {
      double[] rawGyro = new double[3];
      RobotMap.pigeonIMU.getRawGyro(rawGyro);
      return rawGyro[Constants.kGyroZ];
    }
}
