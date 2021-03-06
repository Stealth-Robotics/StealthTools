// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc4089.Test;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static WPI_TalonSRX driveBaseRight1;
    public static WPI_TalonSRX driveBaseRight2;
    public static SpeedControllerGroup driveBaserightSpeed;
    public static WPI_TalonSRX driveBaseLeft1;
    public static WPI_TalonSRX driveBaseLeft2;
    public static SpeedControllerGroup driveBaseleftSpeed;
    public static DifferentialDrive driveBaseRobotDrive;
    public static Encoder driveBaseLeftQuadratureEncoder;
    public static Encoder driveBaseRightQuadratureEncoder;
    public static AnalogGyro driveBaseAnalogGyro1;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static NetworkTable netTable;

    public static void init() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveBaseRight1 = new WPI_TalonSRX(1);
        
        
        driveBaseRight2 = new WPI_TalonSRX(2);
        
        driveBaseRight1.setInverted(true);
        
        driveBaseRight2.setInverted(true);
        
        driveBaserightSpeed = new SpeedControllerGroup(driveBaseRight1, driveBaseRight2);

        driveBaseLeft1 = new WPI_TalonSRX(4);
        
        
        driveBaseLeft2 = new WPI_TalonSRX(3);
        
        driveBaseLeft1.setInverted(true);
        
        driveBaseLeft2.setInverted(true);
        
        driveBaseleftSpeed = new SpeedControllerGroup(driveBaseLeft1, driveBaseLeft2);
        
        driveBaseRobotDrive = new DifferentialDrive(driveBaseleftSpeed,driveBaserightSpeed);
        
        driveBaseRobotDrive.setSafetyEnabled(true);
        driveBaseRobotDrive.setExpiration(0.1);
        driveBaseRobotDrive.setMaxOutput(1.0);
        driveBaseLeftQuadratureEncoder = new Encoder(2, 3, false, EncodingType.k4X);
        driveBaseLeftQuadratureEncoder.setName("DriveBase", "Left Quadrature Encoder");
        driveBaseLeftQuadratureEncoder.setDistancePerPulse(1.0);
        driveBaseLeftQuadratureEncoder.setPIDSourceType(PIDSourceType.kRate);
        driveBaseRightQuadratureEncoder = new Encoder(0, 1, false, EncodingType.k4X);
        driveBaseRightQuadratureEncoder.setName("DriveBase", "Right Quadrature Encoder");
        driveBaseRightQuadratureEncoder.setDistancePerPulse(1.0);
        driveBaseRightQuadratureEncoder.setPIDSourceType(PIDSourceType.kRate);
        driveBaseAnalogGyro1 = new AnalogGyro(0);
        driveBaseAnalogGyro1.setName("DriveBase", "AnalogGyro 1");
        driveBaseAnalogGyro1.setSensitivity(0.007);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveBaseAnalogGyro1.calibrate();
        netTable = NetworkTable.getTable("FRCRobot");
    }
}
