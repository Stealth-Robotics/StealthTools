package org.usfirst.frc.team4089.robot.commands;

import org.usfirst.frc.team4089.robot.RobotMap;
import edu.wpi.first.wpilibj.command.TimedCommand;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import java.io.FileWriter;
import java.io.IOException;

/**
 * Drivetrain characterization command. Makes it easy to perform drivetrain characterization tests and record data for
 * use with R script to calculate kA and kV.
 */
public class DrivetrainCharacterizer extends TimedCommand {

    private FileWriter fw;
    private TestMode mode;
    private double speed, rampRate, maxSpeed;

    public enum TestMode {
        QUASI_STATIC,
        STEP_VOLTAGE;
    }

    public DrivetrainCharacterizer(double timeout, TestMode mode, Double rampRate, Double maxSpeed) {
        super(timeout);
        
      System.out.println("CConstr...");

      this.mode = mode;
        this.rampRate = (rampRate != null) ? rampRate : 0;
        this.maxSpeed = (maxSpeed != null) ? maxSpeed : 0.4;
    }

    @Override
    protected void initialize() {
        System.out.println("Chracterization starting...");
        speed = 0;
        try {
            switch (mode) {
                case QUASI_STATIC:
                    fw = new FileWriter("/home/lvuser/velFile3.csv", false);
                    break;
                case STEP_VOLTAGE:
                    fw = new FileWriter("/home/lvuser/accelFile2.csv", false);
                    speed = maxSpeed;
                    break;
            }
            fw.write("Drive.left_vel, Drive.right_vel, time, Drive.left_voltage, Drive.right_voltage\n");
        }
        catch(IOException e) {
            e.printStackTrace();
            System.out.println("Unable to create FileWriter");
        }
        System.out.println("Writing...");
    }

    /**
     * Called repeatedly when this Command is scheduled to run. If the characterizer is set to QUASI_STATIC mode, then
     * the speed controller input is incremented repeatedly.
     */
    @Override
    protected void execute() {
      RobotMap.leftMotor1SpeedControler.set(speed);
      RobotMap.rightMotor1SpeedControler.set(speed);
      
        try {
            fw.write(
                    RobotMap.leftMotor1SpeedControler.getSelectedSensorVelocity(0) + ", " +
                    RobotMap.rightMotor1SpeedControler.getSelectedSensorVelocity(0) + ", " +
                    timeSinceInitialized()*1000 + ", " +
                    RobotMap.leftMotor1SpeedControler.getMotorOutputVoltage() + ", " +
                    RobotMap.rightMotor1SpeedControler.getMotorOutputVoltage()  + "\n"
            );
            fw.flush();

            //If rampRate causes speed to exceed max speed, stop the command. Speed never exceeds maxSpeed in
            //STEP_VOLTAGE mode because rampRate is 0. Thus, no switch needed.
            if(speed > maxSpeed) {
                System.out.println("Max speed exceeded " + timeSinceInitialized() + " seconds after initialization");
                cancel();
            }
            speed += rampRate;
        }
        catch(IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Called when command isFinished returns true. Closes FileWriter and sets speed controllers to 0
     */
    @Override
    protected void end() {
        //Ramp slowly back down after test to not destroy gearboxes
        for(double s = speed; s > 0; s -= 0.0003) {
          RobotMap.leftMotor1SpeedControler.set(s);
          RobotMap.rightMotor1SpeedControler.set(s);
        }
        try {
            fw.flush();
            fw.close();
        }
        catch(IOException e) {
            e.printStackTrace();
        }
        System.out.println("Done!");
    }
}