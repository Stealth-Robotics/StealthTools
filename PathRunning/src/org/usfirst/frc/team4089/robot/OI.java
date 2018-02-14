package org.usfirst.frc.team4089.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI {
  public Joystick joystick1;

  public OI() {
      joystick1 = new Joystick(0);
  }

  public Joystick getJoystick1() {
      return joystick1;
  }
}
