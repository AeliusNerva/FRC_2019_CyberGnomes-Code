/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Add your docs here.
 */
public class PnumaticSubsystem extends Subsystem {
  // defining solenoids
  Solenoid grab = new Solenoid(1);
  Solenoid shifter = new Solenoid(0);
  // defining solenoids

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void grab() { // grabber to our position
    grab.set(false);
  }
  public void retract() { // grabber to in position
    grab.set(true);
  }
  public void shiftlow() { // shifts to Low gear 
    shifter.set(false);
  }
  public void shifthigh() { // shifts to High gear 
    shifter.set(true);
  }
}
