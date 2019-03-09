/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.OI;

/**
 * Add your docs here.
 */
public class IntakeSubsystem extends Subsystem {
  VictorSPX SPXintake = new VictorSPX(OI.SPXintake); //defining victor
  // Put methods  for controlling this subsystem
  // here. Call these from Commands.

  public void succ() {
    SPXintake.set(ControlMode.PercentOutput, .6); //will spin the the intake to the set ammount
  }
  public void push() {
 SPXintake.set(ControlMode.PercentOutput, -.6);
  }
  public void stall() {
    SPXintake.set(ControlMode.PercentOutput, 0.3); 
  }
  public void stop() {
    SPXintake.set(ControlMode.PercentOutput, 0);
  }
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
