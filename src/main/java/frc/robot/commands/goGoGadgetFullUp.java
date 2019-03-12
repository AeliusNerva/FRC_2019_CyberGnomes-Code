/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
public class goGoGadgetFullUp extends Command {
  boolean end = false;
  CANSparkMax MAXgadgetlift = new CANSparkMax(OI.MAXleftGadget, MotorType.kBrushless);
  CANSparkMax MAXgadgetlift2 = new CANSparkMax(OI.MAXrightgadget, MotorType.kBrushless);
  public CANEncoder Encoder = new CANEncoder(MAXgadgetlift);
  public CANEncoder Encoder2 = new CANEncoder(MAXgadgetlift2);
  public goGoGadgetFullUp() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.gadget.doubleDown();
    if (Encoder.getPosition() > 99 && Encoder2.getPosition() > 99){

      end = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return end;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
