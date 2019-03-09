/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class goGoGadgetbackdown extends Command {
  CANSparkMax MAXgadgetlift = new CANSparkMax(OI.MAXleftGadget, MotorType.kBrushless);
  CANSparkMax MAXgadgetlift2 = new CANSparkMax(OI.MAXrightgadget, MotorType.kBrushless);
  public CANEncoder Encoder = new CANEncoder(MAXgadgetlift);
  public CANEncoder Encoder2 = new CANEncoder(MAXgadgetlift2);
  boolean end= false;
  public goGoGadgetbackdown() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.gadget.stop();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putNumber("enc2", Encoder2.getPosition());
    Robot.gadget.backUp();
    if (Encoder.getPosition() < 3){
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
    SmartDashboard.putNumber("aaaaaa", 122);
    Robot.gadget.stop();
  }


  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
