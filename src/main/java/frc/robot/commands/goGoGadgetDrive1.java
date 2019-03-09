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
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
public class goGoGadgetDrive1 extends Command {
  boolean end = false;
  WPI_TalonSRX SRXgadgetDrive = new WPI_TalonSRX(OI.SRXgadgetDrive);
  public goGoGadgetDrive1() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
   setTimeout(2);
    SRXgadgetDrive.setSelectedSensorPosition(0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putNumber("drivewheelenc", SRXgadgetDrive.getSelectedSensorPosition());
     // if (SRXgadgetDrive.getSelectedSensorPosition() == -100000) {
     //   end = true;
     // }
     // else {
       Robot.gadgetdrive.manuadrive();
      //}

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return  isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.gadgetdrive.stopdrive();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
