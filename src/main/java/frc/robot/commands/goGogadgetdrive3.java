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
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class goGogadgetdrive3 extends Command {
  boolean end = false;
  WPI_TalonSRX SRXsteer = new WPI_TalonSRX(OI.SRXtoprightsteer);
	WPI_TalonSRX SRXsteer2 = new WPI_TalonSRX(OI.SRXtopleftsteer);
	WPI_TalonSRX SRXsteer3 = new WPI_TalonSRX(OI.SRXbottomleftsteer);
  WPI_TalonSRX SRXsteer4 = new WPI_TalonSRX(OI.SRXbottomrightsteer);
  CANSparkMax MAXdrive = new CANSparkMax(OI.MAXtoprightdrive, MotorType.kBrushless);
	CANSparkMax MAXdrive2 = new CANSparkMax(OI.MAXtopleftdrive, MotorType.kBrushless);
	CANSparkMax MAXdrive3 = new CANSparkMax(OI.MAXbottomleftdrive, MotorType.kBrushless);
	CANSparkMax MAXdrive4 = new CANSparkMax(OI.MAXbottomrightdrive, MotorType.kBrushless);
	public CANEncoder Encoder = new CANEncoder(MAXdrive);
	public CANEncoder Encoder2 = new CANEncoder(MAXdrive2);
	public CANEncoder Encoder3 = new CANEncoder(MAXdrive3);
  public CANEncoder Encoder4 = new CANEncoder(MAXdrive4);
  public goGogadgetdrive3() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(4);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   // if (Encoder.getPosition() > 9999){
   //   end = false;
   // }
     Robot.drive.movesideways();

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut() || end;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.stop();
   // Robot.gadgetdrive.stopdrive();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
