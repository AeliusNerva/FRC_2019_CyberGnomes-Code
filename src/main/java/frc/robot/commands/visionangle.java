/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class visionangle extends Command {
  boolean done;
  double offset;
  public visionangle(double testvar2) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    SmartDashboard.putNumber("testvar2", testvar2);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drive.readyToRotate();
    //Robot.drive.zero();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.drive.rotateToAngle();
    try {
			NetworkTableInstance inst = NetworkTableInstance.getDefault();
			NetworkTable table = inst.getTable("CVResultsTable");
			String[] VisionValues = table.getEntry("VisionResults").getString("").split(",");
			offset = Double.parseDouble(VisionValues[4]);
		} catch (Exception e) {
    }
    if (offset > -2 && offset < 2) {
			done = true;
		}
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    SmartDashboard.putString("VisionAngle is Finished", "True");
    return done;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SmartDashboard.putString("Hit End Condition", "True");
    Robot.drive.PIDstop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.drive.PIDstop();
  }
}
