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
  boolean done = false;
  double offset;
  public visionangle(double testvar2) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    SmartDashboard.putNumber("testvar2", testvar2);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    done = false;
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
      SmartDashboard.putNumber("Vision Angle Offset:", offset);
      SmartDashboard.putBoolean("done", done);  
      if (offset > -2.0f && offset < 2.0f) {
        done = true;
      }
		} catch (Exception e) {
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return done;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.PIDstop();  
    SmartDashboard.putNumber("Hit End Condition", 1234);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.drive.PIDstop();
  }
}
