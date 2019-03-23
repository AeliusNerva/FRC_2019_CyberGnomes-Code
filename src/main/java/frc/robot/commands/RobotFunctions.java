/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shar
ed by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
import com.ctre.phoenix.motorcontrol.*; 
import com.ctre.phoenix.motorcontrol.can.*; 
import edu.wpi.first.wpilibj.Joystick;
public class RobotFunctions extends Command {
  CANSparkMax MAXlift = new CANSparkMax(OI.MAXleftlift, MotorType.kBrushless);
  //CANSparkMax MAXlift2 = new CANSparkMax(OI.MAXrightlift, MotorType.kBrushless);
  public CANPIDController lift = new CANPIDController(MAXlift);
  //public CANPIDController lift2 = new CANPIDController(MAXlift2);
  public CANEncoder Encoder = new CANEncoder(MAXlift);
 // public CANEncoder Encoder2 = new CANEncoder(MAXlift2);
  CANSparkMax MAXgadgetlift = new CANSparkMax(OI.MAXleftGadget, MotorType.kBrushless);
  CANSparkMax MAXgadgetlift2 = new CANSparkMax(OI.MAXrightgadget, MotorType.kBrushless);
  public CANPIDController gadgetlift = new CANPIDController(MAXgadgetlift);
  public CANPIDController gadgetlift2 = new CANPIDController(MAXgadgetlift2);
  public CANEncoder EncoderGad = new CANEncoder(MAXgadgetlift);
  public CANEncoder Encoder2Gad = new CANEncoder(MAXgadgetlift2);
  WPI_TalonSRX SRXwrist = new WPI_TalonSRX(OI.SRXwrist);
  int i;
  long startTime = System.currentTimeMillis();
  boolean GrabBall, GrabDisk, ReleaceBall, ReleaceDisk, ManualWrist, ManualLift, ManualGadget, test, stop, GrabDisk2, GrabBall2  = false;
  Joystick _joy = new Joystick(0);
  Joystick _backupJoy = new Joystick(1);
  String Object= "Disc";
  public RobotFunctions() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.lift.movetoHome();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (_joy.getRawButton(2) && _joy.getRawButton(9)) {
      Robot.lift.down();
    }
    if (_joy.getRawButton(12) || _backupJoy.getRawButton(12)) {
      Robot.wrist.stop();
      Robot.lift.stop();
      Robot.intake.stop();
    }
    SmartDashboard.putNumber("encoder1", EncoderGad.getPosition());
    SmartDashboard.putNumber("encoder2", Encoder2Gad.getPosition());
    if (_joy.getRawButton(4)) { // High 4 bar pos
      if (Object == "Ball") {
        Robot.lift.GoToHighBallPosition();
      }
      else if (Object == "Disc") {
        Robot.lift.GoToHighHatchPosition();
      }
    }
    if (_joy.getRawButton(3)) { // Mid 4 bar pos
      if (Object == "Ball") {
        Robot.lift.GoToMidBallPosition();
      }
      else if (Object == "Disc") {
        Robot.lift.GoToMidHatchPosition();
      }
    }
    if (_joy.getRawButton(2)) { // Low 4 bar pos
      if (Object == "Ball") {
      Robot.lift.GoToLowBallPosition();
      }
      else if (Object == "Disc") {
        Robot.lift.GoToLowHatchPosition();
      }
    }
    if (_joy.getRawButton(1)) { // CargoShip 4 bar pos
      Robot.lift.GoToCargoShipPosition();
    }
    SmartDashboard.putNumber("wristpos", SRXwrist.getSelectedSensorPosition(0));
    if (_backupJoy.getRawButton(9)) {
     Robot.gadget.doubleUp();
    }
    if (_backupJoy.getRawButton(13) || _backupJoy.getRawButton(13)) {
      Robot.gadget.zero();
      Robot.lift.init();
      Robot.wrist.reset();
      Robot.gadget.reset();
      //Robot.lift.init();
    }
      //Robot.drive.Swerve();

    if (_joy.getRawButton(7)) { // on press ready and spinning on releace ball is stowed
      Robot.lift.movetoHome();
      Robot.pnumatics.grab();
      Robot.wrist.movetoBallPosition();
      Robot.intake.succ();
      GrabBall = true;
      Object = "Ball";
    }
    else if (GrabBall) {
      Robot.wrist.movetoHomePosition();
      Robot.intake.stall();
      stop = true;
      GrabBall = false;
    }
    if (_joy.getRawButton(5)) { // on press ready to grab hatch on releace grab hatch
      Robot.lift.movetoGrabHatch();
      Robot.wrist.movetoHatchPosition();
      Robot.intake.stop();
      Robot.pnumatics.retract();
      GrabDisk = true;
      Object = "Disc";
    }
    else if (GrabDisk) {
      Robot.pnumatics.grab();
      try {
        Thread.sleep(250);
     } catch (Exception e) {
        System.out.println(e);
     }
     Robot.wrist.movetoHomePosition();
      GrabDisk2 = false;
    }
    if (_backupJoy.getRawButton(7)) { // on press ready and spinning on releace ball is stowed
      Robot.pnumatics.grab();
      Robot.wrist.movetoBallPosition();
      Robot.intake.succ();
      GrabBall2 = true;
      Object = "Ball";
    }
    else if (GrabBall2) {
      Robot.wrist.movetoHomePosition();
      Robot.intake.stall();
      stop = true;
      GrabBall2 = false;
    }
    if (_backupJoy.getRawButton(5)) { // on press ready to grab hatch on releace grab hatch
      Robot.wrist.movetoHatchPosition();
      Robot.intake.stop();
      Robot.pnumatics.retract();
      GrabDisk2 = true;
      Object = "Disc";
    }
    else if (GrabDisk2) {
      Robot.pnumatics.grab();
      try {
        Thread.sleep(250);
     } catch (Exception e) {
        System.out.println(e);
     }
     Robot.wrist.movetoHomePosition();
      GrabDisk = false;
    }
    if (_joy.getRawButton(11) || _backupJoy.getRawButton(11)) { // on press shift to low on releace shift to high
      Robot.pnumatics.shiftlow();
    }
    else {
      Robot.pnumatics.shifthigh();
    }
    if (_joy.getRawButton(6) || _backupJoy.getRawButton(6)) { // on press claws closed on relace claws open
    Robot.intake.stop();
     Robot.wrist.movetoHatchPosition();
      ReleaceDisk = true;
    }
    else if (ReleaceDisk) {
      Robot.pnumatics.retract();
      ReleaceDisk = false;
    }
    if (_joy.getRawButton(8) || _backupJoy.getRawButton(8)) { // on press shoot ball
      Robot.wrist.movetoScoreBallPosition();
      ReleaceBall = true;
    }
    else if (ReleaceBall) {
      Robot.intake.push();
      ReleaceBall = false;
    }
    if (_backupJoy.getRawButton(1) || _backupJoy.getRawButton(3)) { // square button wrist up circle wrist up
      while (_backupJoy.getRawButton(3)) {
        SmartDashboard.putNumber("aaaaaaaaaa", 13);
      Robot.wrist.in();
      }
      while (_backupJoy.getRawButton(1)) {
      Robot.wrist.out();
      }
      ManualWrist = true;
    }
    else if (ManualWrist) {
      Robot.wrist.stop();
      ManualWrist = false;
    }
    if (_backupJoy.getRawButton(2) || _backupJoy.getRawButton(4)) { // triangle button lift up x button lift down
      if (_backupJoy.getRawButton(9) == false) {
        if (_backupJoy.getRawButton(2) && Encoder.getPosition() < -5) {
          Robot.lift.up();
        }
        if (_backupJoy.getRawButton(4)) {
          Robot.lift.down();
        }
        if (Encoder.getPosition() > -5 && _backupJoy.getRawButtonReleased(4)) {
          Robot.lift.stop();
        }
      }
      else {
        if (_backupJoy.getRawButton(2)) {
          Robot.lift.up();
        }
        if (_backupJoy.getRawButton(4)) {
          Robot.lift.down();
        }
      }
    ManualLift = true;
    }
    else if (ManualLift) {
      Robot.lift.stop();
      ManualLift = false;
    }
  /* if (_joy.getPOV() == 90 || _joy.getPOV() == 270) {
      while (_joy.getPOV() == 90) {
        Robot.gadget.up();
      }
      while (_joy.getPOV() == 270) {
       Robot.gadget.down();
      }
    ManualGadget = true;
    }
    else if (ManualGadget) {
      Robot.gadget.stop();
    }*/
    }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
