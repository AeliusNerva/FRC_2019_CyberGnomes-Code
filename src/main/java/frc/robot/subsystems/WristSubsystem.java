/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.*; 
import com.ctre.phoenix.motorcontrol.can.*; 
import frc.robot.OI;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**ZS
 * Add your docs here.
 */
public class WristSubsystem extends Subsystem {
  int kTimeoutMs = 10;
  int kPIDLoopIdx = 0;
  WPI_TalonSRX 
  SRXwrist = new WPI_TalonSRX(OI.SRXwrist);
  CANSparkMax MAXlift = new CANSparkMax(OI.MAXleftlift, MotorType.kBrushless);
  public CANEncoder Encoder = new CANEncoder(MAXlift);
  public WristSubsystem(){
 
  }
  public void reset() {
    SRXwrist.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
  }
  public void initDefaultCommand() {
  }
  public void movetoHatchPosition() { // move wrist to hatch position
    if (Encoder.getPosition() < 90) {
      SRXwrist.configMotionAcceleration(200, kTimeoutMs);// orignal 200
    SRXwrist.set(ControlMode.MotionMagic,475);
    }
    else if (Encoder.getPosition() < 200) {
      SRXwrist.configMotionAcceleration(200, kTimeoutMs);// orignal 200
      SRXwrist.set(ControlMode.MotionMagic,475);
    }
    else {
      SRXwrist.configMotionAcceleration(400, kTimeoutMs);// orignal 200
      SRXwrist.set(ControlMode.MotionMagic,750);
    }
  }
  public void movetoBallPosition() { // move wrist to ball position
    SRXwrist.set(ControlMode.MotionMagic,650);//orgi -650
  }
  public void movetoScoreBallPosition() { // move wrist to ball position
        
    SmartDashboard.putNumber("test1234", 12345);
    if (Encoder.getPosition() < 90) {
      SRXwrist.configMotionAcceleration(200, kTimeoutMs);// orignal 200
    SRXwrist.set(ControlMode.MotionMagic,400);
    }
    else if (Encoder.getPosition() < 200) {
      SRXwrist.configMotionAcceleration(200, kTimeoutMs);// orignal 200
      SRXwrist.set(ControlMode.MotionMagic,475);
    }
    else {
      SRXwrist.configMotionAcceleration(200, kTimeoutMs);// orignal 200
      SRXwrist.set(ControlMode.MotionMagic,720);
    }
  }
  public void movetoHomePosition() { // move wrist to home position
    SRXwrist.set(ControlMode.MotionMagic,0);
  }

  public void in() { // moves the wrist at set speed
    SRXwrist.set(ControlMode.PercentOutput,1); 
    SmartDashboard.putNumber("www", 13290593);
  }
  public void out() { // moves the wrist at set speed
    SRXwrist.set(ControlMode.PercentOutput,-1); 
  }
  public void stop() {
    SRXwrist.set(0);
  }
}
