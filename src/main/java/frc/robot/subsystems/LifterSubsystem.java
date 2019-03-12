/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANError;
/**
 * Add your docs here.
 */
public class LifterSubsystem extends Subsystem {
  // defining motors and encoters 
  CANSparkMax MAXlift = new CANSparkMax(OI.MAXleftlift, MotorType.kBrushless);
  CANSparkMax MAXlift2 = new CANSparkMax(OI.MAXrightlift, MotorType.kBrushless);
  public CANPIDController lift = new CANPIDController(MAXlift);
  public CANPIDController lift2 = new CANPIDController(MAXlift2);
  public CANEncoder Encoder = new CANEncoder(MAXlift);
  public CANEncoder Encoder2 = new CANEncoder(MAXlift2);

  // defining motors and encoters 

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() { 
    }
  public void init() {
    SmartDashboard.putNumber("output2", MAXlift2.get());
    Encoder.setPosition(0);
    Encoder2.setPosition(0);
  }
  public void down() {
    SmartDashboard.putNumber("emcoder", Encoder.getPosition());
    SmartDashboard.putNumber("emcoder2", Encoder2.getPosition());
    MAXlift.set(1);
    MAXlift2.set(-1);
  }
  public void up() {
    SmartDashboard.putNumber("emcoder", Encoder.getPosition());
    SmartDashboard.putNumber("emcoder2", Encoder2.getPosition());

    SmartDashboard.putNumber("down", 214);
    MAXlift.set(-1);
    MAXlift2.set(1);
  }
  public void stop() {
    MAXlift.set(0);
    MAXlift2.set(0);
  }
  public void GoToHighBallPosition() { // moving to given position
      // setting PID values
    lift.setP(1);
    lift.setI(0);
    lift.setD(0);
    lift.setIZone(0);
    lift.setFF(0);
    lift.setOutputRange(-1, 1);

    lift2.setP(1);
    lift2.setI(0);
    lift2.setD(0);
    lift2.setIZone(0);
    lift2.setFF(0);
    lift2.setOutputRange(-1, 1);
    // setting PID values
    lift.setReference(280, ControlType.kPosition);
    lift2.setReference(-280, ControlType.kPosition);//orginal 320
  }
  public void GoToMidBallPosition() { // moving to given position
    // setting PID values
  lift.setP(1);
  lift.setI(0);
  lift.setD(0);
  lift.setIZone(0);
  lift.setFF(0);
  lift.setOutputRange(-1, 1);

  lift2.setP(1);
  lift2.setI(0);
  lift2.setD(0);
  lift2.setIZone(0);
  lift2.setFF(0);
  lift2.setOutputRange(-1, 1);
  // setting PID values
  lift.setReference(165, ControlType.kPosition);
  lift2.setReference(-165, ControlType.kPosition);//orginal 320
}
public void GoToLowBallPosition() { // moving to given position
  // setting PID values
lift.setP(1);
lift.setI(0);
lift.setD(0);
lift.setIZone(0);
lift.setFF(0);
lift.setOutputRange(-1, 1);

lift2.setP(1);
lift2.setI(0);
lift2.setD(0);
lift2.setIZone(0);
lift2.setFF(0);
lift2.setOutputRange(-1, 1);
// setting PID values
lift.setReference(37, ControlType.kPosition);
lift2.setReference(-37, ControlType.kPosition);//orginal 320
}
public void GoToHighHatchPosition() { // moving to given position
  // setting PID values
lift.setP(1);
lift.setI(0);
lift.setD(0);
lift.setIZone(0);
lift.setFF(0);
lift.setOutputRange(-1, 1);

lift2.setP(1);
lift2.setI(0);
lift2.setD(0);
lift2.setIZone(0);
lift2.setFF(0);
lift2.setOutputRange(-1, 1);
// setting PID values
lift.setReference(280, ControlType.kPosition);
lift2.setReference(-280, ControlType.kPosition);//orginal 320
}
public void GoToMidHatchPosition() { // moving to given position
// setting PID values
lift.setP(1);
lift.setI(0);
lift.setD(0);
lift.setIZone(0);
lift.setFF(0);
lift.setOutputRange(-1, 1);

lift2.setP(1);
lift2.setI(0);
lift2.setD(0);
lift2.setIZone(0);
lift2.setFF(0);
lift2.setOutputRange(-1, 1);
// setting PID values
lift.setReference(135, ControlType.kPosition);
lift2.setReference(-135, ControlType.kPosition);//orginal 320
}
public void GoToLowHatchPosition() { // moving to given position
// setting PID values
lift.setP(1);
lift.setI(0);
lift.setD(0);
lift.setIZone(0);
lift.setFF(0);
lift.setOutputRange(-1, 1);

lift2.setP(1);
lift2.setI(0);
lift2.setD(0);
lift2.setIZone(0);
lift2.setFF(0);
lift2.setOutputRange(-1, 1);
// setting PID values
lift.setReference(20, ControlType.kPosition);
lift2.setReference(-20, ControlType.kPosition);//orginal 320
}
public void GoToCargoShipPosition() { // moving to given position
  // setting PID values
  lift.setP(1);
  lift.setI(0);
  lift.setD(0);
  lift.setIZone(0);
  lift.setFF(0);
  lift.setOutputRange(-1, 1);
  
  lift2.setP(1);
  lift2.setI(0);
  lift2.setD(0);
  lift2.setIZone(0);
  lift2.setFF(0);
  lift2.setOutputRange(-1, 1);
  // setting PID values
  lift.setReference(100, ControlType.kPosition);
  lift2.setReference(-100, ControlType.kPosition);//orginal 320
  }
  public void movetoGrabHatch() {
          // setting PID values
          lift.setP(1);
          lift.setI(0);
          lift.setD(0);
          lift.setIZone(0);
          lift.setFF(0);
          lift.setOutputRange(-1, 1);
      
          lift2.setP(1);
          lift2.setI(0);
          lift2.setD(0);
          lift2.setIZone(0);
          lift2.setFF(0);
          lift2.setOutputRange(-1, 1);
          // setting PID values
          lift.setReference(20, ControlType.kPosition);
          lift2.setReference(-20, ControlType.kPosition);//orginal 320
  }
  public void movetoHome() {

    // setting PID values
    lift.setP(1);
    lift.setI(0);
    lift.setD(0);
    lift.setIZone(0);
    lift.setFF(0);
    lift.setOutputRange(-1, 1);

    lift2.setP(1);
    lift2.setI(0);
    lift2.setD(0);
    lift2.setIZone(0);
    lift2.setFF(0);
    lift2.setOutputRange(-1, 1);
    // setting PID values
    lift.setReference(1, ControlType.kPosition);
    lift2.setReference(-1, ControlType.kPosition);//orginal 320
}
}
