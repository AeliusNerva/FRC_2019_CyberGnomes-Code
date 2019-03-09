/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.OI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * Add your docs here.
 */
public class GadgetLiftSubsystem extends Subsystem {
  // defining motors and encoters 
  CANSparkMax MAXgadgetlift = new CANSparkMax(OI.MAXleftGadget, MotorType.kBrushless);
  CANSparkMax MAXgadgetlift2 = new CANSparkMax(OI.MAXrightgadget, MotorType.kBrushless);
  public CANPIDController gadgetlift = new CANPIDController(MAXgadgetlift);
  public CANPIDController gadgetlift2 = new CANPIDController(MAXgadgetlift2);
  public CANEncoder Encoder = new CANEncoder(MAXgadgetlift);
  public CANEncoder Encoder2 = new CANEncoder(MAXgadgetlift2);
  // defining motors and encoters 

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void reset() {
    SmartDashboard.putNumber("encoder1", Encoder.getPosition());
    SmartDashboard.putNumber("encoder2", Encoder2.getPosition());
    Encoder.setPosition(0);
    Encoder2.setPosition(0);
  }
  public void up() {
    SmartDashboard.putNumber("encoder1", Encoder.getPosition());
    SmartDashboard.putNumber("encoder2", Encoder2.getPosition());
    MAXgadgetlift.set(.5);
    MAXgadgetlift2.set(.5);
  }
  public void down() {
    SmartDashboard.putNumber("encoder1", Encoder.getPosition());
    SmartDashboard.putNumber("encoder2", Encoder2.getPosition());
   MAXgadgetlift.set(-.5);
   MAXgadgetlift2.set(-.5);
  }
  public void stop() {
    SmartDashboard.putNumber("encoder1", Encoder.getPosition());
    SmartDashboard.putNumber("encoder2", Encoder2.getPosition());
    MAXgadgetlift.set(0);
    MAXgadgetlift2.set(0);
  }
  public void doubleDown() {
    SmartDashboard.putNumber("encoder1", Encoder.getPosition());
    SmartDashboard.putNumber("encoder2", Encoder2.getPosition());
    // setting PID values
    gadgetlift.setP(2);
    gadgetlift.setI(0);
    gadgetlift.setD(.5);
    gadgetlift.setIZone(0);
    gadgetlift.setFF(0);
    gadgetlift.setOutputRange(-1, 1);

    gadgetlift2.setP(2);
    gadgetlift2.setI(0);
    gadgetlift2.setD(.75);
    gadgetlift2.setIZone(0);
    gadgetlift2.setFF(0);
    gadgetlift2.setOutputRange(-1, 1);
    // setting PID values
    gadgetlift.setReference(75, ControlType.kPosition);
    gadgetlift2.setReference(75, ControlType.kPosition);
  }
  public void frontUp() {
    SmartDashboard.putNumber("encoder1", Encoder.getPosition());
    SmartDashboard.putNumber("encoder2", Encoder2.getPosition());
    // setting PID values
    gadgetlift.setP(2);
    gadgetlift.setI(0);
    gadgetlift.setD(.5);
    gadgetlift.setIZone(0);
    gadgetlift.setFF(0);
    gadgetlift.setOutputRange(-1, 1);

    gadgetlift2.setP(2);
    gadgetlift2.setI(0);
    gadgetlift2.setD(.75);
    gadgetlift2.setIZone(0);
    gadgetlift2.setFF(0);
    gadgetlift2.setOutputRange(-1, 1);
    // setting PID values
    gadgetlift2.setReference(0, ControlType.kPosition);
  }
  public void backUp() {
    SmartDashboard.putNumber("encoder1", Encoder.getPosition());
    SmartDashboard.putNumber("encoder2", Encoder2.getPosition());
    // setting PID values
    gadgetlift.setP(2);
    gadgetlift.setI(0);
    gadgetlift.setD(.5);
    gadgetlift.setIZone(0);
    gadgetlift.setFF(0);
    gadgetlift.setOutputRange(-1, 1);

    gadgetlift2.setP(2);
    gadgetlift2.setI(0);
    gadgetlift2.setD(.75);
    gadgetlift2.setIZone(0);
    gadgetlift2.setFF(0);
    gadgetlift2.setOutputRange(-1, 1);
    // setting PID values
    gadgetlift.setReference(0, ControlType.kPosition);
  }

}
