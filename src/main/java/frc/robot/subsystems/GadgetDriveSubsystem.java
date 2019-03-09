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
import frc.robot.*; 
/**
 * Add your docs here.
 */
public class GadgetDriveSubsystem extends Subsystem {
  WPI_TalonSRX SRXgadgetDrive = new WPI_TalonSRX(OI.SRXgadgetDrive);
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void driveToPosition1() {
    int kTimeoutMs = 10;
		int kPIDLoopIdx = 0;

    int absolutePosition = SRXgadgetDrive.getSelectedSensorPosition(kTimeoutMs) & 0xFFF;

		//SRXgadgetDrive.setSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);
		SRXgadgetDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
    SRXgadgetDrive.setSensorPhase(false); //rm invert
    SRXgadgetDrive.setInverted(false);
		SRXgadgetDrive.selectProfileSlot(0, kPIDLoopIdx);
		SRXgadgetDrive.configNominalOutputForward(0, kTimeoutMs);
		SRXgadgetDrive.configNominalOutputReverse(0, kTimeoutMs);
		SRXgadgetDrive.configPeakOutputForward(1, kTimeoutMs);
		SRXgadgetDrive.configPeakOutputReverse(-1, kTimeoutMs);
		SRXgadgetDrive.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);
		SRXgadgetDrive.config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		SRXgadgetDrive.config_kP(kPIDLoopIdx, 17, kTimeoutMs);
		SRXgadgetDrive.config_kI(kPIDLoopIdx, 0, kTimeoutMs);
		SRXgadgetDrive.config_kD(kPIDLoopIdx, 0, kTimeoutMs);
		SRXgadgetDrive.setSelectedSensorPosition(0, 0, 0);


    SRXgadgetDrive.set(ControlMode.Position, 100);
  }
  public void manuadrive() {
    SRXgadgetDrive.set(-1); 
  }
  public void stopdrive() {
    SRXgadgetDrive.set(0);
  }
}
