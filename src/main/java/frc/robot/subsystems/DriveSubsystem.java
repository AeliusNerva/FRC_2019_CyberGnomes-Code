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
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.OI;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Add your docs here.
 */

public class DriveSubsystem extends Subsystem implements PIDOutput{
	AHRS ahrs;
	Joystick _joy = new Joystick(0);
	Joystick _bakcupJoy = new Joystick(1);
	double radians, angle, temp, A2, B2, R, A, B, C, D, ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4, max, currentAngle,
			rotationAmmount, FWD, STR, RCW, currentAngle2, currentAngle3, currentAngle4, rotationAmmount2,
			rotationAmmount3, rotationAmmount4,offsetangle,disfromtarget; // defining variables 
	boolean firstrun = false;
	// defining motor controlers and encoders
	WPI_TalonSRX SRXsteer = new WPI_TalonSRX(OI.SRXtoprightsteer);
	WPI_TalonSRX SRXsteer2 = new WPI_TalonSRX(OI.SRXtopleftsteer);
	WPI_TalonSRX SRXsteer3 = new WPI_TalonSRX(OI.SRXbottomleftsteer);
	WPI_TalonSRX SRXsteer4 = new WPI_TalonSRX(OI.SRXbottomrightsteer);
	CANSparkMax MAXdrive = new CANSparkMax(OI.MAXtoprightdrive, MotorType.kBrushless);
	CANSparkMax MAXdrive2 = new CANSparkMax(OI.MAXtopleftdrive, MotorType.kBrushless);
	CANSparkMax MAXdrive3 = new CANSparkMax(OI.MAXbottomleftdrive, MotorType.kBrushless);
	CANSparkMax MAXdrive4 = new CANSparkMax(OI.MAXbottomrightdrive, MotorType.kBrushless);
	public CANPIDController drive1 = new CANPIDController(MAXdrive);
	public CANEncoder Encoder = new CANEncoder(MAXdrive);
	public CANPIDController drive2 = new CANPIDController(MAXdrive2);
	public CANEncoder Encoder2 = new CANEncoder(MAXdrive2);
	public CANPIDController drive3 = new CANPIDController(MAXdrive3);
	public CANEncoder Encoder3 = new CANEncoder(MAXdrive3);
	public CANPIDController drive4 = new CANPIDController(MAXdrive4);
	public CANEncoder Encoder4 = new CANEncoder(MAXdrive4);
	// defining motor controlers and encoders

	@Override
	public void initDefaultCommand() {
	}

	public void init() {
		Encoder.setPosition(0);
		Encoder2.setPosition(0);
		Encoder3.setPosition(0);
		Encoder4.setPosition(0);
		// init for navX imu
		ahrs = new AHRS(SPI.Port.kMXP);
		ahrs.resetDisplacement();
		ahrs.reset();
		angle = ahrs.getYaw(); // defining variable for gyro
		// init for navX imu

		// PID config
      // setting PID values


		int kTimeoutMs = 10;
		int kPIDLoopIdx = 0;
		int rampvelocity = 15000;
		int rampaccel = 16000;
		int absolutePosition = SRXsteer.getSelectedSensorPosition(kTimeoutMs) & 0xFFF;
		int P = 1;
		int I = 0;
		int D = 0;
		int F = 0;

		SRXsteer.setSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);
		SRXsteer.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
		SRXsteer.setSensorPhase(false);

		SRXsteer.selectProfileSlot(0, kPIDLoopIdx);
		SRXsteer.configNominalOutputForward(0, kTimeoutMs);
		SRXsteer.configNominalOutputReverse(0, kTimeoutMs);
		SRXsteer.configPeakOutputForward(1, kTimeoutMs);
		SRXsteer.configPeakOutputReverse(-1, kTimeoutMs);
		SRXsteer.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);
		SRXsteer.configMotionCruiseVelocity(rampvelocity, kTimeoutMs);
		SRXsteer.configMotionAcceleration(rampaccel, kTimeoutMs);

		SRXsteer.config_kF(kPIDLoopIdx, F, kTimeoutMs);
		SRXsteer.config_kP(kPIDLoopIdx, P, kTimeoutMs);
		SRXsteer.config_kI(kPIDLoopIdx, I, kTimeoutMs);
		SRXsteer.config_kD(kPIDLoopIdx, D, kTimeoutMs);
		SRXsteer.setSelectedSensorPosition(0, 0, 0);

		int absolutePosition2 = SRXsteer2.getSelectedSensorPosition(kTimeoutMs) & 0xFFF;

		SRXsteer2.setSelectedSensorPosition(absolutePosition2, kPIDLoopIdx, kTimeoutMs);
		SRXsteer2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
		SRXsteer2.setSensorPhase(false);

		SRXsteer2.configNominalOutputForward(0, kTimeoutMs);
		SRXsteer2.configNominalOutputReverse(0, kTimeoutMs);
		SRXsteer2.configPeakOutputForward(1, kTimeoutMs);
		SRXsteer2.configPeakOutputReverse(-1, kTimeoutMs);
		SRXsteer2.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);
		SRXsteer2.configMotionCruiseVelocity(rampvelocity, kTimeoutMs);
		SRXsteer2.configMotionAcceleration(rampaccel, kTimeoutMs);

		SRXsteer2.config_kF(kPIDLoopIdx, F, kTimeoutMs);
		SRXsteer2.config_kP(kPIDLoopIdx, P, kTimeoutMs);
		SRXsteer2.config_kI(kPIDLoopIdx, I, kTimeoutMs);
		SRXsteer2.config_kD(kPIDLoopIdx, D, kTimeoutMs);
		SRXsteer2.setSelectedSensorPosition(0, 0, 0);

		int absolutePosition3 = SRXsteer3.getSelectedSensorPosition(kTimeoutMs) & 0xFFF;

		SRXsteer3.setSelectedSensorPosition(absolutePosition3, kPIDLoopIdx, kTimeoutMs);
		SRXsteer3.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
		SRXsteer3.setSensorPhase(false);

		SRXsteer3.configNominalOutputForward(0, kTimeoutMs);
		SRXsteer3.configNominalOutputReverse(0, kTimeoutMs);
		SRXsteer3.configPeakOutputForward(1, kTimeoutMs);
		SRXsteer3.configPeakOutputReverse(-1, kTimeoutMs);
		SRXsteer3.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);
		SRXsteer3.configMotionCruiseVelocity(rampvelocity, kTimeoutMs);
		SRXsteer3.configMotionAcceleration(rampaccel, kTimeoutMs);

		SRXsteer3.config_kF(kPIDLoopIdx, F, kTimeoutMs);
		SRXsteer3.config_kP(kPIDLoopIdx, P, kTimeoutMs);
		SRXsteer3.config_kI(kPIDLoopIdx, I, kTimeoutMs);
		SRXsteer3.config_kD(kPIDLoopIdx, D, kTimeoutMs);
		SRXsteer3.setSelectedSensorPosition(0, 0, 0);

		int absolutePosition4 = SRXsteer4.getSelectedSensorPosition(kTimeoutMs) & 0xFFF;

		SRXsteer4.setSelectedSensorPosition(absolutePosition4, kPIDLoopIdx, kTimeoutMs);
		SRXsteer4.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
		SRXsteer4.setSensorPhase(false);

		SRXsteer4.configNominalOutputForward(0, kTimeoutMs);
		SRXsteer4.configNominalOutputReverse(0, kTimeoutMs);
		SRXsteer4.configPeakOutputForward(1, kTimeoutMs);
		SRXsteer4.configPeakOutputReverse(-1, kTimeoutMs);
		SRXsteer4.configMotionCruiseVelocity(rampvelocity, kTimeoutMs);
		SRXsteer4.configMotionAcceleration(rampaccel, kTimeoutMs);

		SRXsteer4.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);

		SRXsteer4.config_kF(kPIDLoopIdx, F, kTimeoutMs);
		SRXsteer4.config_kP(kPIDLoopIdx, P, kTimeoutMs);
		SRXsteer4.config_kI(kPIDLoopIdx, I, kTimeoutMs);
		SRXsteer4.config_kD(kPIDLoopIdx, D, kTimeoutMs);
		SRXsteer4.setSelectedSensorPosition(0, 0, 0);

		// PID config

	}

	public void Swerve() {

		try {
			//zero();
			NetworkTableInstance inst = NetworkTableInstance.getDefault();
			table = inst.getTable("CVResultsTable");
			VisionValues = table.getEntry("VisionResults").getString("").split(",");
			XOffset = Double.parseDouble(VisionValues[2]);
			offsetangle = Double.parseDouble(VisionValues[4]);
			disfromtarget = Double.parseDouble(VisionValues[3]);
			SmartDashboard.putNumber("xOFFSET", Double.parseDouble(VisionValues[2]));
			SmartDashboard.putNumber("offset", Double.parseDouble(VisionValues[4]));
		} catch (Exception e) {
	
		}
		angle = ahrs.getYaw(); // defining variable for gyro
		radians = angle * Math.PI / 180;
		currentAngle = SRXsteer.getSelectedSensorPosition(0) / 25.930555555555; // setting the current angle of the
																				// wheel 11.4666 = tick per rotation/360
		currentAngle2 = SRXsteer2.getSelectedSensorPosition(0) / 25.930555555555;
		currentAngle3 = SRXsteer3.getSelectedSensorPosition(0) / 25.930555555555;
		currentAngle4 = SRXsteer4.getSelectedSensorPosition(0) / 25.930555555555;
		// swerve formulas
		FWD = -1*(_joy.getRawAxis(1) + _bakcupJoy.getRawAxis(1));
		STR = _joy.getRawAxis(0) + _bakcupJoy.getRawAxis(0);
		RCW = _joy.getRawAxis(2) + _bakcupJoy.getRawAxis(2);
		if (FWD < .08 && FWD > -.08) {
			FWD = 0;
		}
		if (STR < .08 && STR > -.08) {
			STR = 0;
		}
		if (RCW < .08 && RCW > -.08) {
			RCW = 0;
		}
		temp = FWD * Math.cos(radians) + STR * Math.sin(radians);
		STR = -FWD * Math.sin(radians) + STR * Math.cos(radians);
		FWD = temp;
		R = Math.sqrt((OI.L * OI.L) + (OI.W * OI.W));
		A = STR - RCW * (OI.L / R);
		B = STR + RCW * (OI.L / R);
		C = FWD - RCW * (OI.W / R);
		D = FWD + RCW * (OI.W / R);
		ws1 = Math.sqrt((B * B) + (C * C));
		ws2 = Math.sqrt((B * B) + (D * D));
		ws3 = Math.sqrt((A * A) + (D * D));
		ws4 = Math.sqrt((A * A) + (C * C));
		wa1 = Math.atan2(B, C) * 180 / Math.PI;
		wa2 = Math.atan2(B, D) * 180 / Math.PI;
		wa3 = Math.atan2(A, D) * 180 / Math.PI;	
		wa4 = Math.atan2(A, C) * 180 / Math.PI;
		max = ws1;
		if (ws2 > max) {
			max = ws2;
		}
		if (ws3 > max) {
			max = ws3;
		}
		if (ws4 > max) {
			max = ws4;
		}
		if (max > 1) {
			ws1 /= max;
			ws2 /= max;
			ws3 /= max;
			ws4 /= max;
		}
		// swerve formulas

		// smartdashboard puts

		// smartdashboard puts

		SmartDashboard.putNumber("Encoder4 Position", SRXsteer4.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Encoder3 Position", SRXsteer3.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Encoder Position", SRXsteer.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Encoder2 Position", SRXsteer2.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("encoderdrive1",Encoder.getPosition());
		SmartDashboard.putNumber("encoderdrive2",Encoder2.getPosition());
		SmartDashboard.putNumber("encoderdrive3",Encoder3.getPosition());
		SmartDashboard.putNumber("encoderdrive4",Encoder4.getPosition());
		SmartDashboard.putNumber("angle4", currentAngle4);
		SmartDashboard.putNumber("output4", rotationAmmount4);
		SmartDashboard.putNumber("trtwt4", wa4 - currentAngle4);
		SmartDashboard.putNumber("angle3", currentAngle);
		SmartDashboard.putNumber("output3", rotationAmmount3);
		SmartDashboard.putNumber("trtwt3", wa4 - currentAngle3);

		SmartDashboard.putNumber("WheelAngle", wa1);
		SmartDashboard.putNumber("WheenSpeed", ws1);
		SmartDashboard.putNumber("WheelAngle2", wa2);
		SmartDashboard.putNumber("WheenSpeed2", ws2);
		SmartDashboard.putNumber("WheelAngle3", wa3);
		SmartDashboard.putNumber("WheenSpeed3", ws3);
		SmartDashboard.putNumber("WheelAngle4", wa4);
		SmartDashboard.putNumber("WheenSpeed4", ws4);
		SmartDashboard.putNumber("RotaitonAmount", rotationAmmount);
		SmartDashboard.putNumber("current", SRXsteer4.getOutputCurrent());
		SmartDashboard.putNumber("angle", angle);
		SmartDashboard.putNumber("joyasi1", _joy.getRawAxis(1));
		SmartDashboard.putNumber("joyasi0", _joy.getRawAxis(0));
		SmartDashboard.putNumber("joyasi2", _joy.getRawAxis(2));
		// System.out.println(rotationAmmount);

		rotationAmmount = Math.IEEEremainder(wa1 - currentAngle, 360); // calculating ammount to move wheel
		rotationAmmount2 = Math.IEEEremainder(wa2 - currentAngle2, 360); // calculating ammount to move wheel
		rotationAmmount3 = Math.IEEEremainder(wa3 - currentAngle3, 360); // calculating ammount to move wheel
		rotationAmmount4 = Math.IEEEremainder(wa4 - currentAngle4, 360); // calculating ammount to move wheel


		

		if (FWD < .1 && FWD > -.1 && STR < .1 && STR > -.1 && RCW < .1 && RCW > -.1) { // not letting the wheels move
																						// unitll the joystick is pushed
																						// %10 down
			MAXdrive.set(0);
			MAXdrive2.set(0);
			MAXdrive3.set(0);
			MAXdrive4.set(0);
		} else {
			MAXdrive.set(-ws1); // drining drive wheel off the formulas
			MAXdrive2.set(ws2);
			MAXdrive3.set(ws3);
			MAXdrive4.set(-ws4);
		}

			SRXsteer.set(ControlMode.MotionMagic, (currentAngle + rotationAmmount) * 26.006); // moving the wheel to the
			// required position
			SRXsteer2.set(ControlMode.MotionMagic, (currentAngle2 + rotationAmmount2) * 26.006); // moving the wheel to
																									// the
			// required position
			SRXsteer3.set(ControlMode.MotionMagic, (currentAngle3 + rotationAmmount3) * 26.006); // moving the wheel to
																									// the
			// required position
			SRXsteer4.set(ControlMode.MotionMagic, (currentAngle4 + rotationAmmount4) * 26.006); // moving the wheel to
																									// the
			// required position


		SmartDashboard.putNumber("ANGLE", ahrs.getAngle());
	}


	public void readyToRotate() {
		radians = angle * Math.PI / 180;
		currentAngle = SRXsteer.getSelectedSensorPosition(0) / 25.930555555555; // setting the current angle of the
																				// wheel 11.4666 = tick per rotation/360
		currentAngle2 = SRXsteer2.getSelectedSensorPosition(0) / 25.930555555555;
		currentAngle3 = SRXsteer3.getSelectedSensorPosition(0) / 25.930555555555;
		currentAngle4 = SRXsteer4.getSelectedSensorPosition(0) / 25.930555555555;
		// swerve formulas
		FWD = 0;
		STR = 0;
		RCW = 1;
		if (FWD < .08 && FWD > -.08) {
			FWD = 0;
		}
		if (STR < .08 && STR > -.08) {
			STR = 0;
		}
		if (RCW < .08 && RCW > -.08) {
			RCW = 0;
		}
		temp = FWD * Math.cos(radians) + STR * Math.sin(radians);
		STR = -FWD * Math.sin(radians) + STR * Math.cos(radians);
		FWD = temp;
		R = Math.sqrt((OI.L * OI.L) + (OI.W * OI.W));
		A = STR - RCW * (OI.L / R);
		B = STR + RCW * (OI.L / R);
		C = FWD - RCW * (OI.W / R);
		D = FWD + RCW * (OI.W / R);
		wa1 = Math.atan2(B, C) * 180 / Math.PI;
		wa2 = Math.atan2(B, D) * 180 / Math.PI;
		wa3 = Math.atan2(A, D) * 180 / Math.PI;	
		wa4 = Math.atan2(A, C) * 180 / Math.PI;
		max = ws1;
		if (ws2 > max) {
			max = ws2;
		}
		if (ws3 > max) {
			max = ws3;
		}
		if (ws4 > max) {
			max = ws4;
		}
		if (max > 1) {
			ws1 /= max;
			ws2 /= max;
			ws3 /= max;
			ws4 /= max;
		}

		rotationAmmount = Math.IEEEremainder(wa1 - currentAngle, 360); // calculating ammount to move wheel
		rotationAmmount2 = Math.IEEEremainder(wa2 - currentAngle2, 360); // calculating ammount to move wheel
		rotationAmmount3 = Math.IEEEremainder(wa3 - currentAngle3, 360); // calculating ammount to move wheel
		rotationAmmount4 = Math.IEEEremainder(wa4 - currentAngle4, 360); // calculating ammount to move wheel

			SRXsteer.set(ControlMode.MotionMagic, (currentAngle + rotationAmmount) * 26.006); // moving the wheel to the
			// required position
			SRXsteer2.set(ControlMode.MotionMagic, (currentAngle2 + rotationAmmount2) * 26.006); // moving the wheel to
																									// the
			// required position
			SRXsteer3.set(ControlMode.MotionMagic, (currentAngle3 + rotationAmmount3) * 26.006); // moving the wheel to
																									// the
			// required position
			SRXsteer4.set(ControlMode.MotionMagic, (currentAngle4 + rotationAmmount4) * 26.006); // moving the wheel to
																									// the
			// required position


		SmartDashboard.putNumber("ANGLE", ahrs.getAngle());
	}
	public void movesideways() {
		rotationAmmount = Math.IEEEremainder(270 - currentAngle, 360); // calculating ammount to move wheel
		rotationAmmount2 = Math.IEEEremainder(270 - currentAngle2, 360); // calculating ammount to move wheel
		rotationAmmount3 = Math.IEEEremainder(270 - currentAngle3, 360); // calculating ammount to move wheel
		rotationAmmount4 = Math.IEEEremainder(270 - currentAngle4, 360); // calculating ammount to move wheel

			SRXsteer.set(ControlMode.MotionMagic, (currentAngle + rotationAmmount) * 26.006); // moving the wheel to the
			// required position
			SRXsteer2.set(ControlMode.MotionMagic, (currentAngle2 + rotationAmmount2) * 26.006); // moving the wheel to
																									// the
			// required position
			SRXsteer3.set(ControlMode.MotionMagic, (currentAngle3 + rotationAmmount3) * 26.006); // moving the wheel to
																									// the
			// required position
			SRXsteer4.set(ControlMode.MotionMagic, (currentAngle4 + rotationAmmount4) * 26.006); // moving the wheel to
																									// the
			MAXdrive.set(-.5);
			MAXdrive2.set(-.5);
			MAXdrive3.set(-.5);
			MAXdrive4.set(-.5);
	}
	String[] VisionValues;
	// network table init
	NetworkTable table;
	double XOffset;
	// network table init
	public void movesidewaystoposition() {
		SmartDashboard.putNumber("xOFFSET", Double.parseDouble(VisionValues[2]));
		SmartDashboard.putNumber("encoderdrive1",Encoder.getPosition());
		SmartDashboard.putNumber("encoderdrive2",Encoder2.getPosition());
		SmartDashboard.putNumber("encoderdrive3",Encoder3.getPosition());
		SmartDashboard.putNumber("encoderdrive4",Encoder4.getPosition());
		SmartDashboard.putNumber("aaaa", 90291);
		double speed = .5;
		double P = 1;
		double I = 0;
		double D = .0;
		double F = 0;
		double IZ = 0;
		double ramp = .25;
		MAXdrive.setClosedLoopRampRate(ramp);
		MAXdrive2.setClosedLoopRampRate(ramp);
		MAXdrive3.setClosedLoopRampRate(ramp);
		MAXdrive4.setClosedLoopRampRate(ramp);
		drive1.setP(P);
		drive1.setI(I);
		drive1.setD(D);
		drive1.setIZone(IZ);
		drive1.setFF(F);
		drive1.setOutputRange(-speed, speed);
		drive2.setP(P);
		drive2.setI(I);
		drive2.setD(D);
		drive2.setIZone(IZ);
		drive2.setFF(F);
		drive2.setOutputRange(-speed, speed);
		drive3.setP(P);
		drive3.setI(I);
		drive3.setD(D);
		drive3.setIZone(IZ);
		drive3.setFF(F);
		drive3.setOutputRange(-speed, speed);
		drive4.setP(P);
		drive4.setI(I);
		drive4.setD(D);
		drive4.setIZone(IZ);
		drive4.setFF(F);
		drive4.setOutputRange(-speed, speed);
		rotationAmmount = Math.IEEEremainder(270 - currentAngle, 360); // calculating ammount to move wheel
		rotationAmmount2 = Math.IEEEremainder(270 - currentAngle2, 360); // calculating ammount to move wheel
		rotationAmmount3 = Math.IEEEremainder(270 - currentAngle3, 360); // calculating ammount to move wheel
		rotationAmmount4 = Math.IEEEremainder(270 - currentAngle4, 360); // calculating ammount to move wheel

			SRXsteer.set(ControlMode.MotionMagic, (currentAngle + rotationAmmount) * 26.006); // moving the wheel to the
			// required position
			SRXsteer2.set(ControlMode.MotionMagic, (currentAngle2 + rotationAmmount2) * 26.006); // moving the wheel to
																									// the
			// required position
			SRXsteer3.set(ControlMode.MotionMagic, (currentAngle3 + rotationAmmount3) * 26.006); // moving the wheel to
																									// the
			// required position
			SRXsteer4.set(ControlMode.MotionMagic, (currentAngle4 + rotationAmmount4) * 26.006); // moving the wheel to
																									// the
			try {
				double rotationsPerInch = 1.4556;
				double ticks = (0 - XOffset)/rotationsPerInch;
				drive1.setReference(-ticks, ControlType.kPosition);
				drive2.setReference(ticks, ControlType.kPosition);
				drive3.setReference(ticks, ControlType.kPosition);
				drive4.setReference(-ticks, ControlType.kPosition);
			} catch (Exception e) {
			}

	}	
	public void movefowardtoposition() {
		SmartDashboard.putNumber("xOFFSET", Double.parseDouble(VisionValues[2]));
		SmartDashboard.putNumber("encoderdrive1",Encoder.getPosition());
		SmartDashboard.putNumber("encoderdrive2",Encoder2.getPosition());
		SmartDashboard.putNumber("encoderdrive3",Encoder3.getPosition());
		SmartDashboard.putNumber("encoderdrive4",Encoder4.getPosition());
		SmartDashboard.putNumber("aaaa", 90291);
		double speed = .34;
		double P = 1;
		double I = 0;
		double D = .0;
		double F = 0;
		double IZ = 0;
		double ramp = .25;
		MAXdrive.setClosedLoopRampRate(ramp);
		MAXdrive2.setClosedLoopRampRate(ramp);
		MAXdrive3.setClosedLoopRampRate(ramp);
		MAXdrive4.setClosedLoopRampRate(ramp);
		drive1.setP(P);
		drive1.setI(I);
		drive1.setD(D);
		drive1.setIZone(IZ);
		drive1.setFF(F);
		drive1.setOutputRange(-speed, speed);
		drive2.setP(P);
		drive2.setI(I);
		drive2.setD(D);
		drive2.setIZone(IZ);
		drive2.setFF(F);
		drive2.setOutputRange(-speed, speed);
		drive3.setP(P);
		drive3.setI(I);
		drive3.setD(D);
		drive3.setIZone(IZ);
		drive3.setFF(F);
		drive3.setOutputRange(-speed, speed);
		drive4.setP(P);
		drive4.setI(I);
		drive4.setD(D);
		drive4.setIZone(IZ);
		drive4.setFF(F);
		drive4.setOutputRange(-speed, speed);
		rotationAmmount = Math.IEEEremainder(0 - currentAngle, 360); // calculating ammount to move wheel
		rotationAmmount2 = Math.IEEEremainder(0 - currentAngle2, 360); // calculating ammount to move wheel
		rotationAmmount3 = Math.IEEEremainder(0 - currentAngle3, 360); // calculating ammount to move wheel
		rotationAmmount4 = Math.IEEEremainder(0 - currentAngle4, 360); // calculating ammount to move wheel

			SRXsteer.set(ControlMode.MotionMagic, (currentAngle + rotationAmmount) * 26.006); // moving the wheel to the
			// required position
			SRXsteer2.set(ControlMode.MotionMagic, (currentAngle2 + rotationAmmount2) * 26.006); // moving the wheel to
																									// the
			// required position
			SRXsteer3.set(ControlMode.MotionMagic, (currentAngle3 + rotationAmmount3) * 26.006); // moving the wheel to
																									// the
			// required position
			SRXsteer4.set(ControlMode.MotionMagic, (currentAngle4 + rotationAmmount4) * 26.006); // moving the wheel to
																									// the
			try {
				double rotationsPerInch = 1.4556;
				double ticks = (10 - disfromtarget)/rotationsPerInch;
				drive1.setReference(-ticks, ControlType.kPosition);
				drive2.setReference(ticks, ControlType.kPosition);
				drive3.setReference(ticks, ControlType.kPosition);
				drive4.setReference(-ticks, ControlType.kPosition);
			} catch (Exception e) {
			}

	}	
	public void visionwrite() {
		
		try {
			NetworkTableInstance inst = NetworkTableInstance.getDefault();
			table = inst.getTable("CVResultsTable");
			VisionValues = table.getEntry("VisionResults").getString("").split(",");
			XOffset = Double.parseDouble(VisionValues[2]);
			offsetangle = Double.parseDouble(VisionValues[4]);
			disfromtarget = Double.parseDouble(VisionValues[3]);
		} catch (Exception e) {
	
		}
	}	
	public void stop() {
		SRXsteer.set(0);	
		SRXsteer2.set(0);
		SRXsteer3.set(0);
		SRXsteer4.set(0);
		MAXdrive.set(0);
		MAXdrive2.set(0);
		MAXdrive3.set(0);
		MAXdrive4.set(0);
	}
	static final double kP = 0.02;
    static final double kI = 0.00;
    static final double kD = 0.00;
	static final double kF = 0.00;
	static final double kToleranceDegrees = .1f;
	double rotateToAngleRate;
	PIDController turnController;
	public void PIDinit() {
		try {
			turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
		} catch (Exception e) {
		}
	}		
	public void rotateToAngle(){
		try {
		SmartDashboard.putNumber ("RotateAngle", angle);
		SmartDashboard.putNumber ("Offsetangle", offsetangle);
		double setpoint = angle + offsetangle;
		SmartDashboard.putNumber("setpoint", setpoint);
       	turnController.setInputRange(-180.0f,  180.0f);
        turnController.setOutputRange(-1.0, 1.0);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);
		turnController.setSetpoint(setpoint);
		if (offsetangle > -2 && offsetangle < 2) {
			SmartDashboard.putString("TurnController", "Disabled");
			turnController.disable();
		}
		else {
			turnController.enable();
			SmartDashboard.putString("TurnController", "Enabled");
		}
		MAXdrive.set(-rotateToAngleRate);
		MAXdrive2.set(rotateToAngleRate);
		MAXdrive3.set(rotateToAngleRate);
		MAXdrive4.set(-rotateToAngleRate);
		} catch (Exception e) {
		}
	}
	public void pidWrite(double output) {
        rotateToAngleRate = output;
	}
	public void PIDstop(){
		try {
			SmartDashboard.putString("TurnController", "Disabled");
			turnController.disable();
		} catch (Exception e) {
		}

	}
	public void zero() {
		
		Encoder.setPosition(0);
		Encoder2.setPosition(0);
		Encoder3.setPosition(0);
		Encoder4.setPosition(0);
	}
}
