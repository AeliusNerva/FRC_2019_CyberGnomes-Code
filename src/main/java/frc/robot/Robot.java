/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//imports
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.*; 
import frc.robot.OI;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//imports

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI m_oi;
  WPI_TalonSRX SRXgadgetDrive = new WPI_TalonSRX(OI.SRXgadgetDrive);

  WPI_TalonSRX SRXwrist = new WPI_TalonSRX(OI.SRXwrist);
  int kTimeoutMs = 10;
  int kPIDLoopIdx = 0;
  double test;
  String[] VisionValues;
  // network table init
  NetworkTableEntry Area; 
  NetworkTable table;
  // network table init

  // public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  // public static OI m_oi; 

  Command m_autonomousCommand;
  Joystick _joy = new Joystick(0);
  Joystick _backupJoy = new Joystick(1);
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  // command and subsystem calls
  public static DriveSubsystem drive = null;
  public static LifterSubsystem lift = null;
  public static WristSubsystem wrist = null;
  public static IntakeSubsystem intake = null;
  public static PnumaticSubsystem pnumatics = null;
  public static GadgetLiftSubsystem gadget = null;
  public static GadgetDriveSubsystem gadgetdrive = null;
  // up subsystems down commands
  public static RobotFunctions robot = null;
  public static goGoGadgetFrontDown frontdown = null;
  public static goGoGadgetbackdown backdown = null;
  public static goGoGadgetFullUp fullup = null;
  public static goGoGadget goGoGadget = null;
  public static goGoGadgetDrive1 drive1 = null;
  public static goGogadgetdrive2 drive2 = null;
  public static goGogadgetdrive3 drive3 = null;
  public static lowHatch LowHatch = null;
  public static zeroDrive zeroDrive = null;
  CANSparkMax MAXdrive = new CANSparkMax(OI.MAXtoprightdrive, MotorType.kBrushless);
	CANSparkMax MAXdrive2 = new CANSparkMax(OI.MAXtopleftdrive, MotorType.kBrushless);
	CANSparkMax MAXdrive3 = new CANSparkMax(OI.MAXbottomleftdrive, MotorType.kBrushless);
	CANSparkMax MAXdrive4 = new CANSparkMax(OI.MAXbottomrightdrive, MotorType.kBrushless);
	public CANEncoder Encoder = new CANEncoder(MAXdrive);
	public CANEncoder Encoder2 = new CANEncoder(MAXdrive2);
	public CANEncoder Encoder3 = new CANEncoder(MAXdrive3);
  public CANEncoder Encoder4 = new CANEncoder(MAXdrive4);
  private Boolean ran = false;
	// defining motor controlers and encoders
  // command and subsystem calls
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    ran = true;
    m_oi = new OI();
		int kTimeoutMs = 10;
		int kPIDLoopIdx = 0;

    int absolutePosition = SRXwrist.getSelectedSensorPosition(kTimeoutMs) & 0xFFF;
    double encoderval = 0;

		SRXwrist.setSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);
		SRXwrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
    SRXwrist.setInverted(false);
		SRXwrist.selectProfileSlot(0, kPIDLoopIdx);
		SRXwrist.configNominalOutputForward(0, kTimeoutMs);
		SRXwrist.configNominalOutputReverse(0, kTimeoutMs);
		SRXwrist.configPeakOutputForward(1, kTimeoutMs);
		SRXwrist.configPeakOutputReverse(-1, kTimeoutMs);
		SRXwrist.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);
		SRXwrist.configMotionCruiseVelocity(15000, kTimeoutMs);
		SRXwrist.configMotionAcceleration(200, kTimeoutMs);// orignal 200

		SRXwrist.config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		SRXwrist.config_kP(kPIDLoopIdx, 40, kTimeoutMs);
		SRXwrist.config_kI(kPIDLoopIdx, 0, kTimeoutMs);
		SRXwrist.config_kD(kPIDLoopIdx, 0, kTimeoutMs);
		//SRXwrist.setSelectedSensorPosition(0, 0, 0);
           
           
                   //setting pid values
                   SmartDashboard.putNumber("wristpos", SRXwrist.getSelectedSensorPosition(0));
    Encoder.setPosition(0);
    Encoder2.setPosition(0);
    Encoder3.setPosition(0);
    Encoder4.setPosition(0);
    SRXwrist.setSelectedSensorPosition(0, 0, 0);
    // network table init
   // NetworkTableInstance inst = NetworkTableInstance.getDefault();
   // table = inst.getTable("CVResultsTable");
    //Area = table.getEntry("area");
    // network table init 
    // command and subsystem calls
    drive = new DriveSubsystem();
    lift = new LifterSubsystem();
    wrist = new WristSubsystem();
    intake = new IntakeSubsystem();
    pnumatics = new PnumaticSubsystem();
    gadget = new GadgetLiftSubsystem();
    gadgetdrive = new GadgetDriveSubsystem();
    // up subsystems down commands
    robot = new RobotFunctions();
    frontdown = new goGoGadgetFrontDown();
    backdown = new goGoGadgetbackdown();
    fullup = new goGoGadgetFullUp();
    drive1 = new goGoGadgetDrive1();
    drive2 = new goGogadgetdrive2();
    drive3 = new goGogadgetdrive3();
    goGoGadget = new goGoGadget();
    LowHatch = new lowHatch();
    //xOfset = new XOfset();
    
    //drive.PIDinit();
    drive.init();
    drive.PIDinit();
    // command and subsystem calls

    // m_oi = new OI();
    // m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    // SmartDashboard.putData("Auto mode", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  public double testvar;
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("testvar", testvar);

    SRXwrist.setSensorPhase(false);
    SRXwrist.setInverted(true);
    SmartDashboard.putNumber("gadgetdrivewheelenc", SRXgadgetDrive.getSelectedSensorPosition());
   if (_backupJoy.getPOV() == 90 || _backupJoy.getPOV() == 270) {
      goGoGadget.start();
    }
    if (_backupJoy.getRawButton(10)) {
      goGoGadget.cancel();
      gadget.stop();
    }
  
    
   // if (_joy.getRawButton(1)) {
   //   SmartDashboard.putNumber("check", 87987);
   //   LowHatch.start();
      //drive.moveforwardtoposition();
      //drive.readyToRotate();
      //drive.rotateToAngle();
   //   robot.cancel();
   // }
    //else {
      if(_joy.getRawButton(1)) {
        robot.cancel();
      }
      else {
        LowHatch.cancel();
        robot.start();
        drive.PIDstop();
      }
    //}

   /// SmartDashboard.putNumber("pov", _joy.getPOV());                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   m                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                ,
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
