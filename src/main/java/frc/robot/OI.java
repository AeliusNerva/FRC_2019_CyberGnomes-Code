/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;
import frc.robot.commands.lowHatch;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  Joystick _joy = new Joystick(0);
  Button button1 = new JoystickButton(_joy, 1);
  public OI() {

    button1.whenPressed(new lowHatch());
  } 
  public static double L = 24.05; 
  public static double W = 22.875;
  public static double inch = 200;
  public static int MAXtopleftdrive = 17;
  public static int MAXtoprightdrive = 1;
  public static int MAXbottomleftdrive = 7;
  public static int MAXbottomrightdrive = 3;
  public static int MAXrightlift = 5;
  public static int MAXleftlift = 15;
  public static int MAXleftGadget = 6;
  public static int MAXrightgadget = 18;
  public static int SPXintake = 8;
  public static int SRXtopleftsteer = 9;
  public static int SRXtoprightsteer = 10;
  public static int SRXbottomleftsteer = 11;
  public static int SRXbottomrightsteer = 12;
  public static int SRXwrist = 13;
  public static int SRXgadgetDrive = 14;

    
}
  

  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

