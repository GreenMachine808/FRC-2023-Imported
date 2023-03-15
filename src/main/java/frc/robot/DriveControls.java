/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
/**
 * Custom drive controls class to organize all the driver buttons and stick input
 */
public class DriveControls {
  
  private final Joystick joystick0 = new Joystick(0);
  private final Joystick joystick1 = new Joystick(1);
  private final Joystick joystick2 = new Joystick(2);

  public final JoystickButton slowDriveMode = new JoystickButton(joystick0, 5);
  public final JoystickButton fastDriveMode = new JoystickButton(joystick0, 2);
  public final JoystickButton fullDriveMode = new JoystickButton(joystick0, 4);

  public final JoystickButton slowTurnMode = new JoystickButton(joystick1, 1);
  public final JoystickButton fastTurnMode = new JoystickButton(joystick1, 10);
  
  
  public double getForward() { return -joystick0.getRawAxis(1); }
  public double getStrafe() { return joystick0.getRawAxis(0); }
  public double getYaw() { return joystick1.getRawAxis(0); }
  public double getElevatorAxis() { return joystick2.getRawAxis(1); }

  public final JoystickButton elevatorRetract = new JoystickButton(joystick2, 1);
  public final JoystickButton elevatorLow = new JoystickButton(joystick2, 4);
  public final JoystickButton elevatorMid = new JoystickButton(joystick2, 2);
  public final JoystickButton elevatorHigh = new JoystickButton(joystick2, 5);

  public final JoystickButton clawOpen = new JoystickButton(joystick2, 7);
  public final JoystickButton clawClose = new JoystickButton(joystick2, 6);


  public final JoystickButton runIntakeForward = new JoystickButton(joystick1, 2);
  public final JoystickButton runIntakeReverse = new JoystickButton(joystick1, 3);

  public final JoystickButton runIntakeTwo = new JoystickButton(joystick1, 5);
 
  public final JoystickButton resetDrive = new JoystickButton(joystick2, 11);

  public final JoystickButton makeWheelsSideways = new JoystickButton(joystick0, 7);
  public final JoystickButton balanceRobotForward = new JoystickButton(joystick0, 11);
  public final JoystickButton balanceRobotBackward = new JoystickButton(joystick0, 10);
  
}