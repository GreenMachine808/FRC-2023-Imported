/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.runDrive;
import frc.robot.commands.runIntake;
import frc.robot.commands.runIntakeReverse;
import frc.robot.commands.runIntakeTwo;
import frc.robot.commands.runShooter;
import frc.robot.commands.runShooterHigh;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem robotDrive = new SwerveSubsystem();
  private final HangSubsystem hang = new HangSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();

  private final DriveControls controls = new DriveControls();

  private double speedMod = 0.4;
  private double turnMod = 0.4;
  

  //private final Command simpleAuto = new simpleAutonomous(hang, shooter, robotDrive);
  //private final Command driveBackAuto = new commandBaseAuto(hang, robotDrive, shooter);
  private final Command runDriveAuto = new runDrive(hang, shooter, robotDrive);

  
  //private final Command commandBaseAuto = new commandBaseAuto(robotDrive, shooter);
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    robotDrive.setDefaultCommand(
      new RunCommand(() -> robotDrive.drive(
          modifyDriveInput(controls.getForward()),
          modifyDriveInput(controls.getStrafe()),
          modifyTurnInput(controls.getYaw() * 0.7)), robotDrive ));
    
    hang.setDefaultCommand(new RunCommand(() -> hang.moveElevator(controls.getElevatorAxis() * 0.5), hang));
    /* hang.setDefaultCommand(new ParallelCommandGroup(
        new RunCommand(() -> hang.moveElevator(controls.getElevatorAxis() * 0.5)),
        new RunCommand(() -> hang.popWeightServo(false), hang) ));
    */
    hang.weightdropper.setAngle(180);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Fast drive is toggled, and slow mode is a hold to activate. Need to implement:
    // 1. Smoothing between values
    // 2. SmartDashboard data so that sprinting displayes as toggled

    //Make this a .whenHeld? Want to make this consistant?
    controls.fastDriveMode.whileHeld(new StartEndCommand(
      () -> robotDrive.runSprint = true,
      () -> robotDrive.runSprint = false ));
    controls.slowDriveMode.whileHeld(new StartEndCommand(
      () -> robotDrive.runSlow = true, 
      () -> robotDrive.runSlow = false ));
    /* controls.fullDriveMode.toggleWhenPressed(new StartEndCommand(
      () -> robotDrive.runFull = true, 
      () -> robotDrive.runFull = false ));
    */

    /* controls.fastTurnMode.toggleWhenPressed(new StartEndCommand(
      () -> robotDrive.turnSprint = true, 
      () -> robotDrive.turnSprint = false ));
      */
    controls.slowTurnMode.toggleWhenPressed(new StartEndCommand(
      () -> robotDrive.turnSlow = true,
      () -> robotDrive.turnSlow = false ));

    controls.dropElevator0_0.toggleWhenPressed(new StartEndCommand(
      () -> hang.popWeightServo(true),
      () -> hang.popWeightServo(false) ));

    controls.resetDrive.whenPressed(new InstantCommand(() -> robotDrive.initDrive()) );
    

    /* controls.dropElevator0_0.whileHeld(new RunCommand(
      () -> hang.popWeightServo(true) ));
    */
    //Should we change this to a more convenient button?
    
    controls.shooter.whileHeld((new runShooter( shooter ) ));
    controls.shooterHigh.whileHeld(new runShooterHigh( shooter ));

    controls.runIntakeForward.whileHeld(new runIntake( shooter ));
    controls.runIntakeReverse.whileHeld(new runIntakeReverse( shooter ));
    controls.runIntakeTwo.whileHeld(new runIntakeTwo( shooter ));

    controls.elevatorState.whenPressed(new InstantCommand(
      () -> hang.toggleElevator() ));
  }
  
  /**
   * 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * 
   */
  
  /* 
  public Command getSimpleAuto() {
    return simpleAuto;//autoMidCommand;
  }
  */

  /* 
  public Command getDriveBackAuto() {
    return driveBackAuto;
  }
  */
  
  /*
  public Command getCommandBaseAuto() {
    return commandBaseAuto;
  }
  */

  public Command getRunDriveAuto() {
    return runDriveAuto;
  }
  
  

  public double modifyDriveInput(double value) {
    // Deadband
    if (Math.abs(value) < 0.1) {
      return 0;
    }

    // Lil easing because I don't like the clicking sound. Need to replace numbers with variables though
    if (robotDrive.runFull) {
      if (speedMod < fullSpeed) {
        speedMod += 0.1; 
      } else { speedMod = fullSpeed; }
    }
      else {
        if (robotDrive.runSlow) {
          if(speedMod > slowSpeed) {
            speedMod -= 0.1;
          } else { speedMod = slowSpeed; }
        } else if (robotDrive.runSprint) {
          if (speedMod < sprintSpeed) {
            speedMod += 0.1;
          } else {speedMod = sprintSpeed; }  
        
        } else {
          if (Math.abs(speedMod - normalSpeed) < 0.1) { speedMod = normalSpeed; }
          else if (speedMod < normalSpeed ) {speedMod += 0.07; }
          else {speedMod -= 0.07; }
        }
      }
    

		// Modify the inputed speed based on which speed mode is currently active
    return value * speedMod;
  }

  public double modifyTurnInput(double value) {
    // Deadband
    if (Math.abs(value) < 0.1) {
      return 0;
    }

    // Lil easing because I don't like the clicking sound. Need to replace numbers with variables though
    if (robotDrive.turnSlow) {
      turnMod = slowSpeed;
    } else {
      turnMod = normalTurn;
    }

		// Modify the inputed speed based on which speed mode is currently active
    return value * turnMod;
  }

  public SwerveSubsystem getSwerve() {
    return robotDrive;
  }

  public double getSpeedMod() {
    return speedMod;
  }

  public double getTurnMod(){
    return turnMod;
  }

   public double getPSI() {
    return hang.phCompressor.getPressure();
  }
  
}