// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.delayedAction;

/** An example command that uses an example subsystem. */
public class runShooterHigh extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_subsystem;
  private delayedAction delayConveyor;
  private boolean RUNITYEA = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public runShooterHigh(ShooterSubsystem subsystem) {
    m_subsystem = subsystem;
    delayConveyor = new delayedAction(750L, () -> RUNITYEA = true);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // This might work. If not, switch the action to flipping a boolean that gets checked in execute.
    new Thread(delayConveyor).start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Can all of this be moved into initialize?
    // What?

    if (RUNITYEA) {
      m_subsystem.runConveyorBelt(false);
    }
    m_subsystem.runShooter(0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RUNITYEA = false;
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    RUNITYEA = false;
    return false;
  }
}
