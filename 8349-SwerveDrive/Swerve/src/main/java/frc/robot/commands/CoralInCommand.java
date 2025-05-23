// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClawInOutSubsystem;

import com.ctre.phoenix.time.StopWatch;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class CoralInCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClawInOutSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  StopWatch timer = new StopWatch();
  public CoralInCommand(ClawInOutSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_subsystem.clawIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.clawStopAuto();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return timer.getDuration() > 7;
  }
}
