// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/** A command that uses a swerve subsystem and x and y values to move the bot relative to itself. */
public class RelativeMovementCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField", "unused"})
  private final SwerveSubsystem m_SwerveSubsystem;
  private double x;
  private double y;

  /**
   * Creates a new RelativeMovementCommand.
   *
   * @param swerveSubsystem The subsystem used by this command.
   * @param x The forward speed of the bot.
   * @param y The left speed of the bot.
   */
  public RelativeMovementCommand(SwerveSubsystem swerveSubsystem, double x, double y) {
    this.x = x;
    this.y = y;
    m_SwerveSubsystem = swerveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_SwerveSubsystem.driveRobotOriented(new ChassisSpeeds(x,y,0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.driveRobotOriented(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
