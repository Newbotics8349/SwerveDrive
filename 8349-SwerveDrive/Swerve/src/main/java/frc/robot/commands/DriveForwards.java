// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix.time.StopWatch;

/** An example command that uses an example subsystem. */
public class DriveForwards extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem m_subsystem;
  private StopWatch stopWatch = new StopWatch();
  double x;
  double y;
  double rotationRadians;
  double end;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveForwards(SwerveSubsystem subsystem, double x){//, double y, double rotationRadians, double end) {
    this.x = x;
    this.y = y;
    this.rotationRadians = rotationRadians;
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.end = end;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopWatch.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.driveFieldOriented(new ChassisSpeeds(x, y, rotationRadians));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stopWatch.getDuration() > end;
  }
}
