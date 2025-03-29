// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;

/** A command that uses an April Tag Vision to align with given tag. */
public class AprilTagAlignCommandPathPlanner extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField", "unused"})
  private final SwerveSubsystem m_swerveSubsystem;
  private final AprilTagSubsystem m_aprilTagSubsystem;
  private int TagId;
  private Pose2d TagPosition;
  private Command pathCommand;
  private BooleanSupplier stopCondition;
  /**
   * Creates a new AprilTagAlignCommand.
   *
   * @param swerveSubsystem The swerve subsystem used by this command.
   * @param aprilTagSubsystem The April Tag subsystem used by this command.
   * @param TagId The ID of the tag to align with.
   */
  public AprilTagAlignCommandPathPlanner(SwerveSubsystem swerveSubsystem, AprilTagSubsystem aprilTagSubsystem, int TagId, BooleanSupplier stopCondition) {
    this.TagId = TagId;
    this.stopCondition = stopCondition;
    m_swerveSubsystem = swerveSubsystem;
    m_aprilTagSubsystem = aprilTagSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem, aprilTagSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Aligning to Tag");
    //get the 2dpose of the tag which is relative to the bot
    this.TagPosition = m_aprilTagSubsystem.getCameraToTagPose(this.TagId);
    if (this.TagPosition == null) {
      System.out.println("Tag not found, try again");
      return;
    }
    //translate position to 1 meter in front of tag
    this.TagPosition = this.TagPosition.plus(new Transform2d(1,0,new Rotation2d(0)));

    //gets the distance to the goal postion
    double relativex = this.TagPosition.getX();
    double relativey = this.TagPosition.getY();

    //gets the current angle of the tag
    double currentAngleRadians = this.TagPosition.getRotation().getRadians();
    //determines which way the bot will turn to face the tag
    double directionMultiplier = currentAngleRadians < 0
        ? 1
        : -1;
    //determines how much the bot needs to rotate
    double RotationNeeded = (Math.PI - Math.abs(currentAngleRadians)) * directionMultiplier ;

    pathCommand = m_swerveSubsystem.followPathCommand(relativex, relativey, new Rotation2d(RotationNeeded))
      .until(stopCondition).andThen(m_swerveSubsystem.resetTrajectory());
    pathCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Tag Finished Aligning");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
