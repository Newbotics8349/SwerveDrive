// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.ctre.phoenix.time.StopWatch;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/** A command that uses an April Tag Vision to align with given tag. */
public class AprilTagAlignCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField", "unused"})
  private final SwerveSubsystem m_swerveSubsystem;
  private final AprilTagSubsystem m_aprilTagSubsystem;
  private StopWatch stopWatch = new StopWatch();
  private int TagId;
  private double RunTime;
  private Pose2d TagPosition;
  private double ySpeed;
  private double xSpeed;
  private double rSpeed;
  /**
   * Creates a new AprilTagAlignCommand.
   *
   * @param swerveSubsystem The swerve subsystem used by this command.
   * @param aprilTagSubsystem The April Tag subsystem used by this command.
   * @param TagId The ID of the tag to align with.
   */
  public AprilTagAlignCommand(SwerveSubsystem swerveSubsystem, AprilTagSubsystem aprilTagSubsystem, int TagId) {
    this.TagId = TagId;
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
      xSpeed = 0;
      ySpeed = 0;
      rSpeed = 0;
      System.out.println("Tag not found, try again");
      return;
    }
    //translate position to 1 meter in front of tag
    this.TagPosition = this.TagPosition.plus(new Transform2d(2,0,new Rotation2d(0)));

    //gets the distance to the goal postion
    double relativex = this.TagPosition.getX();
    double relativey = this.TagPosition.getY();
    double distanceToTarget = Math.hypot(relativex,relativey); // a^2 + b^2 = c^2

    //divide the distance travelled by speed (0.5) to know how long it will take
    RunTime = distanceToTarget/0.5;

    //gets the absolute x, y movement to target and dived by time to know speed of each
    xSpeed = relativex/RunTime;
    ySpeed = relativey/RunTime;

    //gets the current angle of the tag
    double currentAngleRadians = this.TagPosition.getRotation().getRadians();
    //determines which way the bot will turn to face the tag
    double directionMultiplier = currentAngleRadians < 0
        ? 1
        : -1;
    //determines how much the bot needs to rotate
    double RotationNeeded = (Math.PI - Math.abs(currentAngleRadians)) * directionMultiplier ;
    
    // Calculate the rotational speed
    rSpeed = RotationNeeded/RunTime;

    //starts the stopwatch
    stopWatch.start(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //moves towards goal positon and rotation
    m_swerveSubsystem.driveRobotOriented(new ChassisSpeeds(xSpeed, ySpeed, rSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.driveRobotOriented(new ChassisSpeeds(0, 0, 0));
    System.out.println("Finished after:" + stopWatch.getDuration());
    System.out.println("Moved x:" + xSpeed*RunTime + " y:" + ySpeed*RunTime + " r:" + rSpeed*RunTime);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stopWatch.getDuration() > RunTime || this.TagPosition == null;
  }
}
