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

/** A command that uses an April Tag Vision to adjust towards given tag while called. */
public class AdjustTowardsAprilTagCommand extends Command {
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
   * @param subsystem The subsystem used by this command.
   */
  public AdjustTowardsAprilTagCommand(SwerveSubsystem swerveSubsystem, AprilTagSubsystem aprilTagSubsystem, int TagId) {
    this.TagId = TagId;
    m_swerveSubsystem = swerveSubsystem;
    m_aprilTagSubsystem = aprilTagSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem, aprilTagSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
public void initialize() {
    System.out.println("Adjusting to Tag");
    stopWatch.start(); // Start the stopwatch
}

@Override
public void execute() {
    // Get the tag position dynamically
    //get the 2dpose of the tag which is relative to the bot
    this.TagPosition = m_aprilTagSubsystem.getCameraToTagPose(this.TagId);
    if (this.TagPosition == null) {
      System.out.println("Tag not found, unable to adjust");
    } else {
      //translate position to 1 meter in front of tag
      this.TagPosition = this.TagPosition.plus(new Transform2d(1,0,new Rotation2d(0)));

      //gets the distance to the goal postion
      double relativex = this.TagPosition.getX();
      double relativey = this.TagPosition.getY();
      double distanceToTarget = Math.hypot(relativex,relativey); // a^2 + b^2 = c^2
      

      //divide the distance travelled by speed (0.5) to know how long it will take
      RunTime = distanceToTarget/0.5;

      //gets the absolute x, y movement to target and dived by time to know speed of each
      xSpeed = relativex/RunTime;//negative since y is to the left
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
    }
    if (stopWatch.getDuration() > RunTime) {
      return;
    }
    // Move the robot towards the goal
    m_swerveSubsystem.driveRobotOriented(new ChassisSpeeds(xSpeed, ySpeed, rSpeed));
    stopWatch.start();
}

@Override
public void end(boolean interrupted) {
    // Stop the robot when the command ends
    m_swerveSubsystem.driveRobotOriented(new ChassisSpeeds(0, 0, 0));
}

@Override
public boolean isFinished() {
    // The command will not end on its own; it will run as long as the button is held
    return false;
}
}
