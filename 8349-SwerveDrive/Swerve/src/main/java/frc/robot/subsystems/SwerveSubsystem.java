// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.io.File;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.wpilibj.DriverStation;



public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final AprilTagSubsystem cameraFunctions = new AprilTagSubsystem("cam1");
  File directory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive  swerveDrive;

  public SwerveSubsystem() {
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maxSpeed,
                                                                  new Pose2d(new Translation2d(Meter.of(1),
                                                                                               Meter.of(4)),
                                                                             Rotation2d.fromDegrees(0)));

      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerveDrive.updateOdometry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public void callingDrive(ChassisSpeeds chassisSpeeds, DriveFeedforwards driveFeedforwards) {
    swerveDrive.drive(chassisSpeeds);
  }
  public Command followPathCommand()
  {
    try {
      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
        cameraFunctions.getCameraToTagPose(cameraFunctions.getBestTarget(cameraFunctions.getTargets()))
      );
      PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);

      PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0.0, cameraFunctions.getCameraToTagPose(cameraFunctions.getBestTarget(cameraFunctions.getTargets())).getRotation()));

      path.preventFlipping = true;
      ModuleConfig mConfig = new ModuleConfig(0.048, 5.450, 1.2, DCMotor.getNEO(4), 5.143, 40, 4);
      Translation2d moduleOffsets[] = {
        new Translation2d(0.273, 0.273),
        new Translation2d(0.273, -0.273),
        new Translation2d(-0.273, 0.273),
        new Translation2d(-0.273, -0.273)
      };
      RobotConfig rConfig = new RobotConfig(74.088,6.883, mConfig, moduleOffsets);
      
      return new FollowPathCommand(
        path, 
        swerveDrive::getPose, 
        swerveDrive::getRobotVelocity, 
        this::callingDrive, 
        new PPHolonomicDriveController(
          new PIDConstants(5.0, 0.0, 0.0), 
          new PIDConstants(5.0, 0.0, 0.0)
        ), 
        rConfig, 
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, 
        this);
    } catch (Exception e) {
      System.out.println("Error");
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }
}

