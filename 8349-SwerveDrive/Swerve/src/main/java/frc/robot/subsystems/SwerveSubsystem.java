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
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import com.pathplanner.lib.auto.AutoBuilder;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.wpilibj.DriverStation;

public class SwerveSubsystem extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */

  File directory = new File(Filesystem.getDeployDirectory(), "swerve");
    SwerveDrive swerveDrive;

    // Objects for path planning commands
    PPHolonomicDriveController holoDriveController;
    RobotConfig robotConfig;
    private Field2d m_field = new Field2d();
    Trajectory displayTrajectory = new Trajectory();
    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  public SwerveSubsystem() {
    try
    {
      // gyro.reset();
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maxSpeed,
                                                                  new Pose2d(new Translation2d(Meter.of(1),
                                                                                               Meter.of(4)),
                                                                             Rotation2d.fromDegrees(0)));
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed\, angleConversionFactor, driveConversionFactor);
      robotConfig = RobotConfig.fromGUISettings();
      holoDriveController = new PPHolonomicDriveController(new PIDConstants(0,0,0), new PIDConstants(0,0,0));
      //gets the field from the swervedrive
      m_field = swerveDrive.field;
      SmartDashboard.putData("field", m_field);

    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    //configures the auto builder
    AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) -> driveRobotOriented(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                  new PIDConstants(0, 0.0, 0.0), // Translation PID constants
                  new PIDConstants(0, 0.0, 0.0) // Rotation PID constants
          ),
          robotConfig, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // Reference to this subsystem to set requirements
    );
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
     * An example method querying a boolean state of the subsystem (for example, a
     * digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    public float getHeading(){
      return gyro.getYaw();
    }

    public Pose2d getPose() {
      return swerveDrive.getPose();
    }
    
    public ChassisSpeeds getRobotVelocity() {
      return swerveDrive.getRobotVelocity();
    }
    public void setPose(Pose2d pose) {
      swerveDrive.swerveDrivePoseEstimator.resetPose(pose);
    } 

  @Override
  public void periodic() {
    displayFieldItems(getPose());
    // This method will be called once per scheduler run
    if (gyro.isConnected()) {
      SmartDashboard.putString("Gyro Status", "Connected");
      
      if (gyro.isCalibrating()) {
          SmartDashboard.putString("Gyro Calibration", "Calibrating");
      } else {
          SmartDashboard.putString("Gyro Calibration", "Calibrated");
      }
      
      SmartDashboard.putNumber("Gyro Heading", gyro.getAngle());  // Correct method for heading
    } else {
      SmartDashboard.putString("Gyro Status", "Not Connected");
    }

  }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

  
  /**setup for movement relative to the field
   * Left is positive, right is negative
   * Forward is positive, backward is negative
   * Counter clockwise is positive, clockwise is negative
   * 
    */
    public void driveFieldOriented(ChassisSpeeds velocity) {
      //System.out.println("Field:   X Speed: " + velocity.vxMetersPerSecond + " Y Speed: " + velocity.vyMetersPerSecond + " R Speed: " + velocity.omegaRadiansPerSecond);
        swerveDrive.driveFieldOriented(velocity);
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
          //System.out.println("Field:   X Speed: " + velocity.get().vxMetersPerSecond + " Y Speed: " + velocity.get().vyMetersPerSecond + " R Speed: " + velocity.get().omegaRadiansPerSecond);
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }

  /**setup for movement relative to the robot
   * Left is positive, right is negative
   * Forward is positive, backward is negative
   * Counter clockwise is positive, clockwise is negative
   * 
    */
    public void driveRobotOriented(ChassisSpeeds velocity) {
      //System.out.println("Robot :   X Speed: " + velocity.vxMetersPerSecond + " Y Speed: " + velocity.vyMetersPerSecond + " R Speed: " + velocity.omegaRadiansPerSecond);
      swerveDrive.drive(velocity);
  }

    public Command driveRobotOriented(Supplier<ChassisSpeeds> velocity) {
      return run(() -> {
          swerveDrive.drive(velocity.get());
      });
    }

    public Command robotForwards() {
      // double angle = Math.PI * gyro.getAngle() / 180;
      double v = 0.5;
      return driveFieldOriented(() -> new ChassisSpeeds(v, 0, 0));
    }
  
    public Command turnLeft() {
      // double angle = Math.PI * (gyro.getAngle() - 90) / 180;
      double v = 0.5;
      return driveFieldOriented(() -> new ChassisSpeeds(0, 0, v));
    }

    public Command turnRight() {
      // double angle = Math.PI * (gyro.getAngle() + 90) / 180;
      double v = 0.5;
      return driveFieldOriented(() -> new ChassisSpeeds(0, 0, -v));
    }

  public void callingDrive(ChassisSpeeds chassisSpeeds, DriveFeedforwards driveFeedforwards) {
    driveRobotOriented(chassisSpeeds);
  }

  public Command resetGyro() {
    return runOnce(() -> {
      gyro.reset();
    });
  }

  public Command resetTrajectory() {
    return runOnce(() -> {
      System.out.println("Resetting Trajectory");
      displayTrajectory = new Trajectory();
    });
  }

  public void displayFieldItems(Pose2d pose) {
      //m_field.setRobotPose(pose);
      m_field.getObject("trajectory").setTrajectory(displayTrajectory);
  }

  // public void visualizePath(PathPlannerPath path) {
  //       PathPlannerTrajectory trajectory = path.generateTrajectory(null, null, robotConfig);// Corrected
  //       Trajectory trajectory2 = path.
  //   }

  // public Command driveTo(Pose2d target) {
  //   SmartDashboard.putNumber("X Distance", target.getX());
  //   SmartDashboard.putNumber("Y Distance", target.getY());
  //   SmartDashboard.putNumber("Rotation", target.getRotation().getRadians());
  //   return new DriveForwards(this, target.getX(), target.getY(), target.getRotation().getRadians(), Math.sqrt(Math.pow(target.getX(), 2) 
  //                                                                                                   + Math.pow(target.getY(), 2)) / 4);
  // }
  
  
  /*
   * Follow a path using the PathPlanner library
   * @param x The relative x coordinate of the end point
   * @param y The relative y coordinate of the end point
   * @param rotation The relative rotation of the end point
   */
  public Command followPathCommand(double x, double y, Rotation2d rotation)
  {
    try {
      //rotates the coordinates so that the x and y translation are relative to the feild and not the robot heading
      Pose2d goalPose = new Pose2d(0, 0, getPose().getRotation())
        .plus(new Transform2d(x,y,new Rotation2d(0)));
      // what direction to the goal postion
      Rotation2d angleOfPath = new Rotation2d(-Math.atan2(goalPose.getX(), goalPose.getY())).plus(new Rotation2d(Math.PI/2));
      //changes robot initial velocity to the direction of the goal
      goalPose = new Pose2d(goalPose.getX(),goalPose.getY(),angleOfPath);
      //creates path waypoints
      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(0.0, 0.0, angleOfPath),
        goalPose
      );

      TrajectoryConfig config = new TrajectoryConfig(3.0, 3.0); // Max velocity and acceleration
      displayTrajectory = TrajectoryGenerator.generateTrajectory(
            List.of(
                    new Pose2d(getPose().getX(), getPose().getY(), angleOfPath), // Start pose
                    new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d(0))
                      .plus(new Transform2d(goalPose.getX(),goalPose.getY(),goalPose.getRotation())) // End pose
            ),config
            );
      PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
      PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0.0, rotation.plus(getPose().getRotation())));

            path.preventFlipping = true;

            return new FollowPathCommand(
                    path,
                    swerveDrive::getPose,
                    swerveDrive::getRobotVelocity,
                    this::callingDrive,
                    holoDriveController,
                    robotConfig,
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