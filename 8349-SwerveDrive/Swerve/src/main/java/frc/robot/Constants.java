// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = 0.04;
  }

  public static final double maxSpeed = Units.feetToMeters(9);

  // cameraOnRobot describes the camera's location relative to the robot center
  // with forward = +Y
  public static final Transform2d cameraOnRobot = new Transform2d(
      Units.inchesToMeters(0.5),
      Units.inchesToMeters(14),
      new Rotation2d(0));

  // Speeds for claw intake/expell
  public static final double clawInSpeed = 0.9;
  public static final double clawOutSpeed = 0.9;
}
