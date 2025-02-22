// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static Command newmarketAuto(SwerveSubsystem subsystem) {
    Command followPath = new PathPlannerAuto("newmarket Auto.auto");
    return Commands.sequence(new PrintCommand("Called the MOVE auto routine"), followPath);
  }

  public static Command autoNotFound() {
    return new PrintCommand("THE SELECTED AUTO WAS NOT FOUND");
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
