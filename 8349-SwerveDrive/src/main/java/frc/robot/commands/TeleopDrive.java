// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleopDrive extends Command {
  /** Creates a new drive. */
  SwerveSubsystem DriveTrain;
  DoubleSupplier x;
  DoubleSupplier y;
  DoubleSupplier r;

  public TeleopDrive(SwerveSubsystem DriveTrain, DoubleSupplier x, DoubleSupplier y, DoubleSupplier r) {
        this.DriveTrain = DriveTrain;
        this.x = x;
        this.y = y;
        this.r = r;
        
    
    addRequirements(DriveTrain);

    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveTrain.TeleopDrive(x.getAsDouble(), y.getAsDouble(), r.getAsDouble());
    System.out.println(String.valueOf(x.getAsDouble()) + " - X");
    System.out.println(String.valueOf(y.getAsDouble()) + " - Y");
    System.out.println(String.valueOf(r.getAsDouble()) + " - R");

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
