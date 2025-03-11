// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawInOutSubsystem extends SubsystemBase {
  /** Creates a new ClawSubsystem. */
  public ClawInOutSubsystem() {
  }

  SparkMax motor41 = new SparkMax(41, MotorType.kBrushless);
  SparkMax motor42 = new SparkMax(42, MotorType.kBrushless);

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

  public Command clawIn() {
    return run(() -> {
      motor41.set(-1 * Constants.clawInSpeed);
      motor42.set(Constants.clawInSpeed);
    });
  }

  public Command clawOut() {
    return run(() -> {
      motor41.set(Constants.clawOutSpeed);
      motor42.set(-1 * Constants.clawOutSpeed);
    });
  }

  public Command clawStop() {
    return run(() -> {
      motor41.set(0);
      motor42.set(0);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
