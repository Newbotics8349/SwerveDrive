// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  DutyCycleEncoder elevatorEncoder = new DutyCycleEncoder(0);
  SparkMax leftMotor = new SparkMax(31, MotorType.kBrushless);
  SparkMax rightMotor = new SparkMax(32, MotorType.kBrushless);
  public ElevatorSubsystem() {
    
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command level1() {
    return run(
        () -> {
          while (elevatorEncoder.get() > 1) {
            leftMotor.set(-0.01);
            rightMotor.set(0.01);
          }
        }
    );
  }
  public Command stop() {
    return runOnce(
        () -> {
          leftMotor.set(0);
          rightMotor.set(0);
        }
    );
  }
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
