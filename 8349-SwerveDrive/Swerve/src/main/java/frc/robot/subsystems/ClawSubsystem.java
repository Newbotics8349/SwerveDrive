// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
  double kP, kI, kD;
  PIDController pid;
  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {
    kP = 0.005;
    kI = 0;
    kD = 0.0003; 
    pid = new PIDController(kP, kI, kD);
  }


  

  SparkMax wristMotor = new SparkMax(60, MotorType.kBrushless);
  DutyCycleEncoder wristEncoder = new DutyCycleEncoder(4, 360.0, 0);
  double wristValue;

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

  public Command wristFloor() {
    return run(() -> {
      double motorSpeed = pid.calculate(wristEncoder.get(), 80);
      wristMotor.set(motorSpeed);
    });
  }

  public Command wristReef() {
    return run(() -> {
      double motorSpeed = pid.calculate(wristEncoder.get(), 138);
      wristMotor.set(motorSpeed);
    });
  }

  public Command wristProcessor() {
    return run(() -> {
      double motorSpeed = pid.calculate(wristEncoder.get(), 120);
      wristMotor.set(motorSpeed);
    });
  }

  public Command stopWrist() {
    return run(() -> {
      wristMotor.set(0);
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
    SmartDashboard.putNumber("Claw Angle", wristEncoder.get());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
