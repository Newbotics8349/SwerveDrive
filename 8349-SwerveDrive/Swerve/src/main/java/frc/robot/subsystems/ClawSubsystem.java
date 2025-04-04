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
    kD = 0.0002; 
    pid = new PIDController(kP, kI, kD);
  }


  

  SparkMax wristMotor = new SparkMax(60, MotorType.kBrushless);
  DutyCycleEncoder wristEncoder = new DutyCycleEncoder(4, 360.0, 44);
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
      double motorSpeed = pid.calculate(wristEncoder.get(), 97);
      wristMotor.set(motorSpeed);
    });
  }

  public Command wristReef() {
    return run(() -> {
      double motorSpeed = pid.calculate(wristEncoder.get(), 143);
      wristMotor.set(motorSpeed);
    });
  }

  public Command wristProcessor() {
    return run(() -> {
      double motorSpeed = pid.calculate(wristEncoder.get(), 145);
      wristMotor.set(motorSpeed);
    });
  }

  public Command wristDefence() {
    return run(() -> {
      double motorSpeed = pid.calculate(wristEncoder.get(), 200);
      wristMotor.set(motorSpeed);
    });
  }

  public Command wristNet() {
    return run(() -> {
      double motorSpeed = pid.calculate(wristEncoder.get(), 125);
      wristMotor.set(motorSpeed);
    });
  }

  public void wristCoralOutAuto() {
    double motorSpeed = pid.calculate(wristEncoder.get(), 120);
    wristMotor.set(motorSpeed);
  }
  
  public void wristPrepAuto() {
    double motorSpeed = -0.1;
    wristMotor.set(motorSpeed);
  }

  public Command stopWrist() {
    return run(() -> {
      wristMotor.set(0);
    });
  }

  public void wristDefenceError() {
    double motorSpeed = pid.calculate(wristEncoder.get(), 200);
    wristMotor.set(motorSpeed);
  }

  public void wristNetAuto() {
    double motorSpeed = pid.calculate(wristEncoder.get(), 190);
    wristMotor.set(motorSpeed);
  }

  public void stopWristAuto() {
    wristMotor.set(0);
  }

  public boolean getPosition(double target) {
    return wristEncoder.get() < target + 2 && wristEncoder.get() > target - 2;
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
