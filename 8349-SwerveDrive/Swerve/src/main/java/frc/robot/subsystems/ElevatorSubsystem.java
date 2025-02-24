// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.GroupMotorControllers;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class ElevatorSubsystem extends SubsystemBase {
  // * Components controlling elevator height
  DigitalInput limitSwitch = new DigitalInput(2);
  Encoder elevatorEncoder = new Encoder(0, 1, false, EncodingType.k4X);
  SparkMax leftMotor = new SparkMax(51, MotorType.kBrushless);
  SparkMax rightMotor = new SparkMax(52, MotorType.kBrushless);

  // * Constants for moving elevator to required heights
  private final double ticksPerInch = (1.0 / 256.0);
  // Heights are relative to the end-effector at it's zero position
  private final double levelHeights[] = { 10, 15, 20, 25 };
  // PID values
  private final float kP = 1;
  private final float kI = 0;
  private final float kD = 0;
  PIDController pidController = new PIDController(kP, kI, kD);

  public ElevatorSubsystem() {
    // Set the conversion factor so meaningful distance values are available
    elevatorEncoder.setDistancePerPulse(ticksPerInch);
  }

  public Command goToLevel(int level) {
    // Check for valid args
    if (level < 1 || level > levelHeights.length)
      return run(() -> {
      });

    // Determine associated height needed to be travelled to
    double targetHeight = levelHeights[level];

    return run(
        () -> {
          double motorSpeed = pidController.calculate(elevatorEncoder.getDistance(), targetHeight);
          leftMotor.set(motorSpeed);
          rightMotor.set(motorSpeed);
        });
  }

  public Command stop() {
    return runOnce(
        () -> {
          leftMotor.set(0);
          rightMotor.set(0);
        });
  }

  // For driving with an axis for debug
  public Command setElevatorSpeed(DoubleSupplier speedFunc) {
    // Scale the speed from the controller axis
    // double mSpeed = new SlewRateLimiter(10).calculate(speed * 0.5);
    return run(
      () -> {
        double mSpeed = speedFunc.getAsDouble();
        leftMotor.set(mSpeed);
          rightMotor.set(mSpeed);
        });
  }

  public Command reset() {
    return run(
        () -> {
          if (!limitSwitch.get()) {
            leftMotor.set(-0.01);
            rightMotor.set(0.01);
          }
        });
  }

  // Limit switch condition
  public boolean atZero() {
    return limitSwitch.get();
  }

  @Override
  public void periodic() {
    // TODO: Uncomment when limit switch is installed 
    // Reset the encoder if the limit switch is triggered
    // if (atZero())
      // elevatorEncoder.reset();
      // leftMotor.set(0);
      // rightMotor.set(0);

    double encoderVal = elevatorEncoder.getDistance();
    // System.out.println(encoderVal);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
