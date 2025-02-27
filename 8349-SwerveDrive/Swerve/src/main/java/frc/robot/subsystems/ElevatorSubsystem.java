// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.GroupMotorControllers;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;

import com.ctre.phoenix.time.StopWatch;

public class ElevatorSubsystem extends SubsystemBase {
  // * Components controlling elevator height
  DigitalInput limitSwitch = new DigitalInput(2);
  Encoder elevatorEncoder = new Encoder(0, 1, false, EncodingType.k4X);
  SparkMax leftMotor = new SparkMax(51, MotorType.kBrushless);
  SparkMax rightMotor = new SparkMax(52, MotorType.kBrushless);

  StopWatch time = new StopWatch();

  // * Constants for moving elevator to required heights
  private final double ticksPerInch = (1.0 / 256.0);
  // Heights are relative to the end-effector at it's zero position
  private final double levelHeights[] = { 0, 10, 15, 20, 25 };
  // PID values
  private final float kP = 1;
  private final float kI = 0;
  private final float kD = 0;
  PIDController pidController = new PIDController(kP, kI, kD);
  CommandGenericHID buttons = new CommandGenericHID(0);
  boolean cancelElevator = false;

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
    return runOnce(
        () -> {
          boolean up = true;
          if (elevatorEncoder.getDistance() < targetHeight) {
            up = false;
          }
          double curDistance = elevatorEncoder.getDistance();
          time.start();

          double motorSpeed = pidController.calculate(elevatorEncoder.getDistance(), targetHeight);
          leftMotor.set(-motorSpeed);
          rightMotor.set(-motorSpeed);
          while ((up && elevatorEncoder.getDistance() > targetHeight || !up && elevatorEncoder.getDistance() < targetHeight) && (time.getDuration() < 1 || Math.floor(curDistance * 100) / 100 != Math.floor(elevatorEncoder.getDistance() * 100) / 100)) {
            System.out.println(curDistance);
            System.out.println(elevatorEncoder.getDistance());
            motorSpeed = pidController.calculate(elevatorEncoder.getDistance(), targetHeight);
            leftMotor.set(-motorSpeed);
            rightMotor.set(-motorSpeed);
            if (time.getDuration() > 1.5) {
              curDistance = elevatorEncoder.getDistance();
              time.start();
            }
          }
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
          System.out.println(limitSwitch.get());
          if (!limitSwitch.get()) {
            leftMotor.set(0.1);
            rightMotor.set(-0.1);
          }
        });
  }

  // Limit switch condition
  public boolean atZero() {
    return limitSwitch.get();
  }

  @Override
  public void periodic() {
    // Reset the encoder if the limit switch is triggered
    if (atZero())
      elevatorEncoder.reset();
      leftMotor.set(0);
      rightMotor.set(0);

    double encoderVal = elevatorEncoder.getDistance();
    // System.out.println(encoderVal);
    // cancelElevator = buttons.button(10).getAsBoolean();
    // System.out.println(cancelElevator);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
