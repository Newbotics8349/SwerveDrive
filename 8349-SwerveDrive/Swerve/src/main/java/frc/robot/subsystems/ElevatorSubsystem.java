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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import java.util.Map;

import edu.wpi.first.wpilibj.SPI;

import com.ctre.phoenix.time.StopWatch;

public class ElevatorSubsystem extends SubsystemBase {
  // * Components controlling elevator height
  DigitalInput lSwitch = new DigitalInput(2);
  Encoder elevatorEncoder;
  SparkMax leftMotor = new SparkMax(51, MotorType.kBrushless);
  SparkMax rightMotor = new SparkMax(52, MotorType.kBrushless);

  StopWatch time = new StopWatch();

  // * Constants for moving elevator to required heights
  private final double ticksPerInch = (1.0 / 256.0);
  // Heights are relative to the end-effector at it's zero position
  private final double levelHeights[] = { 0, 6, 10, 15, 22 };
  private final double algaeHeights[] = { 0, 12, 17 };
  private PIDController pid;
  CommandGenericHID buttons = new CommandGenericHID(0);
  boolean cancelElevator = false;
  private double kP, kI, kD;

  public ElevatorSubsystem() {
    // Set the conversion factor so meaningful distance values are available
    kP = 0;
    kI = 0;
    kD = 0;

    elevatorEncoder = new Encoder(0, 1, false, EncodingType.k4X);
    elevatorEncoder.setDistancePerPulse(ticksPerInch);

    pid = new PIDController(kP, kI, kD);

    Shuffleboard.getTab("PID Tuning")
        .add("P Value", kP)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1.0, "max", 1.0)) // specify widget properties here
        .getEntry();

    Shuffleboard.getTab("PID Tuning")
        .add("I Value", kI)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1.0, "max", 1.0)) // specify widget properties here
        .getEntry();

    Shuffleboard.getTab("PID Tuning")
        .add("D Value", kD)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1.0, "max", 1.0)) // specify widget properties here
        .getEntry();
  }

  public double getElevatorDistance() {
    return elevatorEncoder.getDistance();
  }

  public Command goToLevel(int level) {
    // Check for valid args
    if (level < 1 || level >= levelHeights.length)
      return run(() -> {
      });

    // Determine associated height needed to be travelled to
    double targetHeight = levelHeights[level];
    return run(
        () -> {
          // if (elevatorEncoder.getDistance() < targetHeight) {
          //   leftMotor.set(-0.75);
          //   rightMotor.set(0.75);
          // } else {
          //   leftMotor.set(0.75);
          //   rightMotor.set(-0.75);
          // }
          double motorSpeed = pid.calculate(elevatorEncoder.getDistance(), targetHeight);
          leftMotor.set(motorSpeed);
          rightMotor.set(motorSpeed);

        });
  }

  public Command goToAlgae(int level) {
    // Check for valid args
    if (level < 1 || level >= levelHeights.length)
      return run(() -> {
      });

    // Determine associated height needed to be travelled to
    double targetHeight = levelHeights[level];
    return run(
        () -> {
          System.out.println(kP);
          System.out.println(kI);
          System.out.println(kD);
          // if (elevatorEncoder.getDistance() < targetHeight) {
          //   leftMotor.set(-0.75);
          //   rightMotor.set(0.75);
          // } else {
          //   leftMotor.set(0.75);
          //   rightMotor.set(-0.75);
          // }
          double motorSpeed = pid.calculate(elevatorEncoder.getDistance(), targetHeight);
          leftMotor.set(motorSpeed);
          rightMotor.set(-motorSpeed);

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
          System.out.println(lSwitch.get());
          if (!lSwitch.get()) 
            leftMotor.set(0.15);
            rightMotor.set(-0.15);
          }
        );
  }

  public Command lswitch() {
    return runOnce(
      () -> {
        System.out.println(atZero());
      }
    );
  }

  // Limit switch condition
  public boolean atZero() {
    return lSwitch.get();
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
