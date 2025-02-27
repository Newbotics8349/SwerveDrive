// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {}
  SparkMax orangeMotor = new SparkMax(41, MotorType.kBrushless);
  SparkMax greenMotor = new SparkMax(42, MotorType.kBrushless);
  SparkMax elbowMotor = new SparkMax(61, MotorType.kBrushless);
  SparkMax wristMotor = new SparkMax(60, MotorType.kBrushless);
  DutyCycleEncoder wristEncoder = new DutyCycleEncoder(4, 360.0, 17.568009439200235);
  DutyCycleEncoder elbowEncoder = new DutyCycleEncoder(3, 360.0, 11.241936281048407);
  double elbowValue;
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

  public Command clawIn() {
    return run(() -> {
      orangeMotor.set(-0.5);
      greenMotor.set(0.5);
    });
  }

  public Command clawOut() {
    return run(() -> {
      orangeMotor.set(0.5);
      greenMotor.set(-0.5);
    });
  }

  public Command clawStop() {
    return run(() -> {
      orangeMotor.set(0);
      greenMotor.set(0);
    });
  }

  public Command clawElbowRotateUp(){
    return run(() -> {
      if(elbowValue > 25 && elbowEncoder.get() > 25){
        elbowMotor.set(-0.2);
        System.out.println("1");
      }else if (elbowValue < 25 && elbowEncoder.get() < 25){
        elbowMotor.set(0.2);
        System.out.println("2");
      } else {
        elbowMotor.set(0);
      }
    });
  }

  public Command clawElbowRotateDown(){
    return run(() -> {
      if (elbowEncoder.get() < 60) {
        elbowMotor.set(0.2);
      }
    });
  }

  public Command getEncoder() {
    return runOnce(
      () -> {
        elbowValue = elbowEncoder.get();
        wristValue = wristEncoder.get();
        System.out.println(elbowValue);
        System.out.println(wristValue);
      }
    );
  }

  public Command clawElbowRotateStop(){
    return run(() -> {
      elbowMotor.set(0);
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
