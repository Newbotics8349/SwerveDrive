// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import org.dyn4j.geometry.Vector3;

import edu.wpi.first.math.geometry.Transform3d;

public class LEDSubsystem extends SubsystemBase {
  private AddressableLED leds;
  private int numLeds = 60;
  
  /** Creates a new ExampleSubsystem. */
  public LEDSubsystem() {
    leds = new AddressableLED(0);
    leds.setLength(numLeds);
    leds.start();
  }

  /**
   * Set the entire 60 LEDs to a single colour
   * @color: Vector3 where each component is a value between 0-1
   */ 
  public Command setColour(Vector3 color) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          // Create an array of color data for the LED strip
          AddressableLEDBuffer ledData = new AddressableLEDBuffer(numLeds);
        
          // Set all LEDs to a specific color, e.g., red (255, 0, 0)
          for (int i = 0; i < numLeds; i++) {
              ledData.setRGB(i, (int)color.x*255, (int)color.x*255, (int)color.x*255);
              // ledData.setRGB(i, (int)(255*(((float)i)/59f)), (int)(255*(1-(((float)i)/59f))), 0);
          }
        
          // Send the color data to the LEDs
          leds.setData(ledData);
        });
  }

  /**
   * 
   * @color: List of Vector3s where each vector's components are values between 0-1
   */ 
  public Command setColour(List<Vector3> colors) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          int ledsPerSection = numLeds / colors.size(); // int division, extra leds will just be off

          // Create an array of color data for the LED strip
          AddressableLEDBuffer ledData = new AddressableLEDBuffer(numLeds);
        
          for (Vector3 color : colors)
          {
            // Set all section's leds
            for (int i = 0; i < ledsPerSection; i++) {
                ledData.setRGB(i, (int)color.x*255, (int)color.x*255, (int)color.x*255);
            }

          }
        
          // Send the color data to the LEDs
          leds.setData(ledData);
        });
  }
  

  // Visualizes the first april tag target from the vision system
  public Command debugMode(AprilTagSubsystem visionSubsystem) {
    return runOnce(
        () -> {
          // Get first target's transform from vision subsystem
          Transform3d targetTransform = visionSubsystem.getTargets().get(0).getBestCameraToTarget();
          int r = (int) (targetTransform.getX() * 127 + 127); // Distance outward
          int g = (int) (targetTransform.getY() * 127 + 127); // Right
          int b = (int) (targetTransform.getZ() * 127 + 127); // Up

          // Create an array of color data for the LED strip
          AddressableLEDBuffer ledData = new AddressableLEDBuffer(numLeds);
        
          // Set all LEDs to a specific color, e.g., red (255, 0, 0)
          for (int i = 0; i < numLeds; i++) {
              ledData.setRGB(i, r, g, b);
              // ledData.setRGB(i, (int)(255*(((float)i)/59f)), (int)(255*(1-(((float)i)/59f))), 0);  // Red
          }
        
          // Send the color data to the LEDs
          leds.setData(ledData);
        });
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
