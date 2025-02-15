// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

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
   * Example command factory method.
   *
   * @return a command
   */
  public Command setGlobalColour(int r, int g, int b) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          // Create an array of color data for the LED strip
          AddressableLEDBuffer ledData = new AddressableLEDBuffer(numLeds);
        
          // Set all LEDs to a specific color, e.g., red (255, 0, 0)
          for (int i = 0; i < numLeds; i++) {
              ledData.setRGB(i, r, g, b);  // Red
              // ledData.setRGB(i, (int)(255*(((float)i)/59f)), (int)(255*(1-(((float)i)/59f))), 0);  // Red
          }
        
          // Send the color data to the LEDs
          leds.setData(ledData);
        });
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
