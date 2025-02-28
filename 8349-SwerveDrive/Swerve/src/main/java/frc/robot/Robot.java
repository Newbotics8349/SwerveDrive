// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    @Override
    public void robotInit() {
        System.out.println("Robot initialization complete");
        
        // Configure autonomous chooser
        m_autoChooser.setDefaultOption("Simple Auto", 
            new PrintCommand("Running Simple Autonomous"));
        m_autoChooser.addOption("Complex Auto", 
            new PrintCommand("Executing Complex Autonomous Routine"));
        
        SmartDashboard.putData("Autonomous Mode", m_autoChooser);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        System.out.println("Autonomous mode initialized");
        
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        
        m_autonomousCommand = m_autoChooser.getSelected();
        
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousExit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopInit() {
        System.out.println("Teleoperated mode started");
        
        if (m_autonomousCommand != null && m_autonomousCommand.isScheduled()) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void testInit() {
        System.out.println("Test mode activated");
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void simulationPeriodic() {
        // Simulation-specific logic (if needed)
    }
}