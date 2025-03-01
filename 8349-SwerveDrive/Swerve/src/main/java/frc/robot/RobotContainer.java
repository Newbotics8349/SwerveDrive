// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;

import frc.robot.subsystems.AprilTagSubsystem;

import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.CageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // * Initialize the robot subsystems
    private final SwerveSubsystem drivebase = new SwerveSubsystem();

    private final AprilTagSubsystem vision = new AprilTagSubsystem("cameramain");
    private final LEDSubsystem leds = new LEDSubsystem();

    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final ClawSubsystem claw = new ClawSubsystem();
    private final CageSubsystem cage = new CageSubsystem();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    CommandGenericHID buttons = new CommandGenericHID(1);
    CommandGenericHID buttons2 = new CommandGenericHID(2);

    // * Define objects for autonomous routine selection
    // private final SendableChooser<Command> autoSelector = AutoBuilder.buildAutoChooser();
    // Auto selection strings
    private static final String newmarketAuto = "Newmarket Auto";

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer() {

        // // * Set up autonomous routine selection
        // // Populate autonomous routine selection with available routines
        // autoSelector.setDefaultOption(newmarketAuto,
        // AutoBuilder.buildAuto(newmarketAuto));
        // // Make autonomous routine selector available on the smart dashboard
        // SmartDashboard.putData("Auto choices", autoSelector);

        // * Configure sticks to drive the robot in TeleOp
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                () -> m_driverController.getLeftY() * -1,
                () -> m_driverController.getLeftX() * -1)
                .withControllerRotationAxis(m_driverController::getRightX)
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(0.8)
                .allianceRelativeControl(true);

        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                .withControllerHeadingAxis(m_driverController::getRightX,
                        m_driverController::getRightY)
                .headingWhile(true);

        Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
        Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

        // * Configure the trigger bindings
        configureBindings();
        drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
        elevator.setDefaultCommand(elevator.setElevatorSpeed(m_driverController::getRightY));
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // * Send a debug message when any targets are seen
        Trigger targetsSeen = new Trigger(vision::hasTargets);
        targetsSeen.debounce(0.1);
        targetsSeen.onTrue(leds.debugMode(vision));
        // targetsSeen.onFalse(leds.setGlobalColour(0,0,0)); // LEDs off when no targets

        // Algae half levels
        buttons.button(5).onTrue(elevator.goToAlgae(1));
        buttons.button(6).onTrue(elevator.goToAlgae(2));

        // Elevator buttons
        buttons.button(1).onTrue(elevator.goToLevel(4));
        buttons.button(2).onTrue(elevator.goToLevel(3));
        buttons.button(3).onTrue(elevator.goToLevel(2));

        // Cage climb
        buttons.button(4).whileTrue(cage.raiseCage()).onFalse(cage.stopCage());

        // Claw stuff
        buttons.button(7).whileTrue(claw.wristLX()).onFalse(claw.stopWrist());
        buttons.button(8).whileTrue(claw.wristProcessor()).onFalse(claw.stopWrist());
        buttons.button(9).whileTrue(claw.wristAlgae()).onFalse(claw.stopWrist());
        buttons.button(10).whileTrue(claw.wristL4()).onFalse(claw.stopWrist());

        buttons.button(12).whileTrue(claw.clawElbowRotateUp()).onFalse(claw.clawElbowRotateStop());
        
        // Claw intake / outtake
        buttons2.button(1).whileTrue(claw.clawOut()).onFalse(claw.clawStop());
        buttons2.button(2).whileTrue(claw.clawIn()).onFalse(claw.clawStop());

        buttons2.button(3).whileTrue(elevator.reset());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
        // return autoSelector.getSelected();
        // // Get name of routine to run from the selector
        // String selectedAutoName = autoSelector.getSelected();

        // // Return the associated command from the Autos class
        // switch (selectedAutoName) {
        // case newmarketAuto:
        // return Autos.newmarketAuto(drivebase);
        // default:
        // return Autos.autoNotFound();
        // }
    }
}
