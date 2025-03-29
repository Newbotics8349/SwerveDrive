// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ElevatorUpCommand;
import frc.robot.commands.ResetCommand;
import frc.robot.commands.TimeCommand;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.commands.CoralClawCommand;
import frc.robot.commands.CoralInCommand;
import frc.robot.commands.AprilTagAlignCommandPathPlanner;
import frc.robot.commands.RelativeMovementCommand;
import frc.robot.commands.ClawDefence;
import frc.robot.commands.ClawPrepCommand;
import frc.robot.commands.CoralOutCommand;
import frc.robot.commands.DriveForwards;
import frc.robot.commands.ElevatorStayCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.CageSubsystem;
import frc.robot.subsystems.ClawInOutSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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

    private final AprilTagSubsystem vision = new AprilTagSubsystem("USB_Camera");
    private final LEDSubsystem leds = new LEDSubsystem();

    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final ClawSubsystem claw = new ClawSubsystem();
    private final ClawInOutSubsystem intakeOuttake = new ClawInOutSubsystem();
    private final CageSubsystem cage = new CageSubsystem();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    // the button panel stuff
    CommandGenericHID buttons = new CommandGenericHID(1);

    // * Define objects for autonomous routine selection
    private final SendableChooser<Command> autoSelector =
    AutoBuilder.buildAutoChooser("newmarketSimpleAuto");

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer() {
        // Make autonomous routine selector available on the smart diashboard
        autoSelector.addOption("simpleAuto", getBasicAutonomousCommand());
        SmartDashboard.putData("Auto choices", autoSelector);

        SmartDashboard.putData(elevator);

        // * Configure sticks to drive the robot in TeleOp
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                () -> m_driverController.getLeftY() * -1,
                () -> m_driverController.getLeftX() * -1)
                .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(0.8)
                .allianceRelativeControl(true);
                
        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                .withControllerHeadingAxis(() -> preventReturnHeadingX(
                    m_driverController.getRightX() * -1, m_driverController.getRightY(), 0.2),
                () -> preventReturnHeadingY(
                    m_driverController.getRightY() * -1, m_driverController.getRightX(), 0.2))
                .headingWhile(true);

        Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
        //Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

        //configure dPad movement
        m_driverController.povDown().whileTrue(new RelativeMovementCommand(drivebase, -0.5,0));
        m_driverController.povUp().whileTrue(new RelativeMovementCommand(drivebase, 0.5,0));
        m_driverController.povLeft().whileTrue(new RelativeMovementCommand(drivebase, 0,0.5));
        m_driverController.povRight().whileTrue(new RelativeMovementCommand(drivebase, 0,-0.5));
        
        // * Configure the button bindings
        configureBindings();
    }

    // * Use this method to define deadbands for the sticks and make sure the heading wont reset after pathplanning
    private double preventReturnHeadingX(double value, double value2, double deadband) {
        return Math.hypot(value2, value) > deadband ? value : drivebase.getPose().getRotation().getSin();
    }

    private double preventReturnHeadingY(double value, double value2, double deadband) {
        return Math.hypot(value2, value) > deadband ? value : drivebase.getPose().getRotation().getCos();
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
        //targetsSeen.onFalse(leds.setGlobalColour(0,0,0)); // LEDs off when no targets

        // Algae half levels

        buttons.button(12).whileTrue(elevator.reset()).onFalse(elevator.stop());
        buttons.button(11).whileTrue(elevator.goToLevel(1)).onFalse(elevator.stop());
        buttons.button(11).whileTrue(claw.wristReef()).onFalse(claw.stopWrist());
        buttons.button(10).whileTrue(elevator.goToLevel(2)).onFalse(elevator.stop());
        buttons.button(10).whileTrue(claw.wristReef()).onFalse(claw.stopWrist());
        buttons.button(9).whileTrue(intakeOuttake.clawIn()).onFalse(intakeOuttake.clawStop());
        buttons.button(8).whileTrue(elevator.goToLevel(0)).onFalse(elevator.stop());
        buttons.button(8).whileTrue(claw.wristFloor()).onFalse(claw.stopWrist());
        buttons.button(7).whileTrue(intakeOuttake.clawOut()).onFalse(intakeOuttake.clawStop());
        buttons.button(6).whileTrue(cage.raiseCage()).onFalse(cage.stopCage());
        buttons.button(5).whileTrue(claw.wristDefence()).onFalse(claw.stopWrist());
        buttons.button(4).whileTrue(elevator.goToLevel(0)).onFalse(elevator.stop());
        buttons.button(4).whileTrue(claw.wristProcessor()).onFalse(claw.stopWrist());

        m_driverController.leftBumper().whileTrue(drivebase.turnLeft());
        m_driverController.rightBumper().whileTrue(drivebase.turnRight());

        //m_driverController.leftTrigger().whileTrue(new AdjustTowardsAprilTagCommand(drivebase,vision,15)); still iffy

        m_driverController.y().whileTrue(new AprilTagAlignCommandPathPlanner(drivebase,vision,15,
            (()->!m_driverController.y().getAsBoolean())));

        m_driverController.a().whileTrue(drivebase.resetGyro());

        m_driverController.x().whileTrue(drivebase.robotForwards());

        m_driverController.a().whileTrue(new SequentialCommandGroup(new TimeCommand(), new ResetCommand(elevator), new ClawDefence(claw)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    private Command getBasicAutonomousCommand() {
        return new SequentialCommandGroup(
           // new TurnCommand(drivebase),
            new DriveForwards(drivebase, -1),
            new ClawPrepCommand(claw),
            new ElevatorUpCommand(elevator, 5),
            new ParallelCommandGroup(
                new ElevatorStayCommand(elevator),
                new CoralClawCommand(claw),
                new CoralOutCommand(intakeOuttake)
            ),
            new DriveForwards(drivebase, -0.1),
            new ParallelCommandGroup(
                new ElevatorUpCommand(elevator, 20),
                new ClawPrepCommand(claw)
            ),
            new ParallelCommandGroup(
                new DriveForwards(drivebase, 0.1),
                new CoralInCommand(intakeOuttake),
                new ClawPrepCommand(claw)                
            )
        );
    }

    public Command getAutonomousCommand() {
        Command auto = autoSelector.getSelected();
        return auto;

    }
}
