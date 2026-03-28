// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.DriveToTag;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.climbSubsystem;



public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Import drivetrain
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Import Subsystems
    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final TurretSubsystem turret = new TurretSubsystem();
    public final climbSubsystem climber = new climbSubsystem();
    public final LimelightSubsystem limelight = new LimelightSubsystem();

    // Import limelight vision helpers
    private final LimelightVision vision = new LimelightVision();

    // Cameras



    private final Autos autos;

    // Driver
    private final CommandXboxController driverController = new CommandXboxController(0);

    // Operator
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();




        public RobotContainer() {
        // Initilize autos
        autos = new Autos(drivetrain, shooter, drive, turret, intake, climber, limelight);

        // auto chooser
        autoChooser.setDefaultOption("Abbys auto", autos.abbysAuto());

        autoChooser.addOption("right auto", autos.rightAuto());
        autoChooser.addOption("new middle auto", autos.middleAuto());
        autoChooser.addOption("drive and turn (complex)", autos.driveAndTurnComplex());
        autoChooser.addOption("climb auto", autos.climbAuto());
        autoChooser.addOption("DriveToTag12", autos.driveToTagTest());
        autoChooser.addOption("newAndImprovedLeftAuto", autos.newAndImprovedLeftAuto());


        SmartDashboard.putData(autoChooser);

        configureBindings();
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // shooter.setDefaultCommand(Commands.run(() -> shooter.shootFuel(runFeeder, runShooter), shooter));
    

        // Right Trigger - Shoot Fuel
        //operatorController.rightTrigger()
        //    .whileTrue(new RunCommand(() -> shooter.shootFuel(), shooter));

        // operatorController.rightTrigger()
        //     .whileTrue(new ParallelCommandGroup(
        //         Commands.runEnd(()-> runFeeder = true, () -> runFeeder = false),
        //         Commands.runEnd(() -> runShooter = true, () -> runShooter = false),
        //         Commands.runEnd(()-> shooter.frontIntakeSpeed(), () -> shooter.stopShooter()),
        //         Commands.runEnd(() -> shooter.backIntakeSpeed(), () -> shooter.stopShooter())
        //     ));


        // Left Trigger - Intake Fuel
        // operatorController.leftTrigger()
        //     .whileTrue(new RunCommand(() -> shooter.intakeFuel(), 
        //     shooter));
        

        driverController.x()
        .onTrue(new RunCommand(()-> intake.runIntake()));

        driverController.y()
        .onTrue(new RunCommand(()-> intake.reverseIntake()));
        
        driverController.b()
        .onTrue(new RunCommand(()-> intake.stopIntake()));

        // B Button - Stop all shooter motors
        // operatorController.b()
        //     .onTrue(new InstantCommand(() -> shooter.stopAllMotors()));

        // operatorController.x()
        //     .whileTrue(
        //         Commands.runEnd(() -> shootVel = 0.0, () -> shootVel = 0.5)
        //     );
 
        
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //driverController.y().whileTrue(drivetrain.applyRequest(() ->
        //    point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        //));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        //driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
        
    }
    /* 
    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.


            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.8)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(4.0),

            drivetrain.applyRequest(() -> idle).withTimeout(0.25),

            Commands.runOnce(() -> shooter.setIntakeSpeed(0.5)),

            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.4)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(3.0),
            Commands.runOnce(()-> shooter.stopIntake()),

            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
    */ 

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();  // rightAuto | leftAuto | middleAuto

    }
}
