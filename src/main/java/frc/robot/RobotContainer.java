// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
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
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final TurretSubsystem turret = new TurretSubsystem();
    public final climbSubsystem climber = new climbSubsystem();
    public final LimelightSubsystem limelight = new LimelightSubsystem();

    // Import limelight vision helpers
    private final LimelightVision vision = new LimelightVision();
    public Field2d m_field = new Field2d();

    // Autos
    private final Autos autos;

    // Driver
    private final CommandXboxController driverController = new CommandXboxController(0);

    // Operator
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // auto chooser
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

        public RobotContainer() {
        // Initilize autos
        autos = new Autos(drivetrain, drive, turret, intake, climber, limelight);

        // auto chooser
        //autoChooser.setDefaultOption("Abbys auto", autos.abbysAuto());
        // autoChooser.addOption("right auto", autos.rightAuto());
        //autoChooser.addOption("middle auto", autos.middleAuto());
        //autoChooser.addOption("climb auto", autos.climbAuto());
        // autoChooser.addOption("DriveToTag12", autos.driveToTagTest());
        // autoChooser.addOption("LeftAuto", autos.LeftAuto());
        // autoChooser.addOption("NewmiddleAuto", autos.NewmiddleAuto());
        // autoChooser.addOption("Connor Middle Auto", autos.ConnorMiddleAuto());
        // autoChooser.addOption("---------", autos.middleAuto());
        // autoChooser.setDefaultOption("[RED] Right Auto", autos.RED_RightAuto());
        autoChooser.addOption("[RED] Middle Auto", autos.RED_MiddleAuto());
        autoChooser.addOption("[RED] Left Auto", autos.RED_LeftAuto());
        // autoChooser.addOption("[BLUE] Right Auto", autos.BLUE_RightAuto());
        autoChooser.addOption("[BLUE] Middle Auto", autos.BLUE_MiddleAuto());
        autoChooser.addOption("[BLUE] Left Auto", autos.BLUE_LeftAuto());
        autoChooser.addOption("[RED] right working auto",autos.RED_workingRight());
        autoChooser.addOption("[BLUE] right working auto",autos.BLUE_workingRight());

        SmartDashboard.putData(autoChooser);
        SmartDashboard.putData(m_field);

        configureBindings();
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed * 0.7) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed * 0.7) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        // operatorController.povLeft().onTrue(Commands.run(() -> turret.turnTurretLeft(0.3), turret));
        // operatorController.povRight().onTrue(Commands.run(() -> turret.turnTurretRight(0.3), turret));

        // turret.setDefaultCommand(
        //     new RunCommand(() -> {
                // int pov = operatorController.getPOV();
                // if (pov == 90) {
                //     turret.turnTurretLeft(0.3);
                // }
                // if (pov == 180){
                //     turret.turnTurretRight(0.3);
                // }
                // operatorController.povLeft().onTrue(Commands.run(() -> turret.turnTurretLeft(0.3), turret));
        //         }, turret
        //     )
        // );

    

        // shooter.setDefaultCommand(Commands.run(() -> shooter.shootFuel(runFeeder, runShooter), shooter));
    

        // driverController.pov(90)
        //     .whileTrue(
        //         new RunCommand(() -> turret.turnTurretRight(0.1), turret)
        //     )
        //     .whileFalse(
        //         new RunCommand(() -> turret.stopTurret())
        //     );


        // driverController.pov(270)
        //     .whileTrue(
        //         new RunCommand(() -> turret.turnTurretLeft(0.1), turret)
        //     )
        //     .whileFalse(
        //         new RunCommand(() -> turret.stopTurret())
        //     );



        // new POVButton(driverController, 180)
        //     .whileTrue(new RunCommand(
        //         Commands.runEnd(()->turret.turnTurretRight(0.05), ()->turret.stopTurret())
        //     ));

        //Right Trigger - Shoot Fuel
        // driverController.rightTrigger()
        //     .whileTrue(new ParallelCommandGroup(
        //         Commands.runEnd(()-> intake.raiseIntake(), () -> intake.stopIntake())
        //     ));

        // driverController.leftTrigger()
        //     .whileTrue(new ParallelCommandGroup(
        //         Commands.runEnd(()-> intake.lowerIntake(), () -> intake.stopIntake())
        //     ));

        // //Left Trigger - Intake Fuel
        // driverController.rightBumper()
        //     .whileTrue(new ParallelCommandGroup(
        //         Commands.runEnd(()-> turret.runTurretIntake(), () -> turret.stopTurretIntake()))
        //     );
        

        // driverController.x()
        //     .whileTrue(new ParallelCommandGroup(
        //         Commands.runEnd(()-> intake.reverseFrontIntake(), () -> intake.stopFrontIntake())
        //     ));

        // driverController.y()
        //     .whileTrue(new ParallelCommandGroup(
        //         Commands.runEnd(()-> intake.raiseIntake(), () -> intake.stopIntake())
        //     ));   

        // driverController.b()
        //     .whileTrue(new ParallelCommandGroup(
        //         Commands.runEnd(()-> intake.runFrontInake(), () -> intake.stopFrontIntake())
        //     ));

        // //B Button - Stop all shooter motors
        // driverController.a()
        //     .whileTrue(new ParallelCommandGroup(
        //         Commands.runEnd(()-> intake.lowerIntake(), () -> intake.stopIntake())
        //     ));

 

        // OPERATOR CONTROLLER BINDINGS

        // operatorController.a()
        //     .onTrue(new AimTurretAtHub(turret, limelight, "limelight"));

        // raise/lower front intake
        // drive


        

        // shoot
        // intake into turret
        // run front intake
        // turn turret
        

        

        // operatorController.a()
        //         .whileTrue(new StartEndCommand(
        //             () -> turret.turnTurretRight(0.05),
        //             () -> turret.stopTurret(),
        //             turret));

        // operatorController.x()
        //         .whileTrue(new StartEndCommand(
        //             () -> turret.turnTurretLeft(0.05),
        //             () -> turret.stopTurret(),
        //             turret));


/* working

        // operatorController.b()
        //     .whileTrue(new StartEndCommand(
        //             () -> intake.runFrontIntake(),
        //             () -> intake.stopFrontIntake(),
        //             turret));

        // operatorController.y()
        //     .whileTrue(new ParallelCommandGroup(
        //         Commands.runEnd(()-> turret.runTurret(), () -> turret.stopShooting()),
        //         Commands.runEnd(()-> turret.runTurretIntake(), () -> turret.stopTurretIntake())
        //     ));

        // operatorController.rightBumper()
        //     .whileTrue(new StartEndCommand(
        //         ()-> turret.reverseTurretIntake(), 
        //         ()-> turret.stopTurretIntake(),
        //         turret));

*/





        // operatorController.pov(90)
        //     .whileTrue(
        //         new RunCommand(() -> turret.turnTurretRight(0.1), turret)
        //     )
        //     .whileFalse(
        //         new RunCommand(() -> turret.stopTurret())
        //     );


        // operatorController.pov(270)
        //     .whileTrue(
        //         new RunCommand(() -> turret.turnTurretLeft(0.1), turret)
        //     )
        //     .whileFalse(
        //         new RunCommand(() -> turret.stopTurret())
        //     );
        


        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        //driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
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
        return autoChooser.getSelected();

    }
}
