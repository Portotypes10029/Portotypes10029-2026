package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimTurretAtTag;
import frc.robot.commands.DriveToTag;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.climbSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Optional;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

public class Autos {

    public static final Command RED_workingRight = null;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric drive;
    private final climbSubsystem climb;
    private final IntakeSubsystem intake;
    private final LimelightSubsystem limelight;
    private final TurretSubsystem turret;


    
    
    
    public Autos(CommandSwerveDrivetrain drivetrain, RobotCentric drive2, TurretSubsystem turret, IntakeSubsystem intake, climbSubsystem climb, LimelightSubsystem limelight) {
        this.drivetrain = drivetrain;
        this.drive = drive2;
        this.intake = intake;
        this.climb = climb;
        this.turret = turret;
        this.limelight = limelight;

    }
    
    private final SwerveRequest.FieldCentricFacingAngle turnToAngle = 
        new SwerveRequest.FieldCentricFacingAngle()
            .withHeadingPID(10, 0, 0); 

    
    
    public Command trackTag() {
        return new SequentialCommandGroup(

            // Aim at tag 12 
            new AimTurretAtTag(turret, limelight, "limelight", 12),

            Commands.waitSeconds(1)
        );
    }

    public Command driveToTagTest() {
        final var idle = new SwerveRequest.Idle();

        return new SequentialCommandGroup(

            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),

            // Drive toward tag 12 
            new DriveToTag(drivetrain, limelight, drive, "limelight", 12),

            drivetrain.applyRequest(() -> idle).withTimeout(0.5)
        );
    }




    public Command LeftAuto() {
        @SuppressWarnings("unused")
        final var idle = new SwerveRequest.Idle();

        return new SequentialCommandGroup(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(-2)
                .withVelocityY(0)
                .withRotationalRate(0)
            ).withTimeout(1),

            drivetrain.applyRequest(() -> drive
                .withVelocityX(2)
                .withVelocityY(0)
                .withRotationalRate(0)
            ).withTimeout(3.60),

            drivetrain.applyRequest(() -> turnToAngle
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(Rotation2d.fromDegrees(-90))
            ).withTimeout(1.5),
         
            drivetrain.applyRequest(() -> drive
                .withVelocityX(2)
                .withVelocityY(0)
                .withRotationalRate(0)
            ).withTimeout(2),

             drivetrain.applyRequest(() -> turnToAngle
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(Rotation2d.fromDegrees(-90))
            ).withTimeout(1.5),

            drivetrain.applyRequest(() -> drive
                .withVelocityX(2)
                .withVelocityY(0)
                .withRotationalRate(0)
            ).withTimeout(2),

             drivetrain.applyRequest(() -> turnToAngle
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(Rotation2d.fromDegrees(0))
            ).withTimeout(1.5),

             drivetrain.applyRequest(() -> drive
                .withVelocityX(2)
                .withVelocityY(0)
                .withRotationalRate(0)
            ).withTimeout(3),

              drivetrain.applyRequest(() -> turnToAngle
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(Rotation2d.fromDegrees(-90))
            ).withTimeout(1.5)
        );
    }




    public Command middleAuto() {
        final var idle = new SwerveRequest.Idle();

        return new SequentialCommandGroup(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),

            drivetrain.applyRequest(() -> drive
                .withVelocityX(-2)
                .withVelocityY(0)
                .withRotationalRate(0)
            ).withTimeout(1),

            Commands.runOnce(() -> turret.runTurret()),

            drivetrain.applyRequest(() -> idle).withTimeout(3),

            drivetrain.applyRequest(() -> drive
                .withVelocityX(0)
                .withVelocityY(2)
                .withRotationalRate(0)
            ).withTimeout(2),


            Commands.runOnce(() -> intake.runFrontIntake()),

            drivetrain.applyRequest(() -> drive
                .withVelocityX(-2)
                .withVelocityY(0)
                .withRotationalRate(0)
            ).withTimeout(1),


        

            Commands.runOnce(() -> turret.runTurret())

        );
    }
    
    public Command NewmiddleAuto() {
        return new SequentialCommandGroup(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                drivetrain.applyRequest(()-> drive
                    .withVelocityX(-2)
                    .withVelocityY(0)
                    .withRotationalRate(0)
                ).withTimeout(1),

                drivetrain.applyRequest(() -> drive
                    .withVelocityX(0)
                    .withVelocityY(2)
                    .withRotationalRate(0)  
                ).withTimeout(1.20),
                
                drivetrain.applyRequest(() -> drive
                    .withVelocityX(-2)
                    .withVelocityY(0)
                    .withRotationalRate(0)
                ).withTimeout(1.25)

            
        );
    }
        
    public Command ConnorMiddleAuto() {
        final var idle = new SwerveRequest.Idle();

        return new SequentialCommandGroup(
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),

                drivetrain.applyRequest(() -> drive
                    .withVelocityX(-2)
                    .withVelocityY(0)
                    .withRotationalRate(0)
                ).withTimeout(.5),
                
                drivetrain.applyRequest(()-> idle).withTimeout(.1),

                Commands.runOnce(() -> turret.runTurret()).withTimeout(8),

                drivetrain.applyRequest(() -> drive
                    .withVelocityX(-2)
                    .withVelocityY(0)
                    .withRotationalRate(0)
                ).withTimeout(1),
                
                drivetrain.applyRequest(() -> drive
                    .withVelocityX(-2)
                    .withVelocityY(0)
                    .withRotationalRate(0)
                ).withTimeout(1)


        );
    }

    public Command rightAuto() {
        final var idle = new SwerveRequest.Idle();

        return new SequentialCommandGroup(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),

            drivetrain.applyRequest(() -> drive
                .withVelocityX(-2)
                .withVelocityY(0)
                .withRotationalRate(0)
            ).withTimeout(1.25),

            drivetrain.applyRequest(() -> idle).withTimeout(2),

            Commands.runOnce(() -> turret.runTurret()),


            drivetrain.applyRequest(() -> drive
                .withVelocityX(-2)
                .withVelocityY(0)
                .withRotationalRate(0)
            ).withTimeout(1.25),

            drivetrain.applyRequest(() -> idle).withTimeout(2),

            drivetrain.applyRequest(() -> drive
                .withVelocityX(2)
                .withVelocityY(0)
                .withRotationalRate(0)
            ).withTimeout(1.25)
            


        );
    }        

    public Command climbAuto() {

        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        double direction =
            (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) ? -1.0 : -1.0;


        return new SequentialCommandGroup(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                drivetrain.applyRequest(() -> drive
                .withVelocityX(-1 * direction)
                .withVelocityY(0)
                .withRotationalRate(0)
            ).withTimeout(1.75),
            Commands.runOnce(()-> climb.runClimberMotor(1))

        );
    }


    public Command abbysAuto(){
        final var idle = new SwerveRequest.Idle();

        return new SequentialCommandGroup(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),


            drivetrain.applyRequest(()->drive
            .withVelocityX(1)
            .withVelocityY(0)
            .withRotationalRate(0)
            ).withTimeout(5),

            drivetrain.applyRequest(() -> turnToAngle
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(Rotation2d.fromDegrees(90))
            ).withTimeout(1.5),
            
            drivetrain.applyRequest(()-> drive
            .withVelocityX(1)
            .withVelocityY(0)
            .withRotationalRate(0)
            ).withTimeout(7),
          
            drivetrain.applyRequest(() -> turnToAngle
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(Rotation2d.fromDegrees(-180))
            ).withTimeout(1.5),

            drivetrain.applyRequest(()-> drive
            .withVelocityX(1)
            .withVelocityY(0)
            .withRotationalRate(0)   
            ).withTimeout(7),
             
            drivetrain.applyRequest(() -> turnToAngle
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(Rotation2d.fromDegrees(-90))
            ).withTimeout(1),
 
            drivetrain.applyRequest(() -> drive
            .withVelocityX(1)
            .withVelocityY(0)
            .withRotationalRate(0)
            ).withTimeout(1)
            
            // drivetrain.applyRequest(() -> turnToAngle
            //     .withVelocityX(0)
            //     .withVelocityY(0)
            //     .withTargetDirection(Rotation2d.fromDegrees(90))
            // ),

            // drivetrain.applyRequest(() -> drive
            // .withVelocityX(1)
            // .withVelocityX(0)
            // .withRotationalRate(0)
            // ).withTimeout(1)


        );

    }





    // Alliance Specific Auto Routines

    
    // public Command RED_RightAuto() {
    //     final var idle = new SwerveRequest.Idle();

    //     return new SequentialCommandGroup(

    //         drivetrain.applyRequest(() -> drive
    //             .withVelocityX(-3)
    //             .withVelocityY(0)
    //             .withRotationalRate(0)
    //         ).withTimeout(0.2),
            
    //         drivetrain.applyRequest(() -> idle).withTimeout(2),
    //         Commands.runOnce(() -> turret.runTurret()).withTimeout(3),

    //         drivetrain.applyRequest(() -> turnToAngle
    //             .withVelocityX(0)
    //             .withVelocityY(0)
    //             .withTargetDirection(Rotation2d.fromDegrees(90))
    //         ).withTimeout(1.5),

    //         drivetrain.applyRequest(()-> drive
    //             .withVelocityY(3)
    //             .withVelocityX(0)
    //         ).withTimeout(.7) , 

            
       
    //         //stopping to get balls from human players

    //         drivetrain.applyRequest(() -> idle).withTimeout(3),
        
    //         drivetrain.applyRequest(() -> turnToAngle
    //             .withVelocityX(0)
    //             .withVelocityY(0)
    //             .withTargetDirection(Rotation2d.fromDegrees(0))
    //         ).withTimeout(1.5),

    //         drivetrain.applyRequest(() -> drive
    //             .withVelocityX(3)
    //             .withVelocityY(0)
    //             .withRotationalRate(0)
    //         ).withTimeout(.5),

    //         drivetrain.applyRequest(() -> idle).withTimeout(2),
    //         Commands.runOnce(() -> turret.runTurret()).withTimeout(3),

    //         drivetrain.applyRequest(() -> drive
    //             .withVelocityX(3)
    //             .withVelocityY(0)
    //             .withRotationalRate(0)
    //         ).withTimeout(2),

    //         drivetrain.applyRequest(() -> turnToAngle
    //             .withVelocityX(0)
    //             .withVelocityY(0)
    //             .withTargetDirection(Rotation2d.fromDegrees(90))
    //         ).withTimeout(1),

    //         drivetrain.applyRequest(() -> drive
    //             .withVelocityX(3)
    //             .withVelocityY(0)
    //             .withRotationalRate(0)
    //         ).withTimeout(1),

    //         drivetrain.applyRequest(() -> turnToAngle
    //             .withVelocityX(0)
    //             .withVelocityY(0)
    //             .withTargetDirection(Rotation2d.fromDegrees(-90))
    //         ).withTimeout(1),
           
    //         drivetrain.applyRequest(() -> drive
    //             .withVelocityX(3)
    //             .withVelocityY(0)
    //             .withRotationalRate(0)
    //         ).withTimeout(1),

    //         drivetrain.applyRequest(() -> turnToAngle
    //             .withVelocityX(0)
    //             .withVelocityY(0)
    //             .withTargetDirection(Rotation2d.fromDegrees(180))
    //         ).withTimeout(1),
           
    //         drivetrain.applyRequest(() -> drive
    //             .withVelocityX(3)
    //             .withVelocityY(0)
    //             .withRotationalRate(0)
    //         ).withTimeout(2)
    //         );
    // }

    // Middle red side
    public Command RED_MiddleAuto() {

        final var idle = new SwerveRequest.Idle();

        return new SequentialCommandGroup(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),

                drivetrain.applyRequest(()-> drive
                    .withVelocityX(-3)
                ).withTimeout(0.4),


                // stops robot to sit still
                drivetrain.applyRequest(()-> idle).withTimeout(1),

                Commands.run(()-> turret.runTurret()).withTimeout(3),

                drivetrain.applyRequest(() -> drive
                    .withVelocityX(0)
                    .withVelocityY(3)
                ).withTimeout(0.8),
                
                drivetrain.applyRequest(() -> drive
                    .withVelocityX(-3)
                    .withVelocityY(0)
                ).withTimeout(0.5),
                
                //stops robot to intake

                drivetrain.applyRequest(()-> idle).withTimeout(0.2),
                Commands.run(()-> intake.runFrontIntake()).withTimeout(1.50),

                drivetrain.applyRequest(() -> drive
                    .withVelocityX(3)
                    .withVelocityY(0)
                ).withTimeout(0.4),
        
                drivetrain.applyRequest(() -> drive
                    .withVelocityX(0)
                    .withVelocityY(-3)
                ).withTimeout(0.8),

                
                drivetrain.applyRequest(()-> drive
                    .withVelocityX(3)
                    .withVelocityY(0)
                ).withTimeout(0.7)


        );

    }


    public Command RED_LeftAuto() {
        final var idle = new SwerveRequest.Idle();

        return new SequentialCommandGroup(
            // drivetrain.applyRequest(() -> drive
            //     .withVelocityX(-3)
            //     .withVelocityY(0)
            //     .withRotationalRate(0)
            // ).withTimeout(0.5),
          
            // drivetrain.applyRequest(() -> idle).withTimeout(2),
            // Commands.runOnce(() -> turret.runTurret()).withTimeout(3),
            
            // Commands.runOnce(()-> intake.lowerIntake()
            // ).withTimeout(1),
            
            // Commands.run(()-> intake.runFrontIntake(), intake),

            drivetrain.applyRequest(() -> drive
                .withVelocityX(2)
                .withVelocityY(0)
                .withRotationalRate(0)
            ).withTimeout(1.5),

            drivetrain.applyRequest(() -> turnToAngle
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(Rotation2d.fromDegrees(-90))
            ).withTimeout(1),
         
            drivetrain.applyRequest(() -> drive
                .withVelocityX(2)
                .withVelocityY(0)
                .withRotationalRate(0)
            ).withTimeout(.90),

             drivetrain.applyRequest(() -> turnToAngle
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(Rotation2d.fromDegrees(0))
            ).withTimeout(.5),

            drivetrain.applyRequest(() -> drive
                .withVelocityX(2)
                .withVelocityY(0)
                .withRotationalRate(0)
            ).withTimeout(.2),

            drivetrain.applyRequest(() -> turnToAngle
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(Rotation2d.fromDegrees(-90))
            ).withTimeout(.5),

            drivetrain.applyRequest(() -> drive
                .withVelocityX(2)
                .withVelocityY(0)
                .withRotationalRate(0)
            ).withTimeout(.70)
            
            // Commands.run(()-> intake.stopFrontIntake(), intake),

            // drivetrain.applyRequest(() -> turnToAngle
            //     .withVelocityX(0)
            //     .withVelocityY(0)
            //     .withTargetDirection(Rotation2d.fromDegrees(180))
            // ).withTimeout(1),

            // drivetrain.applyRequest(() -> drive
            //     .withVelocityX(2)
            //     .withVelocityY(0)
            //     .withRotationalRate(0)
            // ).withTimeout(1),

            // drivetrain.applyRequest(() -> turnToAngle
            //     .withVelocityX(0)
            //     .withVelocityY(0)
            //     .withTargetDirection(Rotation2d.fromDegrees(90))
            // ).withTimeout(1)
        );
    }

    public Command RED_workingRight() {
    
    final var idle = new SwerveRequest.Idle();
    
        return new SequentialCommandGroup(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(2)
                .withVelocityY(0)
                .withRotationalRate(0)  
            ).withTimeout(1.70),

            drivetrain.applyRequest(() -> drive
                .withVelocityX(0)
                .withVelocityY(1)
                .withRotationalRate(0)
            ).withTimeout(4),

            drivetrain.applyRequest(() -> drive 
                .withVelocityX(2)
                .withVelocityY(0)
                .withRotationalRate(0)
            ).withTimeout(.20),

            drivetrain.applyRequest(() -> drive
                .withVelocityX(0)
                .withVelocityY(-2)
                .withRotationalRate(0)
            ).withTimeout(.30),

            
            drivetrain.applyRequest(() -> idle).withTimeout(1)
        );
    }


    // public Command BLUE_RightAuto() {

    // final var idle = new SwerveRequest.Idle();

    //     return new SequentialCommandGroup(

    //         drivetrain.applyRequest(() -> drive
    //             .withVelocityX(-3)
    //             .withVelocityY(0)
    //             .withRotationalRate(0)
    //         ).withTimeout(0.2),
            
    //         drivetrain.applyRequest(() -> idle).withTimeout(2),
    //         Commands.runOnce(() -> turret.runTurret()).withTimeout(3),

    //         drivetrain.applyRequest(() -> turnToAngle
    //             .withVelocityX(0)
    //             .withVelocityY(0)
    //             .withTargetDirection(Rotation2d.fromDegrees(-90))
    //         ).withTimeout(1.5),

    //         drivetrain.applyRequest(()-> drive
    //             .withVelocityY(3)
    //             .withVelocityX(0)
    //         ).withTimeout(.7), 

    //         //stopping to get balls from human players

    //         drivetrain.applyRequest(() -> idle).withTimeout(3),
        
    //         drivetrain.applyRequest(() -> turnToAngle
    //             .withVelocityX(0)
    //             .withVelocityY(0)
    //             .withTargetDirection(Rotation2d.fromDegrees(180))
    //         ).withTimeout(1.5),

    //         drivetrain.applyRequest(() -> drive
    //             .withVelocityX(3)
    //             .withVelocityY(0)
    //             .withRotationalRate(0)
    //         ).withTimeout(.5),

    //         drivetrain.applyRequest(() -> idle).withTimeout(2),
    //         Commands.runOnce(() -> turret.runTurret()).withTimeout(3),

    //         drivetrain.applyRequest(() -> drive
    //             .withVelocityX(3)
    //             .withVelocityY(0)
    //             .withRotationalRate(0)
    //         ).withTimeout(2),

    //         drivetrain.applyRequest(() -> turnToAngle
    //             .withVelocityX(0)
    //             .withVelocityY(0)
    //             .withTargetDirection(Rotation2d.fromDegrees(-90))
    //         ).withTimeout(1),

    //         drivetrain.applyRequest(() -> drive
    //             .withVelocityX(3)
    //             .withVelocityY(0)
    //             .withRotationalRate(0)
    //         ).withTimeout(1),

    //         drivetrain.applyRequest(() -> turnToAngle
    //             .withVelocityX(0)
    //             .withVelocityY(0)
    //             .withTargetDirection(Rotation2d.fromDegrees(90))
    //         ).withTimeout(1),
           
    //         drivetrain.applyRequest(() -> drive
    //             .withVelocityX(3)
    //             .withVelocityY(0)
    //             .withRotationalRate(0)
    //         ).withTimeout(1),

    //         drivetrain.applyRequest(() -> turnToAngle
    //             .withVelocityX(0)
    //             .withVelocityY(0)
    //             .withTargetDirection(Rotation2d.fromDegrees(0))
    //         ).withTimeout(1),
           
    //         drivetrain.applyRequest(() -> drive
    //             .withVelocityX(3)
    //             .withVelocityY(0)
    //             .withRotationalRate(0)
    //         ).withTimeout(2)
    //     );

    // }  
    
    public Command BLUE_workingRight() {
    
    final var idle = new SwerveRequest.Idle();
        return new SequentialCommandGroup(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(2)
                .withVelocityY(0)
                .withRotationalRate(0)  
            ).withTimeout(1.70),

            drivetrain.applyRequest(() -> drive
                .withVelocityX(0)
                .withVelocityY(1)
                .withRotationalRate(0)
            ).withTimeout(4),

            drivetrain.applyRequest(() -> drive 
                .withVelocityX(2)
                .withVelocityY(0)
                .withRotationalRate(0)
            ).withTimeout(.30),

            drivetrain.applyRequest(() -> drive
                .withVelocityX(0)
                .withVelocityY(-2)
                .withRotationalRate(0)
            ).withTimeout(.30),

            
            drivetrain.applyRequest(() -> idle).withTimeout(1)
        );
    }

    public Command BLUE_MiddleAuto() {
        
            final var idle = new SwerveRequest.Idle();
            
            return new SequentialCommandGroup(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),

                drivetrain.applyRequest(()-> drive
                    .withVelocityX(-3)
                ).withTimeout(0.4),


                // stops robot to sit still
                drivetrain.applyRequest(()-> idle).withTimeout(0.2),

                Commands.run(()-> turret.runTurret()).withTimeout(3),

                drivetrain.applyRequest(() -> drive
                    .withVelocityX(0)
                    .withVelocityY(3)
                ).withTimeout(0.8),
                
                drivetrain.applyRequest(() -> drive
                    .withVelocityX(-3)
                    .withVelocityY(0)
                ).withTimeout(0.5),
                
                //stops robot to intake

                drivetrain.applyRequest(()-> idle).withTimeout(0.2),
                Commands.run(()-> intake.runFrontIntake()).withTimeout(1.50),

                drivetrain.applyRequest(() -> drive
                    .withVelocityX(3)
                    .withVelocityY(0)
                ).withTimeout(0.4),
        
                drivetrain.applyRequest(() -> drive
                    .withVelocityX(0)
                    .withVelocityY(-3)
                ).withTimeout(0.8),

                
                drivetrain.applyRequest(()-> drive
                    .withVelocityX(3)
                    .withVelocityY(0)
                ).withTimeout(0.6)
        );
    }


    public Command BLUE_LeftAuto() {
    
        return new SequentialCommandGroup(
            // drivetrain.applyRequest(() -> drive
            //     .withVelocityX(-2)
            //     .withVelocityY(0)
            //     .withRotationalRate(0)
            // ).withTimeout(1),
            
            // drivetrain.applyRequest(() -> idle).withTimeout(2),
            // Commands.runOnce(() -> turret.runTurret()).withTimeout(3),

            // Commands.runOnce(()-> intake.lowerIntake()
            // ).withTimeout(1),
            
            // Commands.run(()-> intake.runFrontIntake(), intake),

            drivetrain.applyRequest(() -> drive
                .withVelocityX(2)
                .withVelocityY(0)
                .withRotationalRate(0)
            ).withTimeout(1.50),

            drivetrain.applyRequest(() -> turnToAngle
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(Rotation2d.fromDegrees(90))
            ).withTimeout(.90),

            drivetrain.applyRequest(() -> drive
                .withVelocityX(2)
                .withVelocityY(0)
                .withRotationalRate(0)
            ).withTimeout(1.75),

            drivetrain.applyRequest(() -> turnToAngle
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(Rotation2d.fromDegrees(-90))
            ).withTimeout(.90),
             
            // Commands.run(()-> intake.stopFrontIntake(), intake),

            drivetrain.applyRequest(() -> drive
                .withVelocityX(2)
                .withVelocityY(0)
                .withRotationalRate(0)
            ).withTimeout(.2),

             drivetrain.applyRequest(() -> turnToAngle
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(Rotation2d.fromDegrees(0))
            ).withTimeout(1),

            // Commands.run(()-> intake.stopFrontIntake(), intake),

             drivetrain.applyRequest(() -> drive
                .withVelocityX(2)
                .withVelocityY(0)
                .withRotationalRate(0)
            ).withTimeout(1.8),

              drivetrain.applyRequest(() -> turnToAngle
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(Rotation2d.fromDegrees(-90))
            ).withTimeout(1.5)
        );
    }




}
