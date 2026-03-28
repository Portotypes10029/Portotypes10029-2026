package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class TurnToTag extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final LimelightSubsystem limelight;
    private final String llName;
    private final int targetID;

    private final SwerveRequest.RobotCentric drive;

    public TurnToTag(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight, SwerveRequest.RobotCentric drive, String limelightName, int tagID) {

        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.llName = limelightName;
        this.targetID = tagID;
        this.drive = drive;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (!limelight.hasTarget(llName)) return;

        if ((int)limelight.getID(llName) != targetID) return;

        double tx = limelight.getTX(llName);

        double kP = 0.02; // tune this
        double rotation = -tx * kP;

        drivetrain.applyRequest(() -> drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(rotation)
        );
    }

    @Override
    public boolean isFinished() {
        return limelight.hasTarget(llName)
            && (int)limelight.getID(llName) == targetID
            && Math.abs(limelight.getTX(llName)) < 1.5;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.applyRequest(() -> drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0)
        );
    }
}