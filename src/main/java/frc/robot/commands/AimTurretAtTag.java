package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;


public class AimTurretAtTag extends Command {

    private final TurretSubsystem turret;
    private final LimelightSubsystem limelight;
    private final String llName;
    private final int targetID;

    public AimTurretAtTag(TurretSubsystem turret,
                          LimelightSubsystem limelight,
                          String limelightName,
                          int tagID) {
        this.turret = turret;
        this.limelight = limelight;
        this.llName = limelightName;
        this.targetID = tagID;

        addRequirements(turret);
    }

    @Override
    public void execute() {

        if (!limelight.hasTarget(llName)) {
            turret.stopTurret();
            return;
        }

        if ((int)limelight.getID(llName) != targetID) {
            turret.stopTurret();
            return;
        }

        double tx = limelight.getTX(llName);

        double kP = 0.02; // tune this
        double speed = tx * kP;

        speed = Math.max(-0.4, Math.min(0.4, speed));

        // apply direction
        if (Math.abs(tx) < 1.0) {
            turret.stopTurret();
        } else if (tx > 0) {
            turret.turnTurretRight(speed);
        } else {
            turret.turnTurretLeft(-speed);
        }
    }

    @Override
    public boolean isFinished() {
        return limelight.hasTarget(llName)
            && (int)limelight.getID(llName) == targetID
            && Math.abs(limelight.getTX(llName)) < 1.0;
    }

    @Override
    public void end(boolean interrupted) {
        turret.stopTurret();
    }
}