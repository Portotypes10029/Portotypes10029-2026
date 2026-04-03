package frc.robot.subsystems;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class TurretSubsystem extends SubsystemBase {
    public final SparkMax turningMotor = new SparkMax(20, MotorType.kBrushless);
    private static final DutyCycleOut STOP = new DutyCycleOut(0);

    private final RelativeEncoder turretEncoder = turningMotor.getEncoder();

    // gear ratio for turning motor
    private static final double GEAR_RATIO = 9.0;

    // max rotation for turret
    private static final double MAX_DEGREES = 180.0;

    // How close (in degrees) counts as "arrived" at a snap target
    private static final double SNAP_DEADBAND = 3.0;

    // Speed used when snapping to a cardinal angle
    private static final double SNAP_SPEED = 0.15;

    private boolean isUnwinding = false;

    // Target angle for snap-to-cardinal, null means no snap active
    private Double snapTargetDegrees = null;

    public final TalonFX shootMotor = new TalonFX(24);
    private final TalonFX otherShootMotor = new TalonFX(25);
    public final double SHOOTER_SPEED = 0.9;
    private final TalonFX turretIntakeMotor = new TalonFX(18);
    private final double turretIntakeSpeed = 0.5;

    public TurretSubsystem() {
        turretEncoder.setPosition(0);
    }

    // get turret encoder and turret angle

    public double getTurretDegrees() {
        return (turretEncoder.getPosition() / GEAR_RATIO) * 360.0;
    }

    public void zeroEncoder() {
        turretEncoder.setPosition(0);
    }

    // ── Cardinal Snap ─────────────────────────────────────────────────────────

    /**
     * Returns the nearest cardinal angle (multiple of 90) to the current position.
     * e.g. if at 74°, returns 90. If at 130°, returns 90. If at 46°, returns 0 (or 90).
     */
    public double getNearestCardinal() {
        double current = getTurretDegrees();
        // Round to the nearest multiple of 90
        return Math.round(current / 90.0) * 90.0;
    }

    /**
     * Snaps to the next 90-degree position in the given direction.
     * direction > 0 = snap to next cardinal clockwise (right)
     * direction < 0 = snap to next cardinal counter-clockwise (left)
     *
     * Safe — will trigger an unwind instead if the target would exceed MAX_DEGREES.
     *
     * Example usage in a command:
     *   turret.snapToNextCardinal(1);   // snap right
     *   turret.snapToNextCardinal(-1);  // snap left
     */
    public void snapToNextCardinal(int direction) {
        double nearest = getNearestCardinal();
        double target;

        if (direction > 0) {
            target = nearest + 90.0;
        } else {
            target = nearest - 90.0;
        }

        // If snapping would exceed the wire limit, unwind instead
        if (target >= MAX_DEGREES || target <= -MAX_DEGREES) {
            startUnwind();
            return;
        }

        snapTargetDegrees = target;
        isUnwinding = false;
    }

    /**
     * Cancels any active snap-to-cardinal in progress.
     */
    public void cancelSnap() {
        snapTargetDegrees = null;
    }

    /**
     * Drives the motor toward the current snapTargetDegrees.
     * Stops and clears the target once within SNAP_DEADBAND degrees.
     */
    private void handleSnap() {
        if (snapTargetDegrees == null) return;

        double current = getTurretDegrees();
        double error = snapTargetDegrees - current;

        if (Math.abs(error) <= SNAP_DEADBAND) {
            // Arrived — stop motor and clear target
            turningMotor.set(0);
            snapTargetDegrees = null;
        } else if (error > 0) {
            turningMotor.set(SNAP_SPEED);
        } else {
            turningMotor.set(-SNAP_SPEED);
        }
    }

    // turret rotation methods
    public void turnTurretRight(double speed) {
        // if (getTurretDegrees() >= MAX_DEGREES) {
        //     startUnwind();
        //     return;
        // }
        snapTargetDegrees = null;
        isUnwinding = false;
        turningMotor.set(speed);
    }

    public void turnTurretLeft(double speed) {
        // if (getTurretDegrees() <= -MAX_DEGREES) {
        //     startUnwind();
        //     return;
        // }
        snapTargetDegrees = null;
        isUnwinding = false;
        turningMotor.set(-speed);
    }

    public void stopTurret() {
        turningMotor.set(0);
        isUnwinding = false;
        snapTargetDegrees = null;
    }







    // unwinding methods to handle 
    private void startUnwind() {
        isUnwinding = true;
        snapTargetDegrees = null;
    }

    private void handleUnwind() {
        double current = getTurretDegrees();
        if (Math.abs(current) < 5.0) {
            turningMotor.set(0);
            isUnwinding = false;
        } else if (current > 0) {
            turningMotor.set(-0.4);
        } else {
            turningMotor.set(0.4);
        }
    }

    // turret intake
    public void runTurretIntake() {
        turretIntakeMotor.setControl(new DutyCycleOut(turretIntakeSpeed));
    }

    public void reverseTurretIntake() {
        turretIntakeMotor.setControl(new DutyCycleOut(-turretIntakeSpeed));
    }

    public void stopTurretIntake() {
        turretIntakeMotor.setControl(STOP);
    }


    //
    public void runTurret() {
        shootMotor.setControl(new DutyCycleOut(-SHOOTER_SPEED));
        otherShootMotor.setControl(new DutyCycleOut(-SHOOTER_SPEED));
    }

    public void reverseTurret() {
        shootMotor.setControl(new DutyCycleOut(SHOOTER_SPEED));
        otherShootMotor.setControl(new DutyCycleOut(SHOOTER_SPEED));
    }

    public void stopShooting() {
        shootMotor.setControl(STOP);
        otherShootMotor.setControl(STOP);
    }


    // periodic to handle unwinding
    @Override
    public void periodic() {
        // if (isUnwinding) {
        //     handleUnwind();
        // } else if (snapTargetDegrees != null) {
        //     handleSnap();
        // }

        SmartDashboard.putNumber("Turret Degrees", getTurretDegrees());
        SmartDashboard.putNumber("Turret Nearest Cardinal", getNearestCardinal());
        SmartDashboard.putBoolean("Turret Unwinding", isUnwinding);
        SmartDashboard.putBoolean("Turret Snapping", snapTargetDegrees != null);
    }
}