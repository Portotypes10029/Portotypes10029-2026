package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    // Toggle for live tuning
    // true = live dashboard tuning, false = constants
    private static final boolean USE_LIVE_TUNING = false; 

    // Constants | Change only when values are decided.
    private static final double SHOOTER_SPEED = 1.0;
    private static final double SHOOTING_INTAKE_SPEED = 1.0;
    private static final double INTAKE_SPEED = 0.6;

    



    //? Don't change below this line. ===================
    public ShooterSubsystem() {
        // Publish dashboard values if using live tuning
        SmartDashboard.putBoolean("Live Tuning", USE_LIVE_TUNING);
        if (USE_LIVE_TUNING) {
            SmartDashboard.putNumber("Shooter Speed", SHOOTER_SPEED);
            SmartDashboard.putNumber("Shooting Intake Speed", SHOOTING_INTAKE_SPEED);
            SmartDashboard.putNumber("Intake Speed", INTAKE_SPEED);
        }
    }

    // Motors
    private final TalonFX leftShooter = new TalonFX(17);
    private final TalonFX rightShooter = new TalonFX(19);
    private final TalonFX frontIntake = new TalonFX(16);
    private final TalonFX backIntake = new TalonFX(18);

    // Stop object (reused)
    private static final DutyCycleOut STOP = new DutyCycleOut(0);

    /** Run shooter motors and intakes for shooting fuel */
    public void shootFuel() {
        double shooterSpeed;
        if (USE_LIVE_TUNING) {
            shooterSpeed = SmartDashboard.getNumber("Shooter Speed", SHOOTER_SPEED);
        } else {
            shooterSpeed = SHOOTER_SPEED;
        }

        double shootingIntakeSpeed;
        if (USE_LIVE_TUNING) {
            shootingIntakeSpeed = SmartDashboard.getNumber("Shooting Intake Speed", SHOOTING_INTAKE_SPEED);
        } else {
            shootingIntakeSpeed = SHOOTING_INTAKE_SPEED;
        }

        leftShooter.setControl(new DutyCycleOut(shooterSpeed));
        rightShooter.setControl(new DutyCycleOut(-shooterSpeed));
        frontIntake.setControl(new DutyCycleOut(shootingIntakeSpeed));
        backIntake.setControl(new DutyCycleOut(shootingIntakeSpeed));
    }

    /** Run intake motors to pick up fuel */
    public void intakeFuel() {
        double intakeSpeed;
        if (USE_LIVE_TUNING) {
            intakeSpeed = SmartDashboard.getNumber("Intake Speed", INTAKE_SPEED);
        } else {
            intakeSpeed = INTAKE_SPEED;
        }

        frontIntake.setControl(new DutyCycleOut(intakeSpeed));
        backIntake.setControl(new DutyCycleOut(-intakeSpeed));
    }


    /** Stop all shooter and intake motors */
    public void stopAllMotors() {
        leftShooter.setControl(STOP);
        rightShooter.setControl(STOP);
        frontIntake.setControl(STOP);
        backIntake.setControl(STOP);
    }
}
