package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    // Motors
    public final TalonFX leftShooter = new TalonFX(17);
    public final TalonFX rightShooter = new TalonFX(19);
    public final TalonFX backIntake = new TalonFX(18);
    public final TalonFX frontIntake = new TalonFX(16);

    // Motor speeds
    private final double SHOOTER_SPEED = 1;
    private final double SHOOTER_INTAKE_SPEED = 1.0;
    private final double INTAKE_SPEED = 0.6;
    private final double FRONT_INTAKE_SHOOTER_SPEED = 0.4;
    private final double BACK_INTAKE_SHOOTER_SPEED = 1;



    // DutyCycleOut objects
    private final DutyCycleOut shooterSpeed = new DutyCycleOut(SHOOTER_SPEED);
    private final DutyCycleOut shooterSpeedReversed = new DutyCycleOut(-SHOOTER_SPEED);

    private final DutyCycleOut shooterIntakeSpeed = new DutyCycleOut(SHOOTER_INTAKE_SPEED);
    private final DutyCycleOut shooterIntakeSpeedReversed = new DutyCycleOut(-SHOOTER_INTAKE_SPEED);
    
    private final DutyCycleOut intakeSpeed = new DutyCycleOut(INTAKE_SPEED);
    private final DutyCycleOut intakeSpeedReversed = new DutyCycleOut(-INTAKE_SPEED);

    private final DutyCycleOut frontIntakeShooterSpeed = new DutyCycleOut(FRONT_INTAKE_SHOOTER_SPEED);
    private final DutyCycleOut frontIntakeShooterSpeedReversed = new DutyCycleOut(-FRONT_INTAKE_SHOOTER_SPEED);

    private final DutyCycleOut backIntakeShooterSpeed = new DutyCycleOut(BACK_INTAKE_SHOOTER_SPEED);
    private final DutyCycleOut backIntakeShooterSpeedReversed = new DutyCycleOut(-BACK_INTAKE_SHOOTER_SPEED);

    private static final DutyCycleOut STOP = new DutyCycleOut(0);

    /** Run shooter and intake for shooting fuel */
    public void shootFuel() {
        leftShooter.setControl(shooterSpeed);
        rightShooter.setControl(shooterSpeedReversed);
        frontIntake.setControl(frontIntakeShooterSpeedReversed);
        backIntake.setControl(backIntakeShooterSpeedReversed);
    }

    /** Run intake motors to pick up fuel */
    public void intakeFuel() {
        frontIntake.setControl(intakeSpeed);
        backIntake.setControl(intakeSpeedReversed);
    }




    /** Stop all shooter and intake motors */
    public void stopAllMotors() {
        leftShooter.setControl(STOP);
        rightShooter.setControl(STOP);
        frontIntake.setControl(STOP);
        backIntake.setControl(STOP);
    }
}
