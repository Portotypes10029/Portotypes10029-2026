package frc.robot.subsystems;

import java.util.prefs.BackingStoreException;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    // Motors
    public final TalonFX leftShooter = new TalonFX(17);

    public final TalonFX rightShooter = new TalonFX(19);
    public final TalonFX backIntake = new TalonFX(18);
    public final TalonFX frontIntake = new TalonFX(16);

    // Motor speeds - Change these values for adjustment
    public final double SHOOTER_SPEED = 0.9; // only for the shooting motors 

    public final double INTAKE_SPEED = 0.40; // speed for only the intaking (Left Trigger)

    public final double FRONT_INTAKE_SHOOTER_SPEED = 0.4; // speed for intake motors when shooting (Right Trigger)
    public final double BACK_INTAKE_SHOOTER_SPEED = 1; // speed for intake motors when shooting (Right Trigger)



    // DutyCycleOut objects
    public final DutyCycleOut shooterSpeed = new DutyCycleOut(SHOOTER_SPEED);
    public final DutyCycleOut shooterSpeedReversed = new DutyCycleOut(-SHOOTER_SPEED);

   // public final DutyCycleOut shooterIntakeSpeed = new DutyCycleOut(SHOOTER_INTAKE_SPEED);
   // public final DutyCycleOut shooterIntakeSpeedReversed = new DutyCycleOut(-SHOOTER_INTAKE_SPEED);
    
    public final DutyCycleOut intakeSpeed = new DutyCycleOut(INTAKE_SPEED);
    public final DutyCycleOut intakeSpeedReversed = new DutyCycleOut(-INTAKE_SPEED);

    public final DutyCycleOut frontIntakeShooterSpeed = new DutyCycleOut(FRONT_INTAKE_SHOOTER_SPEED);
    public final DutyCycleOut frontIntakeShooterSpeedReversed = new DutyCycleOut(-FRONT_INTAKE_SHOOTER_SPEED);

    public final DutyCycleOut backIntakeShooterSpeed = new DutyCycleOut(BACK_INTAKE_SHOOTER_SPEED);
    public final DutyCycleOut backIntakeShooterSpeedReversed = new DutyCycleOut(-BACK_INTAKE_SHOOTER_SPEED);

    public final DutyCycleOut shooterIdleSpeed = new DutyCycleOut(0.5);
    public final DutyCycleOut rightShooterIdleSpeed = new DutyCycleOut(-0.5);

    private static final DutyCycleOut STOP = new DutyCycleOut(0);

    public ShooterSubsystem() { 
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.CurrentLimits.StatorCurrentLimit = 40;
        shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        leftShooter.getConfigurator().apply(shooterConfig);
        rightShooter.getConfigurator().apply(shooterConfig);
        frontIntake.getConfigurator().apply(shooterConfig);
        backIntake.getConfigurator().apply(shooterConfig);
    }






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

    public void frontIntakeSpeed() {
        frontIntake.setControl(frontIntakeShooterSpeedReversed);
    }
    public void backIntakeSpeed() {
        backIntake.setControl(backIntakeShooterSpeedReversed);
    }

    /** Stop all shooter and intake motors */
    public void stopIntakeMotors() {
        frontIntake.setControl(STOP);
        backIntake.setControl(STOP);
    }

    public void stopAllMotors() {
        frontIntake.setControl(STOP);
        backIntake.setControl(STOP);
        leftShooter.setControl(STOP);
        rightShooter.setControl(STOP);
    }


    //AUTO FUNCS

    public void setIntakeSpeed(double speed) {
        DutyCycleOut newSpeed = new DutyCycleOut(speed);
        DutyCycleOut newSpeedREV = new DutyCycleOut(-speed);
        frontIntake.setControl(newSpeed);
        backIntake.setControl(newSpeedREV);
    }

    public void stopIntake() {
        frontIntake.setControl(STOP);
        backIntake.setControl(STOP); 
    }

    public void setShooterSpeed(double speed) {
        DutyCycleOut output = new DutyCycleOut(speed);
        DutyCycleOut outputREV = new DutyCycleOut(-speed);
        rightShooter.setControl(outputREV);
        leftShooter.setControl(output);
    }

    public void stopShooter() {
        rightShooter.setControl(STOP);
        leftShooter.setControl(STOP);
    }
}
