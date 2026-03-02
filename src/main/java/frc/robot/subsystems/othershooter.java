package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Timer;

import java.util.prefs.BackingStoreException;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class othershooter extends SubsystemBase {

    // Motors
    public final TalonFX leftShooter = new TalonFX(17);
    public final TalonFX rightShooter = new TalonFX(19);
    public final TalonFX backIntake = new TalonFX(18);
    public final TalonFX frontIntake = new TalonFX(16);

    // Motor speeds
    public final double SHOOTER_SPEED = 1;
    public final double SHOOTER_INTAKE_SPEED = 1.0;
    public final double INTAKE_SPEED = 0.6;
    public final double FRONT_INTAKE_SHOOTER_SPEED = 0.4;
    public final double BACK_INTAKE_SHOOTER_SPEED = 1;

    // DutyCycleOut objects
    public final DutyCycleOut shooterSpeed = new DutyCycleOut(SHOOTER_SPEED);
    public final DutyCycleOut shooterSpeedReversed = new DutyCycleOut(-SHOOTER_SPEED);

    public final DutyCycleOut shooterIntakeSpeed = new DutyCycleOut(SHOOTER_INTAKE_SPEED);
    public final DutyCycleOut shooterIntakeSpeedReversed = new DutyCycleOut(-SHOOTER_INTAKE_SPEED);
    
    public final DutyCycleOut intakeSpeed = new DutyCycleOut(INTAKE_SPEED);
    public final DutyCycleOut intakeSpeedReversed = new DutyCycleOut(-INTAKE_SPEED);

    public final DutyCycleOut frontIntakeShooterSpeed = new DutyCycleOut(FRONT_INTAKE_SHOOTER_SPEED);
    public final DutyCycleOut frontIntakeShooterSpeedReversed = new DutyCycleOut(-FRONT_INTAKE_SHOOTER_SPEED);

    public final DutyCycleOut backIntakeShooterSpeed = new DutyCycleOut(BACK_INTAKE_SHOOTER_SPEED);
    public final DutyCycleOut backIntakeShooterSpeedReversed = new DutyCycleOut(-BACK_INTAKE_SHOOTER_SPEED);

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

    /**
     * Staged shooting for button press:
     * 0–2s: shooter spins, intake backwards to ramp up
     * 2–10s: intake forwards to feed balls, shooter continues
     */
    
   
    private final Timer shooterTimer = new Timer();
    private boolean shooterActive = false;

    public void shootersequence (){
        if(!shooterActive){
            shooterTimer.reset();
            shooterTimer.start();
            shooterActive=true;
        }
        double time = shooterTimer.get();
        if(time<2.0){
            leftShooter.setControl(shooterSpeed);
            rightShooter.setControl(shooterSpeedReversed);
            frontIntake.setControl(intakeSpeedReversed);
            backIntake.setControl(intakeSpeed);
        } else if(time<10){
            shootFuel();
        } else{
            //stops motors if the timer is >10 sec
            stopAllMotors();
            shooterTimer.stop();
            shooterActive=false;
        }
    }
    public void resetTimer(){
        shooterTimer.reset();
        shooterTimer.stop();
        shooterActive=false;
    }
     
    
}
