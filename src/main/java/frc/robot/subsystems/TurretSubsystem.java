package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;


public class TurretSubsystem extends SubsystemBase {
    public final SparkMax turningMotor = new SparkMax(0, MotorType.kBrushless);
     private static final DutyCycleOut STOP = new DutyCycleOut(0);
    public final TalonFX shootMotor = new TalonFX(1);
    public final double SHOOTER_SPEED = 0.9;
    


    /**Run turn motor */
    public void turnTurretRight(double speed){
        turningMotor.set(speed);
    }

    public void turnTurretLeft(double speed){
        turningMotor.set(-speed);
    }
        
    public void stopTurret(){
        turningMotor.set(0);
    }
    



    /** Run shooter for shooting fuel */
    public void runTurret(){
        DutyCycleOut newSpeed = new DutyCycleOut(SHOOTER_SPEED);
        shootMotor.setControl(newSpeed);
    }

    public void reverseTurret(){
        DutyCycleOut newSpeedREV = new DutyCycleOut(-SHOOTER_SPEED);
        shootMotor.setControl(newSpeedREV);
    }

    /** Stop shooter */
    public void stopShooting(){
        shootMotor.setControl(STOP);
    }


    }
















