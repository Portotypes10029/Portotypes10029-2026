package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeSubsystem {
     private static final DutyCycleOut STOP = new DutyCycleOut(0);
    public final TalonFX shootMotor = new TalonFX(11);
    public final double SHOOTER_SPEED = 0.9;
    

     /** Run shooter for shooting fuel */
    public void runIntake(){
        DutyCycleOut newSpeed = new DutyCycleOut(SHOOTER_SPEED);
        shootMotor.setControl(newSpeed);
    }

    public void reverseIntake(){
        DutyCycleOut newSpeedREV = new DutyCycleOut(-SHOOTER_SPEED);
        shootMotor.setControl(newSpeedREV);
    }

    /** Stop shooter */
    public void stopIntake(){
        shootMotor.setControl(STOP);
    }
    
}
