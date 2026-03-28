package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climbSubsystem extends SubsystemBase {
    public TalonFX climberMotor = new TalonFX(16);

    public void runClimberMotor(double speed){
        DutyCycleOut output = new DutyCycleOut(speed);
        climberMotor.setControl(output);
    }

    public void reverseClimberMotor(double speed){
        DutyCycleOut output = new DutyCycleOut(-speed);
        climberMotor.setControl(output);
    }

    public void stopClimberMotor(){
        DutyCycleOut output = new DutyCycleOut(0);
        climberMotor.setControl(output);
    }
}
