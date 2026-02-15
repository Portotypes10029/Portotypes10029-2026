package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class ClimberSubsystem implements Subsystem {

    public ClimberSubsystem() {}

    private final TalonFX climberMotor = new TalonFX(20);

    public void runClimber(double speed) {
        climberMotor.setControl(new DutyCycleOut(speed));
    }

    public void stopClimber() {
        climberMotor.setControl(new DutyCycleOut(0));
    }
}
