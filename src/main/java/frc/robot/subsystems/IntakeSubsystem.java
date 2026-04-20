package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeSubsystem extends SubsystemBase {
    public final SparkMax IntakeMotor = new SparkMax(11, MotorType.kBrushless);
    public final double IntakeSpeed = 0.3;

    //public final TalonFX conveyorBelt = new TalonFX(19);
    private final double conveyorSpeed = 0.25;

    private final TalonFX frontIntakeMotor = new TalonFX(29);
    private final double frontIntakeSpeed = 0.7; // raise to 0.8 later

    private final RelativeEncoder raiseMotorEncoder = IntakeMotor.getEncoder();

    public IntakeSubsystem() {
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake);

        // Persist parameters to retain configuration in the event of a power cycle
        IntakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        raiseMotorEncoder.setPosition(0);

    }


     /** Run shooter for shooting fuel */
    public void raiseIntake(){
        IntakeMotor.set(IntakeSpeed);
    }

    public void lowerIntake(){
       IntakeMotor.set(-IntakeSpeed);
    }

    /** Stop shooter */
    public void stopIntake(){
        IntakeMotor.set(0);
    }


    // public void runConveyor(){
    //     DutyCycleOut output = new DutyCycleOut(conveyorSpeed);
    //     conveyorBelt.setControl(output);
    // }
    
    // public void reverseConveyor(){
    //     DutyCycleOut output = new DutyCycleOut(-conveyorSpeed);
    //     conveyorBelt.setControl(output);
    // }

    // public void stopConveyor(){
    //     DutyCycleOut output = new DutyCycleOut(0);
    //     conveyorBelt.setControl(output);
    // }



    public void runFrontIntake() {
        DutyCycleOut output = new DutyCycleOut(-frontIntakeSpeed);
        frontIntakeMotor.setControl(output);
    }
    public void reverseFrontIntake() {
        DutyCycleOut output = new DutyCycleOut(frontIntakeSpeed);
        frontIntakeMotor.setControl(output);
    }

    public void stopFrontIntake() {
        DutyCycleOut output = new DutyCycleOut(0);
        frontIntakeMotor.setControl(output);
    }
    
    private double getTurretDegrees() {
        return (raiseMotorEncoder.getPosition());
    }

    private void zeroEncoder() {
        raiseMotorEncoder.setPosition(0);
    }

    public void moveIntake(double targetValue) {
        double current = getTurretDegrees();
        double target = targetValue;

        

    }


}   
