package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.IOAllocation;

public class IntakeSubsystem extends SubsystemBase {

    public enum State {
        IDLE,
        INTAKE
    }

    private final TalonFX motor;
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);
    private State targetState;

    public IntakeSubsystem() {
        motor = IOAllocation.CAN.kIntakeMotor.createTalonFX();
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // --- PID GAINS ---
        //TODO: These values need tuning! Start with kP at 0.1 and kV at 0.12 DO NOT SET kI or kD YET, as they are not always necessary and can cause instability if set too high
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.001;
        config.Slot0.kV = 0.12; // kV is vital for velocity control

        motor.getConfigurator().apply(config);

        targetState = State.IDLE;
    }

    @Override
    public void periodic() {

        double targetRPS = 0;
        switch (targetState) {
            case INTAKE -> targetRPS = Constants.kIntakeRPS.get();
            case IDLE -> targetRPS = Constants.kIntakeIdleRPS.get();
        }

        // Apply control
        motor.setControl(velocityControl.withVelocity(targetRPS));
        
    }

    public void setTargetState(State targetState) {
        this.targetState = targetState;
    }

    public Command commandSetState(State targetState) {
        return Commands.run(() -> setTargetState(targetState), this);
    }
}
