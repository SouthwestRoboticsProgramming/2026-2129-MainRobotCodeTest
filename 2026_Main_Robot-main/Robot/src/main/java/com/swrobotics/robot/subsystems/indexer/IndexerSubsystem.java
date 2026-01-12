package com.swrobotics.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.controls.VelocityVoltage; // Switched to Velocity
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.subsystems.motortracker.MotorTrackerSubsystem;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

    public enum State {
        IDLE,
        INTAKE
    }

    private final TalonFX motor;
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);
    private State targetState;

    public IndexerSubsystem() {
        motor = IOAllocation.CAN.kIndexerMotor.createTalonFX();
        
        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // --- PID GAINS ---
        //TODO: These values need tuning! Start with kP at 0.1 and kV at 0.12 DO NOT SET kI or kD YET, as they are not always necessary and can cause instability if set too high
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.001;
        config.Slot0.kV = 0.12; // kV is vital for velocity control

        config.apply(motor);
            
        MotorTrackerSubsystem.getInstance().addMotor("Indexer", motor);
        MusicSubsystem.getInstance().addInstrument(motor);

        targetState = State.IDLE;
    }

    @Override
    public void periodic() {
        // Refresh PID gains from NetworkTables if they changed
        

        double targetRPS = 0;
        switch (targetState) {
            case INTAKE -> targetRPS = Constants.kIndexerIntakeRPS.get();
            case IDLE -> targetRPS = Constants.kIndexerIdleRPS.get();
        }

        // Apply control
        motor.setControl(velocityControl.withVelocity(targetRPS));
        
        Logger.recordOutput("Indexer/TargetRPS", targetRPS);
        Logger.recordOutput("Indexer/ActualRPS", motor.getVelocity().getValueAsDouble());
    }

    public void setTargetState(State targetState) {
        this.targetState = targetState;
    }

    public Command commandSetState(State targetState) {
        return Commands.run(() -> setTargetState(targetState), this);
    }
}