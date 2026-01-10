package com.swrobotics.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.subsystems.motortracker.MotorTrackerSubsystem;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    private enum State {
        IDLE,
        HOLD,
        SHOOT
    }

    private State state = State.IDLE;

    private final DigitalInput beamBreak = new DigitalInput(IOAllocation.RIO.kDIO_IndexerBeamBreak);
    private final TalonFX motor;
    private final VoltageOut voltageOut = new VoltageOut(0);
    
    public IndexerSubsystem() {
        // Configure motor
        motor = IOAllocation.CAN.kIndexerMotor.createTalonFX();
        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.apply(motor);
        
        // Register motor for tracking and music
        MotorTrackerSubsystem.getInstance().addMotor("Indexer", motor);
        MusicSubsystem.getInstance().addInstrument(motor);
    }

    private void setState(State newState) {
        state = newState;
    } 

    public Command shoot() {
        return Commands.runOnce(() -> this.setState(State.SHOOT));
    }

    public boolean hasPiece() {
        return !beamBreak.get();
    }

    @Override
    public void periodic() {
        switch (state) {
            case IDLE:

                if (hasPiece()) {
                    setState(State.HOLD);
                } else {
                    // Run the motor slowly
                    motor.setControl(voltageOut.withOutput(Constants.kIndexerIdleVoltage.get()));
                }

                break;
            case HOLD:
                // Don't run the motor
                motor.setControl(voltageOut.withOutput(0));
                break;
            case SHOOT:
                // Run the motor quickly
                motor.setControl(voltageOut.withOutput(Constants.kIndexerShootVoltage.get()));
                break;
        }

        Logger.recordOutput("Indexer/State", state);
        Logger.recordOutput("Indexer/Has Piece", hasPiece());
    }
}