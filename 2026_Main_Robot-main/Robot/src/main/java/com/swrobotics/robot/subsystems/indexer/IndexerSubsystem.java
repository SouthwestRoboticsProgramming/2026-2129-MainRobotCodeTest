package com.swrobotics.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.swrobotics.robot.config.IOAllocation;

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

    
    public IndexerSubsystem() {
        motor = IOAllocation.CAN.kIndexerMotor.createTalonFX();
        
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
                }

                break;
            case HOLD:
                // Don't run the motor
                break;
            case SHOOT:
                // Run the motor quickly
                break;
        }

        Logger.recordOutput("Indexer/State", state);
    }
}