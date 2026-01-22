package com.swrobotics.robot.subsystems.expansions;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// ...

public class ExpansionSubsystem extends SubsystemBase {

    public enum State {
        RETRACTED,
        EXTENDED
    }

    private final TalonFX motor;
    private final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0).withSlot(0);
    private State targetState;

    public ExpansionSubsystem() {
        motor = IOAllocation.CAN.kExpansionMotor.createTalonFX();

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // PID gains (slot 0) – tune later
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.001;

        // Motion Magic profile using constants
        MotionMagicConfigs jerry = new MotionMagicConfigs();
        jerry.MotionMagicCruiseVelocity = Constants.kExpansionCruiseVelocity.get();
        jerry.MotionMagicAcceleration   = Constants.kExpansionAcceleration.get();
        config.MotionMagic = jerry;

        config.apply(motor);

        motor.setPosition(0); // zero at startup

        MusicSubsystem.getInstance().addInstrument(motor);

        targetState = State.RETRACTED;
    }

        @Override
    public void periodic() {
        double targetRotations;

        switch (targetState) {
            case EXTENDED:
                targetRotations = Constants.kExpansionExtendedRotations.get();
                break;
            case RETRACTED:
            default:
                targetRotations = Constants.kExpansionRetractedRotations.get();
                break;
        }

        Logger.recordOutput("Expansion/TargetState", targetState.toString());
        Logger.recordOutput("Expansion/TargetRotations", targetRotations);
        Logger.recordOutput("Expansion/PositionRotations", motor.getPosition().getValueAsDouble());

        motor.setControl(m_motionMagic.withPosition(targetRotations));
    }

    public void setTargetState(State targetState) {
        this.targetState = targetState;
    }

    public Command commandSetState(State targetState) {
        return Commands.run(() -> setTargetState(targetState), this);
    }
}

