package com.swrobotics.robot.subsystems.hood;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    enum State{
        IDLE,
        MOVING,
        READY
    }
    private final TalonFX motor;
    private final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0).withSlot(0);
    private State targetState;

    public HoodSubsystem() {
        motor = IOAllocation.CAN.kHoodMotor.createTalonFX();

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // PID gains (slot 0) – tune later
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.001;

        // Motion Magic profile using constants
        MotionMagicConfigs jerry2 = new MotionMagicConfigs();
        jerry2.MotionMagicCruiseVelocity = Constants.kHoodCruiseVelocity.get();
        jerry2.MotionMagicAcceleration   = Constants.kHoodCruiseVelocity.get();
        config.MotionMagic = jerry2;

        config.apply(motor);

        motor.setPosition(0); // zero at startup

        targetState = State.IDLE;
    }

        @Override
    public void periodic() {
        double targetRotations;

        switch (targetState) {
            case IDLE:
            default:
                targetRotations = Constants.kHoodIdleRotations.get();
                break;
            case MOVING:
                targetRotations = Constants.kHoodMovingRotations.get();
            
                break;
            case READY:
                targetRotations = Constants.kHoodReadyRotations.get();
                break;
        }

        motor.setControl(m_motionMagic.withPosition(targetRotations));
    }

    public void setTargetState(State targetState) {
        this.targetState = targetState;
    }

    public Command commandSetState(State targetState) {
        return Commands.run(() -> setTargetState(targetState), this);
    }
    
}
