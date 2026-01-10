package com.swrobotics.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.subsystems.motortracker.MotorTrackerSubsystem;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;

import org.littletonrobotics.junction.Logger;

public class ShooterIOReal implements ShooterIO {
    private final TalonFX motor;

    private final VoltageOut voltageControl;

    public ShooterIOReal() {
        motor = IOAllocation.CAN.kShooterMotor.createTalonFX();

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.addTunable(Constants.kShooterPID);
        config.apply(motor);

        MotorTrackerSubsystem.getInstance().addMotor("Shooter", motor);
        MusicSubsystem.getInstance().addInstrument(motor);

        voltageControl = new VoltageOut(0)
                .withEnableFOC(true);
    }

    @Override
    public void updateInputs(Inputs inputs) {
        inputs.voltage = 0;
    }

    @Override
    public void setVoltage(double voltage) {
        Logger.recordOutput("Shooter/State", "Voltage");
        Logger.recordOutput("Shooter/Voltage Out", voltage);
        motor.setControl(voltageControl.withOutput(voltage));
    }
}