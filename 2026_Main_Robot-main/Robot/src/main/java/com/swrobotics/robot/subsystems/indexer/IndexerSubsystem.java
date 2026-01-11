//Intake subsystem for the robot

package com.swrobotics.robot.subsystems.indexer;

// Imports of Littleton Robotics Junction

import org.littletonrobotics.junction.Logger;

// Imports of CTRE Phoenix 6

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// Imports of SW Robotics Library and Robot Code

import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.subsystems.motortracker.MotorTrackerSubsystem;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;

// Imports of WPILib

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Intake subsystem for the robot class

public class IndexerSubsystem extends SubsystemBase {

    // Target state of the indexer
    
    public enum State{
        IDLE,
        INTAKE
        
    }
    
    // Target state variable

    private final TalonFX motor;
    private final VoltageOut voltageOut = new VoltageOut(0);
    private State targetState;

    //Intake config + rain cool stuff

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
    
    
    
    public void setTargetState(State targetState) {
    
        this.targetState = targetState;
    }



    public Command commandSetState(State targetState) {



        return Commands.run(() -> setTargetState(targetState), this);
    }



    public Command Idle() {
        return commandSetState(State.IDLE);
    }



    public Command Intake() {
        return commandSetState(State.INTAKE);
    }

    

    @Override
    public void periodic() {
        switch (targetState) {
            case IDLE -> IndexerIO.setVoltage(kIndexerIdleVoltage.get());
                break;
            case INTAKE -> IndexerIO.setVoltage(Constants.kIndexerIntakeVoltage.get());
                break;
        }
    }
}