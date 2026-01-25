package com.swrobotics.robot.subsystems.turret;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
    public enum State {
        GO_TO_START,
        AUTO_TRACKING,
        MANUAL_MODE
    }

    private final TalonFX motor;
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);
    private State targetState = State.GO_TO_START;
    private double manualTargetRotations = 0.0;
    private double currentTurretAngle = 0.0; // degrees
    private Pose2d robotPose;
    private Rotation2d hubAngle;

    public TurretSubsystem() {
        motor = IOAllocation.CAN.kTurretMotor.createTalonFX();

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = Constants.kTurretInverted.get() ? 
            InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Improved PID gains (tuned for turret)
        Slot0Configs gains = new Slot0Configs();
        gains.withKP(12.0);  // Slightly higher for better tracking
        gains.withKI(0.0);
        gains.withKD(0.8);   // Increased damping
        gains.withKV(0.1);
        config.Slot0 = gains;

        // Enhanced Motion Magic configuration
        MotionMagicConfigs mmConfig = new MotionMagicConfigs();
        mmConfig.MotionMagicCruiseVelocity = Constants.kTurretCruiseVelocity.get();
        mmConfig.MotionMagicAcceleration = Constants.kTurretAcceleration.get();
        mmConfig.MotionMagicJerk = 2000.0;  // Critical: Smooth motion profile
        config.MotionMagic = mmConfig;

        config.apply(motor);
        motor.setPosition(0); // Zero at origin
    }

    @Override
    public void periodic() {
        currentTurretAngle = Math.toDegrees(motor.getPosition().getValueAsDouble());
        
        double maxAngle = Constants.kTurretMaxAngle.get();
        double minAngle = Constants.kTurretMinAngle.get();
        double maxRotations = maxAngle / 360.0;
        double minRotations = minAngle / 360.0;
        
        switch (targetState) {
            case GO_TO_START:
                manualTargetRotations = 0.0; // Home to center
                break;
                
            case AUTO_TRACKING:
                if (robotPose != null && hubAngle != null) {
                    double robotYaw = robotPose.getRotation().getDegrees();
                    double targetTurretAngle = hubAngle.minus(Rotation2d.fromDegrees(robotYaw)).getDegrees();
                    
                    // Normalize to [-180, 180] for shortest rotation
                    while (targetTurretAngle > 180) targetTurretAngle -= 360;
                    while (targetTurretAngle < -180) targetTurretAngle += 360;
                    
                    // Clamp in rotations (CORRECTED: clamp after normalization)
                    double normalizedAngle = targetTurretAngle / 360.0;
                    manualTargetRotations = Math.max(minRotations, Math.min(maxRotations, normalizedAngle));
                }
                break;
            
            case MANUAL_MODE:
                // Hold manual position
                manualTargetRotations = Math.max(minRotations, Math.min(maxRotations, manualTargetRotations));
                break;
        }
        
        // Apply limits to prevent hardware damage
        motor.setControl(motionMagic.withPosition(manualTargetRotations));
    }

    // Public API
    public void setTargetState(State state) { this.targetState = state; }
    public void setRobotPose(Pose2d pose) { this.robotPose = pose; }
    public void setHubAngle(Rotation2d hubAngle) { this.hubAngle = hubAngle; }
    public void setManualPosition(double rotations) { 
        this.manualTargetRotations = rotations; 
    }
    
    public State getCurrentState() { return targetState; }
    public double getCurrentAngleDegrees() { return currentTurretAngle; }

    // Enhanced Commands
    public Command commandSetState(State state) {
        return Commands.runOnce(() -> setTargetState(state), this);
    }

    public Command commandHome() {
        return Commands.runOnce(() -> {
            setTargetState(State.GO_TO_START);
        }, this);
    }

    public Command commandToggleAutoManual() {
        return Commands.runOnce(() -> {
            if (targetState == State.AUTO_TRACKING) {
                setTargetState(State.MANUAL_MODE);
            } else {
                setTargetState(State.AUTO_TRACKING);
            }
        }, this);
    }

    public Command commandManualLeft() {
        return Commands.runOnce(() -> {
            setTargetState(State.MANUAL_MODE);
            double newPos = Math.max(Constants.kTurretMinAngle.get() / 360.0, 
                                   (getCurrentAngleDegrees() - 5.0) / 360.0);
            setManualPosition(newPos);
        }, this);
    }

    public Command commandManualRight() {
        return Commands.runOnce(() -> {
            setTargetState(State.MANUAL_MODE);
            double newPos = Math.min(Constants.kTurretMaxAngle.get() / 360.0, 
                                   (getCurrentAngleDegrees() + 5.0) / 360.0);
            setManualPosition(newPos);
        }, this);
    }

    public Command commandManualJog(double degrees) {
        return Commands.runOnce(() -> {
            setTargetState(State.MANUAL_MODE);
            double newAngle = getCurrentAngleDegrees() + degrees;
            double newPos = Math.max(Constants.kTurretMinAngle.get() / 360.0,
                                   Math.min(Constants.kTurretMaxAngle.get() / 360.0,
                                          newAngle / 360.0));
            setManualPosition(newPos);
        }, this);
    }
}
