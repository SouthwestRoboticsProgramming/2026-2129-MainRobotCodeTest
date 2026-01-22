package com.swrobotics.robot.subsystems.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface SwerveIO {
    final class Inputs {
        public Pose2d estPose;
        public ChassisSpeeds robotRelSpeeds;
        public SwerveModuleState[] moduleStates;
        public SwerveModuleState[] moduleTargets;
        public SwerveModulePosition[] modulePositions;
        public double odometryPeriod;
        public int successfulDaqs;
        public int failedDaqs;

        public Rotation2d rawGyroRotation;
    }

    void updateInputs(Inputs inputs);

    void setControl(SwerveRequest request);

    void resetPose(Pose2d pose);

    void resetRotation(Rotation2d rotation);

    void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);

    void calibrateModuleOffsets();
}
