package com.swrobotics.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.lib.utils.PolynomialRegression;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.subsystems.lights.LightsSubsystem;
import com.swrobotics.robot.subsystems.swerve.SwerveDriveSubsystem;
import com.swrobotics.robot.subsystems.vision.limelight.LimelightCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public final class DriveCommands {
    public static Command driveRobotRelative(
            SwerveDriveSubsystem drive,
            Supplier<Translation2d> translationSupplier, // m/s
            Supplier<Double> rotationSupplier // rad/s
    ) {
        return Commands.run(() -> {
            Translation2d tx = translationSupplier.get();
            double rot = rotationSupplier.get();

            drive.setControl(new SwerveRequest.RobotCentric()
                    .withVelocityX(tx.getX())
                    .withVelocityY(tx.getY())
                    .withRotationalRate(rot));
        }, drive);
    }

    public static Command driveFieldRelative(
            SwerveDriveSubsystem drive,
            Supplier<Translation2d> translationSupplier, // m/s
            Supplier<Double> rotationSupplier // rad/s
    ) {
        return Commands.run(() -> {
            Translation2d tx = translationSupplier.get();
            double rot = rotationSupplier.get();

            drive.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(tx.getX())
                    .withVelocityY(tx.getY())
                    .withRotationalRate(rot));
        }, drive);
    }

    public static Command driveFieldRelativeSnapToAngle(
            SwerveDriveSubsystem drive,
            LightsSubsystem lights,
            Supplier<Translation2d> translationSupplier,
            Supplier<Rotation2d> targetRotationSupplier) {
        PIDController turnPid = new PIDController(0, 0, 0);
        turnPid.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.startRun(() -> {
            turnPid.setPID(Constants.kSnapTurnKp.get(), 0, Constants.kSnapTurnKd.get());
            turnPid.reset();
        }, () -> {
            Translation2d tx = translationSupplier.get();

            Rotation2d currentRot = drive.getEstimatedPose().getRotation();
            Rotation2d targetRot = targetRotationSupplier.get();
            double rotOutput = turnPid.calculate(
                    MathUtil.wrap(currentRot.getRadians(), -Math.PI, Math.PI),
                    MathUtil.wrap(targetRot.getRadians(), -Math.PI, Math.PI)
            );

            double maxTurnSpeed = Units.rotationsToRadians(Constants.kSnapMaxTurnSpeed.get());
            rotOutput = MathUtil.clamp(rotOutput, -maxTurnSpeed, maxTurnSpeed);

            drive.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(tx.getX())
                    .withVelocityY(tx.getY())
                    .withRotationalRate(rotOutput));
        }, drive)
                .raceWith(LightCommands.showSnappingToAngle(lights));
    }

    public static Command snapToPose(SwerveDriveSubsystem drive, LightsSubsystem lights, Supplier<Pose2d> targetPoseSupplier) {
        PIDController driveXPid = new PIDController(0, 0, 0);
        PIDController driveYPid = new PIDController(0, 0, 0);
        PIDController turnPid = new PIDController(0, 0, 0);
        turnPid.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.startRun(() -> {
            // Update PIDs in case we tuned them since last time
            driveXPid.setPID(Constants.kSnapDriveKp.get(), 0, Constants.kSnapDriveKd.get());
            driveYPid.setPID(Constants.kSnapDriveKp.get(), 0, Constants.kSnapDriveKd.get());
            turnPid.setPID(Constants.kSnapTurnKp.get(), 0, Constants.kSnapTurnKd.get());

            driveXPid.reset();
            driveYPid.reset();
            turnPid.reset();
        }, () -> {
            Pose2d currentPose = drive.getEstimatedPose();
            Pose2d targetPose = targetPoseSupplier.get();

            double xyError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
            double thetaError = MathUtil.absDiffRad(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

            double xOutput = 0, yOutput = 0;
            if (xyError > Constants.kSnapXYDeadzone.get()) {
                xOutput = driveXPid.calculate(currentPose.getX(), targetPose.getX());
                yOutput = driveYPid.calculate(currentPose.getY(), targetPose.getY());
            }
            double rotOutput = 0;
            if (thetaError > Math.toRadians(Constants.kSnapThetaDeadzone.get())) {
                rotOutput = turnPid.calculate(
                        MathUtil.wrap(currentPose.getRotation().getRadians(), -Math.PI, Math.PI),
                        MathUtil.wrap(targetPose.getRotation().getRadians(), -Math.PI, Math.PI)
                );
            }

            double maxDriveSpeed = Constants.kSnapMaxSpeed.get();
            double driveSpeed = Math.hypot(xOutput, yOutput);
            if (driveSpeed > maxDriveSpeed) {
                double scale = maxDriveSpeed / driveSpeed;
                xOutput *= scale;
                yOutput *= scale;
            }

            double maxTurnSpeed = Units.rotationsToRadians(Constants.kSnapMaxTurnSpeed.get());
            rotOutput = MathUtil.clamp(rotOutput, -maxTurnSpeed, maxTurnSpeed);

            drive.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(xOutput)
                    .withVelocityY(yOutput)
                    .withRotationalRate(rotOutput));
        }, drive)
                .raceWith(LightCommands.showSnappingToPose(lights));
    }

    // Updated DriveCommands.java - EXACT snapToPose clone with arc logic:
public static Command autoalignArcToHub(SwerveDriveSubsystem drive, LightsSubsystem lights) {
    PIDController driveXPid = new PIDController(0, 0, 0);
    PIDController driveYPid = new PIDController(0, 0, 0);
    PIDController turnPid = new PIDController(0, 0, 0);
    turnPid.enableContinuousInput(-Math.PI, Math.PI);

    Pose2d hubPose = new Pose2d(8.013, 7.937, new Rotation2d()); // 2026 Red Hub

    return Commands.startRun(() -> {
        // IDENTICAL snapToPose PID setup
        driveXPid.setPID(Constants.kArcAlignDriveKp.get(), 0, Constants.kArcAlignDriveKd.get());
        driveYPid.setPID(Constants.kArcAlignDriveKp.get(), 0, Constants.kArcAlignDriveKd.get());
        turnPid.setPID(Constants.kArcAlignTurnKp.get(), 0, Constants.kArcAlignTurnKd.get());

        driveXPid.reset(); driveYPid.reset(); turnPid.reset();
    }, () -> {
        Pose2d currentPose = drive.getEstimatedPose();
        
        // **ENGAGE DISTANCE CHECK** - Only run when close enough to hub
        Translation2d toHub = hubPose.getTranslation().minus(currentPose.getTranslation());
        double distanceToHub = toHub.getNorm();
        
        if (distanceToHub > Constants.kArcAlignEngageDistance.get()) {
            // Too far - stop
            drive.setControl(new SwerveRequest.FieldCentric()
                .withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            return;
        }
        
        // **ARC SEGMENT MATH** - Find closest point on π radian arc
        double radius = Constants.kArcAlignRadius.get();
        double angleOffsetDeg = Constants.kArcAlignAngleOffset.get();
        double angleOffsetRad = Math.toRadians(angleOffsetDeg);
        
        // Robot's approach angle defines arc center
        double robotAngle = Math.atan2(toHub.getY(), toHub.getX());
        
        // Limit to π/x radians total arc 
        double arcHalfAngle = Math.toRadians(80);
        double minAngle = robotAngle - arcHalfAngle + angleOffsetRad;
        double maxAngle = robotAngle + arcHalfAngle + angleOffsetRad;
        
        // Clamp to arc segment (nearest valid point)
        double targetAngle = MathUtil.clamp(robotAngle + angleOffsetRad, minAngle, maxAngle);
        
        // Target position on arc
        Translation2d targetPos = new Translation2d(
            hubPose.getX() + radius * Math.cos(targetAngle),
            hubPose.getY() + radius * Math.sin(targetAngle)
        );
        
        // Always face hub center
        Translation2d toHubFromTarget = hubPose.getTranslation().minus(targetPos);
        Rotation2d targetRot = toHubFromTarget.getAngle();

        
        // **IDENTICAL snapToPose logic from here ↓**
        double xyError = currentPose.getTranslation().getDistance(targetPos);
        double thetaError = MathUtil.absDiffRad(
            currentPose.getRotation().getRadians(), targetRot.getRadians()
        );

        double xOutput = 0, yOutput = 0;
        if (xyError > Constants.kArcAlignXYDeadzone.get()) {
            xOutput = driveXPid.calculate(currentPose.getX(), targetPos.getX());
            yOutput = driveYPid.calculate(currentPose.getY(), targetPos.getY());
        }
        
        double rotOutput = 0;
        if (thetaError > Math.toRadians(Constants.kArcAlignThetaDeadzone.get())) {
            rotOutput = turnPid.calculate(
                MathUtil.wrap(currentPose.getRotation().getRadians(), -Math.PI, Math.PI),
                MathUtil.wrap(targetRot.getRadians(), -Math.PI, Math.PI)
            );
        }

        // IDENTICAL speed limiting
        double maxDriveSpeed = Constants.kArcAlignMaxSpeed.get();
        double driveSpeed = Math.hypot(xOutput, yOutput);
        if (driveSpeed > maxDriveSpeed) {
            double scale = maxDriveSpeed / driveSpeed;
            xOutput *= scale; yOutput *= scale;
        }

        double maxTurnSpeed = Units.rotationsToRadians(Constants.kArcAlignMaxTurnSpeed.get());
        rotOutput = MathUtil.clamp(rotOutput, -maxTurnSpeed, maxTurnSpeed);

        drive.setControl(new SwerveRequest.FieldCentric()
            .withVelocityX(xOutput)
            .withVelocityY(yOutput)
            .withRotationalRate(rotOutput));
    }, drive)
    .raceWith(LightCommands.showArcAligning(lights));
}


    public static Command snapToPoseUntilInTolerance(
            SwerveDriveSubsystem drive,
            LightsSubsystem lights,
            Supplier<Pose2d> poseSupplier,
            Supplier<Double> toleranceSupplier) {
        return snapToPose(drive, lights, poseSupplier)
                .until(() -> drive.isCloseTo(
                        poseSupplier.get().getTranslation(),
                        toleranceSupplier.get()
                ));
    }

    // public static Command autoalignToHub(SwerveDriveSubsystem drive, LightsSubsystem lights, LimelightCamera frontLeftLimelight, LimelightCamera frontRightLimelight, LimelightCamera backLimelight, int hubTagId /*  AprilTag ID for 2026 Rebuilt hub*/){
        
    // }TODO: Implement autoalign to hub command using distance constant

    public static Command feedforwardCharacterization(SwerveDriveSubsystem drive) {
        List<Double> velocitySamples = new ArrayList<>();
        List<Double> voltageSamples = new ArrayList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                Commands.runOnce(() -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),

                // Allow modules to orient forward
                Commands.run(() -> drive.setControl(new SwerveRequest.SysIdSwerveTranslation()
                        .withVolts(0)), drive)
                        .withTimeout(1),

                Commands.runOnce(timer::restart),

                Commands.run(() -> {
                    double voltage = timer.get() * 0.1;

                    drive.setControl(new SwerveRequest.SysIdSwerveTranslation()
                            .withVolts(voltage));

                    // Convert units to rotor rotations, which is what CTRE wants
                    double metersPerSec = drive.getFFCharacterizationVelocity();
                    double wheelRadPerSec = metersPerSec / Constants.kModuleConstantsFactory.WheelRadius;
                    double wheelRotPerSec = Units.radiansToRotations(wheelRadPerSec);
                    double rotorRotPerSec = wheelRotPerSec * Constants.kModuleConstantsFactory.DriveMotorGearRatio;

                    velocitySamples.add(rotorRotPerSec);
                    voltageSamples.add(voltage);
                }, drive).finallyDo(() -> {
                    if (velocitySamples.size() < 2 || voltageSamples.size() < 2)
                        return;

                    double[] velocityArray = new double[velocitySamples.size()];
                    for (int i = 0; i < velocityArray.length; i++) {
                        velocityArray[i] = velocitySamples.get(i);
                    }

                    double[] voltageArray = new double[voltageSamples.size()];
                    for (int i = 0; i < voltageArray.length; i++) {
                        voltageArray[i] = voltageSamples.get(i);
                    }

                    PolynomialRegression regression = new PolynomialRegression(velocityArray, voltageArray, 1);
                    double kS = regression.beta(0); // y-intercept
                    double kV = regression.beta(1); // slope
                    double R2 = regression.R2();

                    System.out.println("FF Characterization Results:");
                    System.out.printf("\tR2=%.5f%n", R2);
                    System.out.printf("\tkS=%.5f%n", kS);
                    System.out.printf("\tkV=%.5f%n", kV);

                    Logger.recordOutput("Drive/FF Characterization/kS", kS);
                    Logger.recordOutput("Drive/FF Characterization/kV", kV);
                    Logger.recordOutput("Drive/FF Characterization/R2", R2);
                })
        );
    }
}
