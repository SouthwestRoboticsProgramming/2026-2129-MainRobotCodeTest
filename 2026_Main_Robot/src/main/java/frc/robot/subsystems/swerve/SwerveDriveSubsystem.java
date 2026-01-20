package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldView;
import frc.robot.lib.nt.NTBoolean;
import frc.robot.lib.nt.NTEntry;

public class SwerveDriveSubsystem extends SubsystemBase {
    private static final NTBoolean CALIBRATE = new NTBoolean("Drive/Modules/Calibrate", false);
    
    SwerveDrivetrain<TalonFX, TalonFX, CANcoder> driveTrain;
    SwerveDriveState currState;
    public SwerveDriveSubsystem () {
        int kModuleCount = Constants.kSwerveModuleInfos.length;
        var moduleConstants = new SwerveModuleConstants[kModuleCount];
        for (int i = 0; i < kModuleCount; i++) {
            SwerveModuleInfo info = Constants.kSwerveModuleInfos[i];
            moduleConstants[i] = Constants.kModuleConstantsFactory.createModuleConstants(
                    info.turnId(),
                    info.driveId(),
                    info.encoderId(),
                    info.offset().get(),
                    info.position().getX(),
                    info.position().getY(),
                    false,
                    true,
                    false
            );
        }

        driveTrain = new SwerveDrivetrain<>(
                TalonFX::new, TalonFX::new, CANcoder::new,
                Constants.kDrivetrainConstants,
                Constants.kOdometryUpdateFreq,
                Constants.kOdometryStdDevs,
                // These values have no effect, they are overridden by the                                            
                // standard deviations given to drivetrain.addVisionMeasurement()                                     
                VecBuilder.fill(0.6, 0.6, 0.6),
                moduleConstants
        );
        for (int i = 0; i < kModuleCount; i++) {
            SwerveModule<TalonFX, TalonFX, CANcoder> module = driveTrain.getModule(i);
            
            CurrentLimitsConfigs driveLimits = new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Constants.kDriveStatorCurrentLimit)
                    .withSupplyCurrentLowerLimit(Constants.kDriveSupplyCurrentLimit)
                    .withSupplyCurrentLowerTime(Constants.kDriveCurrentLimitTime);
            module.getDriveMotor().getConfigurator().apply(driveLimits);
        }
        // Set operator perspective to field +X axis (0 degrees) so coordinate                                        
        // system stays centered on blue alliance origin                                                              
        driveTrain.setOperatorPerspectiveForward(new Rotation2d(0));
        currState = driveTrain.getState();
    }

    public Pose2d getEstimatedPose(){
        return currState.Pose;
    }
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return currState.Speeds;
    }
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix <N3, N1> stdDevs ){
        driveTrain.addVisionMeasurement(pose, Utils.fpgaToCurrentTime(timestamp), stdDevs);
    }

    public void calibrateModuleOffsets() {
        SwerveModule<TalonFX, TalonFX, CANcoder>[] modules = driveTrain.getModules();
        for (int i = 0; i < modules.length; i++) {
            CANcoder canCoder = modules[i].getEncoder();
            NTEntry<Double> offset = Constants.kSwerveModuleInfos[i].offset();

            StatusSignal<Angle> canCoderPos = canCoder.getAbsolutePosition();
            canCoderPos.waitForUpdate(1);
            double position = canCoderPos.getValueAsDouble();

            offset.set(offset.get() - position);
            canCoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(offset.get()));
        }
    }
    public void setControl(SwerveRequest request) {
        driveTrain.setControl(request);
    }
    public Command commandSetControl(Supplier<SwerveRequest> requestSupplier) {
        return Commands.run(() -> setControl(requestSupplier.get()));
    }
    @Override
    public void periodic() {
        currState = driveTrain.getState();

        FieldView.robotPose.setPose(currState.Pose);

        if (CALIBRATE.get()) {
            CALIBRATE.set(false);
            calibrateModuleOffsets();
        }
    }
    public void resetRotation(Rotation2d r2d) {
        driveTrain.resetRotation(r2d);        
    }
}
