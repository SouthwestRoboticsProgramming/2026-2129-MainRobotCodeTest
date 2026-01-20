package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.lib.nt.NTDouble;
import frc.robot.lib.nt.NTEntry;
import frc.robot.subsystems.swerve.SwerveModuleInfo;
import frc.robot.subsystems.vision.limelight.LimelightCamera;

import static edu.wpi.first.units.Units.*;

public class Constants {
    /* --- Expansion ---  */ 
    public static final NTEntry<Double> kExpansionRetractedRotations = new NTDouble("Expansion/Retracted Rotations", 0.0).setPersistent();
    public static final NTEntry<Double> kExpansionExtendedRotations  = new NTDouble("Expansion/Extended Rotations", 20.0).setPersistent();
    public static final NTEntry<Double> kExpansionCruiseVelocity = new NTDouble("Expansion/Cruise Velocity", 40.0).setPersistent();   
    public static final NTEntry<Double> kExpansionAcceleration  = new NTDouble("Expansion/Acceleration", 160.0).setPersistent();
    
    /* --- Hood --- */ //TODO: Adjust constant values
    public static final NTEntry<Double> kHoodIdleRotations = new NTDouble("Hood/Target Rotations", 0.0).setPersistent();
    public static final NTEntry<Double> kHoodMovingRotations = new NTDouble("Hood/Retracted Rotations", 2.0).setPersistent();
    public static final NTEntry<Double> kHoodReadyRotations  = new NTDouble("Hood/Extended Rotations", 0.0).setPersistent();
    public static final NTEntry<Double> kHoodCruiseVelocity = new NTDouble("Hood/Cruise Velocity", 20.0).setPersistent();   
    public static final NTEntry<Double> kHoodAcceleration  = new NTDouble("Hood/Acceleration", 80.0).setPersistent();

    /* --- Indexer --- */
    public static final NTEntry<Double> kIndexerRollRPS = new NTDouble("Indexer/Intake RPS", 40.0).setPersistent();
    public static final NTEntry<Double> kIndexerIdleRPS = new NTDouble("Indexer/Idle RPS", 0.0).setPersistent();
    
    /* --- Intake  ---  */ 
    public static final NTEntry<Double> kIntakeRPS = new NTDouble("Intake/Intake RPS", 40.0).setPersistent();
    public static final NTEntry<Double> kIntakeIdleRPS = new NTDouble("Intake/Idle RPS", 0.0).setPersistent();

    /* --- Shooter --- */
    public static final NTEntry<Double> kShooterRPS = new NTDouble("Shooter/Intake RPS", 100.0).setPersistent();
    public static final NTEntry<Double> kShooterIdleRPS = new NTDouble("Shooter/Idle RPS", 0.0).setPersistent();

    /* --- Vision --- */
    public static final double kVisionMT2SpeedThreshold = 0.2; // m/s
    // This will be different for each lens type, cameras with same lens should
    // have the same config
    public static final LimelightCamera.Config kLimelightConfig = new LimelightCamera.Config(
            // These were tuned at MURA using red alliance speaker AprilTags
            2,
            0.00197,
            0.002,
            0.00117
    );
    /* --- DRIVE --- */
    public static final double kDriveMaxAchievableSpeed = Units.feetToMeters(18.9); // m/s  TODO: Measure
    public static final NTEntry<Double> kFrontLeftOffset = new NTDouble("Drive/Modules/Front Left Offset (rot)", -0.33935546875).setPersistent();
    public static final NTEntry<Double> kFrontRightOffset = new NTDouble("Drive/Modules/Front Right Offset (rot)", 0.323486328125).setPersistent();
    public static final NTEntry<Double> kBackLeftOffset = new NTDouble("Drive/Modules/Back Left Offset (rot)", -0.320556640625).setPersistent();
    public static final NTEntry<Double> kBackRightOffset = new NTDouble("Drive/Modules/Back Right Offset (rot)", -0.367431640625).setPersistent();
    public static final double kDriveWheelSpacingX = 63.0 / 100; // m
    public static final double kDriveWheelSpacingY = 55.3 / 100; // m
    public static final SwerveModuleInfo[] kSwerveModuleInfos = {
            new SwerveModuleInfo(IOAllocation.CAN.kSwerveFL, kDriveWheelSpacingX / 2, kDriveWheelSpacingY / 2, Constants.kFrontLeftOffset, "Front Left"),
            new SwerveModuleInfo(IOAllocation.CAN.kSwerveFR, kDriveWheelSpacingX / 2, -kDriveWheelSpacingY / 2, Constants.kFrontRightOffset, "Front Right"),
            new SwerveModuleInfo(IOAllocation.CAN.kSwerveBL, -kDriveWheelSpacingX / 2, kDriveWheelSpacingY / 2, Constants.kBackLeftOffset, "Back Left"),
            new SwerveModuleInfo(IOAllocation.CAN.kSwerveBR, -kDriveWheelSpacingX / 2, -kDriveWheelSpacingY / 2, Constants.kBackRightOffset, "Back Right")
    };
    public static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kModuleConstantsFactory =
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                    .withDriveMotorGearRatio((50.0/16) * (16.0/28) * (45.0/15))
                    .withSteerMotorGearRatio(150.0 / 7)
                    .withCouplingGearRatio(50.0 / 16)
                    .withWheelRadius(Meters.of(0.0485603333))
                    .withSteerMotorGains(new Slot0Configs().withKP(50).withKD(0.01).withKV(0.1))
                    .withDriveMotorGains(new Slot0Configs().withKP(0.35).withKD(0).withKV(0.012621).withKS(0.22109))
                    .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                    .withSlipCurrent(Amps.of(80))
                    .withSpeedAt12Volts(MetersPerSecond.of(kDriveMaxAchievableSpeed))
                    .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                    .withDriveMotorInitialConfigs(new TalonFXConfiguration())
                    .withSteerMotorInitialConfigs(new TalonFXConfiguration())
                    .withEncoderInitialConfigs(new CANcoderConfiguration());
    static {
        if (RobotBase.isSimulation()) {
            kModuleConstantsFactory.DriveMotorGains
                    .withKV(0.12612)
                    .withKS(0.22510);
        }
    }
    public static final SwerveDrivetrainConstants kDrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(IOAllocation.CAN.kSwerveBus)
            .withPigeon2Id(IOAllocation.CAN.kJosh.id())
            .withPigeon2Configs(new Pigeon2Configuration());
    public static final double kOdometryUpdateFreq = 200; // Hz
    public static final Matrix<N3, N1> kOdometryStdDevs = VecBuilder.fill(0.005, 0.005, 0.001);
    public static final double kDriveStatorCurrentLimit = 60; // A
    public static final double kDriveSupplyCurrentLimit = 40; // A
    public static final double kDriveCurrentLimitTime = 0.25; // sec
    public static final double kDriveControlMaxTurnSpeed = 1; // rot/s
    public static final double kDriveControlTurnPower = 2;
    public static final double kDeadband = 0.15;
    public static final double kDriveControlDrivePower = 2; // Exponent input is raised to
}