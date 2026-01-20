// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.expansion.ExpansionSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  public final SwerveDriveSubsystem drive;
  public final ShooterSubsystem shooter;
  public final IndexerSubsystem indexer;
  public final IntakeSubsystem intake;
  public final ExpansionSubsystem expansion;
  public final VisionSubsystem vision;
  public final ClimberSubsystem climb;
  public final HoodSubsystem hood;

  private final CommandXboxController driver;
  private final CommandXboxController operator; 
  private final DriveAccelFilter driveControlFilter;

  public RobotContainer() {
    drive = new SwerveDriveSubsystem();
    shooter = new ShooterSubsystem();
    indexer = new IndexerSubsystem();
    intake = new IntakeSubsystem();
    expansion = new ExpansionSubsystem();
    vision = new VisionSubsystem(drive);
    climb = new ClimberSubsystem();
    hood = new HoodSubsystem();
    driver = new CommandXboxController(0);
    operator = new CommandXboxController(1);

    FieldView.publish();
    configureBindings();
  }

  private Translation2d getDriveTranslation() {
    return getDesiredDriveTranslation();
    
  }

private Translation2d getDesiredDriveTranslation() {
        double maxSpeed = Constants.kDriveMaxAchievableSpeed;

        Translation2d leftStick = new Translation2d(driver.getLeftX(), driver.getLeftY());

        // Apply an exponential curve to the driver's input. This allows the
        // driver to have slower, more precise movement in the center of the
        // stick, while still having high speed movement towards the edges.
        double rawMag = leftStick.getNorm();
        double powerMag = SWMathUtil.powerWithSign(rawMag, Constants.kDriveControlDrivePower);

        // Prevent division by zero, which would result in a target velocity of
        // (NaN, NaN), which motor controllers do not like
        if (rawMag == 0 || powerMag == 0)
            return new Translation2d(0, 0);

        double targetSpeed = powerMag * maxSpeed;
        double filteredSpeed = driveControlFilter.calculate(targetSpeed);
        return new Translation2d(-leftStick.getY(), -leftStick.getX())
                .div(rawMag) // Normalize translation
                .times(filteredSpeed) // Apply new speed
                .rotateBy(FieldInfo.getAllianceForwardAngle()); // Account for driver's perspective
    }

  /**
  * @return radians per second input for the drive base
  */
  private double getDriveRotation() {
    double input = SWMathUtil.powerWithSign(SWMathUtil.deadband(-driver.getRightX(), Constants.kDeadband), Constants.kDriveControlTurnPower);
    return Units.rotationsToRadians(input * Constants.kDriveControlMaxTurnSpeed);
  }

  private void configureBindings() {
    // Gyro reset buttons
    driver.start().onFalse(Commands.run(() -> drive.resetRotation(new Rotation2d())));
    driver.back().onFalse(Commands.run(() -> drive.resetRotation(new Rotation2d()))); // Two buttons to reset gyro so the driver can't get confused
    drive.setDefaultCommand(DriveCommands.driveFieldRelative(
                drive,
                this::getDriveTranslation,
                this::getDriveRotation
        ));
        
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
