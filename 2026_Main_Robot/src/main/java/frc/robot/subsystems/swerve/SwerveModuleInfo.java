package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.IOAllocation;
import frc.robot.nt.NTEntry;

public record SwerveModuleInfo(
        String canBus,
        int driveId, int turnId, int encoderId,
        Translation2d position,
        NTEntry<Double> offset,
        String name) {
    public SwerveModuleInfo(IOAllocation.SwerveIDs ids, double x, double y, NTEntry<Double> offset, String name) {
        this(
                ids.drive.bus(),
                ids.drive.id(), ids.turn.id(), ids.encoder.id(),
                new Translation2d(x, y),
                offset,
                name);
    }
}
