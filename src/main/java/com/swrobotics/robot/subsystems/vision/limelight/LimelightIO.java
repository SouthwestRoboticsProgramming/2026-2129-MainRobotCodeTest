package com.swrobotics.robot.subsystems.vision.limelight;

public interface LimelightIO {
    final class EstimateInputs {
        public long timestamp;
        public double[] data;
    }

    final class Inputs {
        public final EstimateInputs megaTag1 = new EstimateInputs();
        public final EstimateInputs megaTag2 = new EstimateInputs();
    }

    void updateInputs(Inputs inputs);

    // Both in degrees CCW
    void updateRobotState(double yawAngle, double yawRate);
}
