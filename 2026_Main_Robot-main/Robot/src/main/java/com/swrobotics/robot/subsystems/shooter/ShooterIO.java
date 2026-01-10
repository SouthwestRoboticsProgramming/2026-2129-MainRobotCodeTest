package com.swrobotics.robot.subsystems.shooter;

import com.swrobotics.robot.logging.AutoLoggedInputs;

public interface ShooterIO {
    class Inputs extends AutoLoggedInputs {
        public double voltage;
    }

    void updateInputs(Inputs inputs);

    void setVoltage(double voltage);
}
