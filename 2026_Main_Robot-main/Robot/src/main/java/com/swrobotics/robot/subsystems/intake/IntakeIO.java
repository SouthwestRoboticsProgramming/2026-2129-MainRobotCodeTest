package com.swrobotics.robot.subsystems.intake;


import com.swrobotics.robot.logging.AutoLoggedInputs;

public interface IntakeIO {
    class Inputs extends AutoLoggedInputs {
        public double voltage;

    }

    void updateInputs(Inputs inputs);

    void setVoltage(double voltage);

}
