package com.swrobotics.robot.subsystems.indexer;


import com.swrobotics.robot.logging.AutoLoggedInputs;

public interface IndexerIO {
    class Inputs extends AutoLoggedInputs {
        public double voltage;

    }

    void updateInputs(Inputs inputs);

    void setVoltage(double voltage);

}