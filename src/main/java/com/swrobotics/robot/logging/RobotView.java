package com.swrobotics.robot.logging;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class RobotView {
    private static final Mechanism2d mechanism = new Mechanism2d(2, 3);

    public static void publish() {
        SmartDashboard.putData("Robot View", mechanism);
    }
}
