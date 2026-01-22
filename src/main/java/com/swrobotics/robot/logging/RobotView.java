package com.swrobotics.robot.logging;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

public final class RobotView {
    private static final LoggedMechanism2d mechanism = new LoggedMechanism2d(2, 3);

    public static void publish() {
        SmartDashboard.putData("Robot View", mechanism);
    }
}
