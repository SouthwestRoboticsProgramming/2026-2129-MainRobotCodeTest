package com.swrobotics.robot.logging;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * View of the field and various objects on it for SmartDashboard. Used in both
 * simulation and the real robot.
 */
public final class FieldView {
    private static final Field2d field = new Field2d();
    private static final Field2d left = new Field2d();
    private static final Field2d right = new Field2d();

    public static final FieldObject2d robotPose = field.getRobotObject();
    public static final FieldObject2d visionEstimates = field.getObject("Vision estimates");
    public static final FieldObject2d pathPlannerPath = field.getObject("PathPlanner path");
    public static final FieldObject2d pathPlannerSetpoint = field.getObject("PathPlanner setpoint");

    /**
     * Adds the field view to SmartDashboard
     */
    public static void publish() {
        SmartDashboard.putData("Field View", field);
        SmartDashboard.putData("Left", left);
        SmartDashboard.putData("Right", right);
    }
}
