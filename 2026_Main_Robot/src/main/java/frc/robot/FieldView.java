package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldView {
    public static final Field2d field = new Field2d();

    public static final FieldObject2d robotPose = field.getRobotObject();
    public static final FieldObject2d visionEstimates = field.getObject("Vision estimates");

    public static void publish() {
        SmartDashboard.putData("Field View", field);
    }
}
