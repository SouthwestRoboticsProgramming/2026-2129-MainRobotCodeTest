package com.swrobotics.lib.utils;

import edu.wpi.first.math.geometry.Pose3d;

@FunctionalInterface
public interface Transformation3d {
    Pose3d transform(Pose3d pose);
}
