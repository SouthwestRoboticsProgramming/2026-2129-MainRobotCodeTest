package com.swrobotics.lib.pathfinding;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Circular pathfinding obstacle
 */
@JsonIgnoreProperties(ignoreUnknown = true)
@JsonAutoDetect(getterVisibility = JsonAutoDetect.Visibility.NONE)
public final class Circle extends Obstacle {
    private final Translation2d center;
    private final double radius;

    /**
     * @param center center point of the circle
     * @param radius radius of the circle
     */
    @JsonCreator
    public Circle(
            @JsonProperty(required = true, value = "center") Translation2d center,
            @JsonProperty(required = true, value = "radius") double radius) {
        this.center = center;
        this.radius = radius;
    }

    public Translation2d getCenter() {
        return center;
    }

    public double getRadius() {
        return radius;
    }

    @Override
    void addToJNIObstacleList(long obsHandle) {
        PathfindingJNI.addCircle(obsHandle, center.getX(), center.getY(), radius);
    }

    @Override
    public String toString() {
        return "Circle{" +
                "center=" + center +
                ", radius=" + radius +
                '}';
    }
}
