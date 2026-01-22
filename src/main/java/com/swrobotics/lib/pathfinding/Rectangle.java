package com.swrobotics.lib.pathfinding;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Rectangular pathfinding obstacle
 */
@JsonIgnoreProperties(ignoreUnknown = true)
@JsonAutoDetect(getterVisibility = JsonAutoDetect.Visibility.NONE)
public final class Rectangle extends Obstacle {
    private final Translation2d center;
    private final Translation2d size;
    private final double rotation;

    /**
     * @param center center point of the rectangle
     * @param size size of the rectangle, as (width, height)
     * @param rotation counterclockwise rotation of the rectangle about its
     *                 center in radians
     */
    @JsonCreator
    public Rectangle(
            @JsonProperty(required = true, value = "center") Translation2d center,
            @JsonProperty(required = true, value = "size") Translation2d size,
            @JsonProperty(required = true, value = "rotation") double rotation) {
        this.center = center;
        this.size = size;
        this.rotation = rotation;
    }

    public Translation2d getCenter() {
        return center;
    }

    public Translation2d getSize() {
        return size;
    }

    public double getRotation() {
        return rotation;
    }

    /**
     * Creates a polygon obstacle representing the same shape.
     *
     * @return matching polygon with vertices at the rectangle's corners
     */
    public Polygon asPolygon() {
        double halfW = size.getX() / 2;
        double halfH = size.getY() / 2;

        Rotation2d rotation = Rotation2d.fromRadians(this.rotation);

        Translation2d[] vertices = {
                center.plus(new Translation2d(-halfW, -halfH).rotateBy(rotation)),
                center.plus(new Translation2d( halfW, -halfH).rotateBy(rotation)),
                center.plus(new Translation2d( halfW,  halfH).rotateBy(rotation)),
                center.plus(new Translation2d(-halfW,  halfH).rotateBy(rotation))
        };
        return new Polygon(vertices);
    }

    @Override
    void addToJNIObstacleList(long obsHandle) {
        asPolygon().addToJNIObstacleList(obsHandle);
    }

    @Override
    public String toString() {
        return "Rectangle{" +
                "center=" + center +
                ", size=" + size +
                ", rotation=" + rotation +
                '}';
    }
}
