package com.swrobotics.lib.pathfinding;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.Arrays;

/**
 * Polygonal pathfinding obstacle
 */
@JsonIgnoreProperties(ignoreUnknown = true)
@JsonAutoDetect(getterVisibility = JsonAutoDetect.Visibility.NONE)
public final class Polygon extends Obstacle {
    private final Translation2d[] vertices;

    /**
     * Creates a new polygon with the specified vertices. The polygon may be
     * concave, but must not contain any concave regions the robot does not fit
     * inside. If the vertices are in counterclockwise order, the path will not
     * be allowed to pass inside it. If they are in clockwise order, the path
     * will not be allowed to pass <i>outside</i> it, which is useful for the
     * field perimeter.
     *
     * @param vertices vertices of the polygon
     */
    @JsonCreator
    public Polygon(
            @JsonProperty(required = true, value = "vertices") Translation2d... vertices) {
        this.vertices = vertices;
    }

    public Translation2d[] getVertices() {
        return vertices;
    }

    @Override
    void addToJNIObstacleList(long obsHandle) {
        double[] coords = new double[vertices.length * 2];
        for (int i = 0; i < vertices.length; i++) {
            Translation2d vertex = vertices[i];
            coords[i * 2] = vertex.getX();
            coords[i * 2 + 1] = vertex.getY();
        }
        PathfindingJNI.addPolygon(obsHandle, coords);
    }

    @Override
    public String toString() {
        return "Polygon{" +
                "vertices=" + Arrays.toString(vertices) +
                '}';
    }
}
