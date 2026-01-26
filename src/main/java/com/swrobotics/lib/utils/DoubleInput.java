package com.swrobotics.lib.utils;

/**
 * Helper to read a double array sequentially, from beginning to end.
 */
public final class DoubleInput {
    private final double[] data;
    private int index;

    /**
     * @param data double array to read
     */
    public DoubleInput(double[] data) {
        this.data = data;
        this.index = 0;
    }

    /**
     * Gets the next value in the array.
     * @return next value
     * @throws ArrayIndexOutOfBoundsException if the end of the array has been reached
     */
    public double next() {
        return data[index++];
    }

    /**
     * Gets whether there are values left in the array. If this returns true,
     * {@link #next()} is guaranteed to not throw.
     *
     * @return true if the end has not been reached, false otherwise
     */
    public boolean hasNext() {
        return index < data.length;
    }
}
