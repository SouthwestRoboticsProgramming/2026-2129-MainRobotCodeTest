package com.swrobotics.lib.utils;

public final class DoubleOutput {
    private final double[] array;
    private int index;

    public DoubleOutput(int capacity) {
        array = new double[capacity];
        index = 0;
    }

    public void add(double value) {
        array[index++] = value;
    }

    public double[] toArray() {
        return array;
    }
}
