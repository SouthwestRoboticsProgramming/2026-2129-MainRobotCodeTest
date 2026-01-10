package com.swrobotics.lib.net;

import edu.wpi.first.networktables.NetworkTableEntry;
import org.littletonrobotics.junction.LogTable;

/**
 * An integer value stored in NetworkTables. Actually stored as a double, since
 * integer entries are somewhat broken.
 */
// TODO: Actually store as integer entry once WPILib fixes integer entries
public final class NTInteger extends NTEntry<Integer> {
    public NTInteger(String path, int defaultValue) {
        super(path, defaultValue);
    }

    @Override
    protected Integer getValue(NetworkTableEntry entry, Integer defaultValue) {
        return (int) entry.getDouble(defaultValue);
    }

    @Override
    protected void setValue(NetworkTableEntry entry, Integer value) {
        entry.setDouble(value);
    }

    @Override
    protected void toLog(LogTable table, String key, Integer value) {
        table.put(key, value.doubleValue());
    }

    @Override
    protected Integer fromLog(LogTable table, String key, Integer defaultValue) {
        return (int) table.get(key, defaultValue.doubleValue());
    }
}
