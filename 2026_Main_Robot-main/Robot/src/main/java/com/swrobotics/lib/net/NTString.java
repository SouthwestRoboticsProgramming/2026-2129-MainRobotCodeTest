package com.swrobotics.lib.net;

import edu.wpi.first.networktables.NetworkTableEntry;
import org.littletonrobotics.junction.LogTable;

public final class NTString extends NTEntry<String> {
    public NTString(String path, String defaultValue) {
        super(path, defaultValue);
    }

    @Override
    protected String getValue(NetworkTableEntry entry, String defaultValue) {
        return entry.getString(defaultValue);
    }

    @Override
    protected void setValue(NetworkTableEntry entry, String value) {
        entry.setString(value);
    }

    @Override
    protected void toLog(LogTable table, String key, String value) {
        table.put(key, value);
    }

    @Override
    protected String fromLog(LogTable table, String key, String defaultValue) {
        return table.get(key, defaultValue);
    }
}
