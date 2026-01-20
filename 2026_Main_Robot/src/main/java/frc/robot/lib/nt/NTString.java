package frc.robot.lib.nt;

public final class NTString extends NTPrimitive<String> {
    private final String defaultVal;

    public NTString(String path, String defaultVal) {
        super(path, defaultVal);
        this.defaultVal = defaultVal;
    }

    @Override
    public String get() {
        return entry.getString(defaultVal);
    }

    @Override
    public void set(String value) {
        entry.setString(value);
    }
}
