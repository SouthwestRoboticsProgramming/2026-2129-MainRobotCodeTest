package frc.robot.nt;

public class NTBoolean extends NTPrimitive<Boolean> {

    private final boolean defaultVal;


    public NTBoolean(String path, boolean defaultVal) {
        super(path, defaultVal);
        this.defaultVal = defaultVal;
    }

    @Override
    public Boolean get() {
        return entry.getBoolean(defaultVal);
    }

    @Override
    public void set(Boolean value) {
        entry.setBoolean(value);
    }
}
