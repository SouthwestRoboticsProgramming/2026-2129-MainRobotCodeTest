package com.swrobotics.lib.net;

import com.swrobotics.robot.Robot;

import java.util.ArrayList;
import java.util.function.Supplier;

public abstract class NTEntry<T> implements Supplier<T> {
    private final ArrayList<Runnable> changeListeners;
    private boolean hasSetChangeListener;

    public NTEntry() {
        changeListeners = new ArrayList<>();
        hasSetChangeListener = false;
    }

    public abstract void set(T value);

    public abstract NTEntry<T> setPersistent();

    public abstract void registerChangeListeners(Runnable fireFn);

    public void onChange(Runnable listener) {
        if (!hasSetChangeListener) {
            registerChangeListeners(this::fireOnChanged);
            hasSetChangeListener = true;
        }

        changeListeners.add(listener);
    }

    public void nowAndOnChange(Runnable listener) {
        listener.run();
        onChange(listener);
    }

    private void fireOnChanged() {
        for (Runnable listener : changeListeners) {
            Robot.runOnMainThread(listener);
        }
    }
}
