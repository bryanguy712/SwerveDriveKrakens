package frc.robot.moprefs;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableValue;
import java.util.EnumSet;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;

public class Pref<T> {
    public final String key;
    private Function<NetworkTableValue, T> getter;
    private BiFunction<NetworkTableEntry, T, Boolean> setter;

    private final NetworkTableEntry entry;

    private Consumer<T> subscriber = null;

    public Pref(
            String key,
            T defaultValue,
            Function<NetworkTableValue, T> getter,
            BiFunction<NetworkTableEntry, T, Boolean> setter) {
        if (key.contains("/")) {
            throw new IllegalArgumentException("Pref keys must not contain '/'");
        }

        this.key = key;
        this.getter = getter;
        this.setter = setter;

        this.entry = MoPrefs.getInstance().backingTable.getEntry(key);
        this.entry.setDefaultValue(defaultValue);
        this.entry.setPersistent();
    }

    public T get() {
        return getter.apply(entry.getValue());
    }

    public void set(T value) {
        setter.apply(entry, value);
    }

    public void subscribe(Consumer<T> consumer) {
        subscribe(consumer, false);
    }

    public void subscribe(Consumer<T> consumer, boolean notifyImmediately) {
        if (subscriber != null) {
            subscriber = subscriber.andThen(consumer);
        } else {
            subscriber = consumer;
            entry.getInstance()
                    .addListener(
                            entry,
                            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                            (e) -> consumer.accept(getter.apply(e.valueData.value)));
        }

        if (notifyImmediately) {
            consumer.accept(this.get());
        }
    }

    public String getKey() {
        return key;
    }
}
