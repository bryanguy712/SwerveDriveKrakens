package frc.robot.moprefs;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.networktables.Topic;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.HashSet;
import java.util.Set;

/** Robot preferences, accessible through Shuffleboard */
public class MoPrefs {
    public static final Pref<Double> driveMaxSpeedMPS = unitlessDoublePref("Drive Max Speed meterspersecond", 5);
    public static final Pref<Double> turnMaxSpeedRPS = unitlessDoublePref("Turn Max Speed radiansPerSecond", Math.PI);

    public static final Pref<Double> autoAlignXP = unitlessDoublePref("Auto Align X P", 0);
    public static final Pref<Double> autoAlignYP = unitlessDoublePref("Auto Align Y P", 0);
    public static final Pref<Double> autoAlignRoationP = unitlessDoublePref("Auto Align Rotation P", 0);

    public static final Pref<Double> autoAlignXSetPoint = unitlessDoublePref("Auto Align X Setpoint", 0);
    public static final Pref<Double> autoAlignYSetpoint = unitlessDoublePref("Auto Align Y Setpoint", 0);
    public static final Pref<Double> autoAlignRotSetpoint = unitlessDoublePref("Auto Align Rotation Setpoint", 0);

    public static final Pref<Double> autoAlignXTolerance = unitlessDoublePref("Auto Align X Tolerance", 0);
    public static final Pref<Double> autoAlignYTolerance = unitlessDoublePref("Auto Align Y Tolerance", 0);
    public static final Pref<Double> autoAlignRotTolerance = unitlessDoublePref("Auto Align Rotation Tolerance", 0);

    public static final Pref<Double> autoAlignMaxSpeed = unitlessDoublePref("Auto Align Max Speed", 0);

    NetworkTable backingTable;

    private static MoPrefs instance;
    private StringPublisher typePublisher;

    static synchronized MoPrefs getInstance() {
        if (instance == null) {
            instance = new MoPrefs();
        }
        return instance;
    }

    public static void cleanUpPrefs() {
        MoPrefs instance = getInstance();

        HashSet<String> pref_keys = new HashSet<>();

        // Shouldn't remove the special field .type
        pref_keys.add(".type");

        for (Field f : MoPrefs.class.getFields()) {
            if (Modifier.isStatic(f.getModifiers())) {
                Object fo;

                try {
                    fo = f.get(instance);
                } catch (IllegalArgumentException | IllegalAccessException e) {
                    continue;
                }

                if (fo instanceof Pref fop) {
                    pref_keys.add(fop.getKey());
                }
            }
        }

        Set<String> table_keys = instance.backingTable.getKeys();

        System.out.println("****** Clean up MoPrefs ******");
        for (String key : table_keys) {
            if (!pref_keys.contains(key)) {
                System.out.format("Remove unused pref \"%s\"\n", key);

                Topic topic = instance.backingTable.getTopic(key);
                topic.setPersistent(false);
                topic.setRetained(false);
            }
        }
    }

    private MoPrefs() {
        backingTable = NetworkTableInstance.getDefault().getTable("Preferences");

        String kSmartDashboardType = "RobotPreferences";
        typePublisher = backingTable
                .getStringTopic(".type")
                .publishEx(StringTopic.kTypeString, "{\"SmartDashboard\":\"" + kSmartDashboardType + "\"}");
        typePublisher.set(kSmartDashboardType);
    }

    private static Pref<Boolean> booleanPref(String key, boolean defaultValue) {
        return new Pref<>(key, defaultValue, NetworkTableValue::getBoolean, NetworkTableEntry::setBoolean);
    }

    private static Pref<Double> unitlessDoublePref(String key, double defaultValue) {
        return new Pref<>(key, defaultValue, NetworkTableValue::getDouble, NetworkTableEntry::setDouble);
    }
}
