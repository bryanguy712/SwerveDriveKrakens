// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.molib;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Map;
import java.util.function.Supplier;

public class MoShuffleBoard {
    private static MoShuffleBoard instance;

    public static synchronized MoShuffleBoard getInstance() {
        if (instance == null) {
            try {
                instance = new MoShuffleBoard();
            } catch (IllegalArgumentException e) {
                DriverStation.reportError("Failed to initialize MoShuffleboard", e.getStackTrace());
                instance = null;
            }
        }

        return instance;
    }

    private enum SysIdMode {
        NONE,
        QUASISTATIC_FORWARD,
        QUASISTATIC_REVERSE,
        DYNAMIC_FORWARD,
        DYNAMIC_REVERSE
    };

    private final SendableChooser<SysIdMode> sysidMode = MoShuffleBoard.enumToChooser(SysIdMode.class);

    public final ShuffleboardTab driveTab;
    public final ShuffleboardTab intakeTab;
    public final ShuffleboardTab climberTab;
    public final ShuffleboardTab settingsTab;
    public final ShuffleboardTab autoTab;
    public final ShuffleboardTab elevatorTab;
    public final Field2d field;

    private final GenericEntry voltRampEntry;
    private final GenericEntry voltEntry;

    public final GenericSubscriber tuneSetpointSubscriber;

    private MoShuffleBoard() {
        // Note: if you're getting an IllegalArgumentException here complaining about "title already in use",
        // check that you're not calling MoShuffleboard.getInstance() from within this constructor!
        settingsTab = Shuffleboard.getTab("Settings");
        autoTab = Shuffleboard.getTab("Auto");
        driveTab = Shuffleboard.getTab("Drive");
        intakeTab = Shuffleboard.getTab("Intake");
        climberTab = Shuffleboard.getTab("Climber");
        elevatorTab = Shuffleboard.getTab("Elevator");

        field = new Field2d();

        driveTab.add(field).withSize(5, 3);

        settingsTab.add("Sysid Mode", sysidMode);

        var sysidGroup = settingsTab
                .getLayout("Sysid Settings", BuiltInLayouts.kList)
                .withSize(2, 1)
                .withProperties(Map.of("Label position", "RIGHT"));
        voltRampEntry = sysidGroup
                .add("Volts Ramp", 1.5)
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();
        voltEntry = sysidGroup
                .add("Volts Step", 2)
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        tuneSetpointSubscriber = settingsTab
                .add("Tune Setpoints?", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();
    }

    public SysIdRoutine.Config getSysidConfig() {
        var voltsPerSec = Units.Volts.per(Units.Second);
        return new SysIdRoutine.Config(
                voltsPerSec.of(voltRampEntry.getDouble(1.5)),
                Units.Volts.of(voltEntry.getDouble(3)),
                Units.Seconds.of(45));
    }

    public Command getSysidCommand(Supplier<SysIdRoutine.Mechanism> mechanismSupplier) {
        return Commands.deferredProxy(() -> {
            if (DriverStation.isFMSAttached()) {
                return Commands.print("Refusing to run SysId because the FMS is attached!");
            }

            var mechanism = mechanismSupplier.get();
            var routine = new SysIdRoutine(getSysidConfig(), mechanism);
            switch (sysidMode.getSelected()) {
                case QUASISTATIC_FORWARD:
                    return routine.quasistatic(SysIdRoutine.Direction.kForward);
                case QUASISTATIC_REVERSE:
                    return routine.quasistatic(SysIdRoutine.Direction.kReverse);
                case DYNAMIC_FORWARD:
                    return routine.dynamic(SysIdRoutine.Direction.kForward);
                case DYNAMIC_REVERSE:
                    return routine.dynamic(SysIdRoutine.Direction.kReverse);
                case NONE:
                default:
                    return Commands.none();
            }
        });
    }

    public static <T extends Enum<?>> SendableChooser<T> enumToChooser(Class<T> toConvert) {
        return enumToChooser(toConvert, toConvert.getEnumConstants()[0]);
    }

    public static <T extends Enum<?>> SendableChooser<T> enumToChooser(Class<T> toConvert, T defaultValue) {
        var chooser = new SendableChooser<T>();
        chooser.setDefaultOption(defaultValue.name(), defaultValue);
        for (T entry : toConvert.getEnumConstants()) {
            if (entry != defaultValue) {
                chooser.addOption(entry.name(), entry);
            }
        }
        return chooser;
    }
}