package frc.robot;

import java.lang.reflect.GenericArrayType;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.molib.MoShuffleBoard;
import frc.robot.subsystems.DriveSubsystem;

public class AutoChooser {
    private DriveSubsystem driveTrain;

    private ShuffleboardTab autoTab;

    private enum autoRoutines {
        LEAVE,
        REEF
    }

    private enum startingPosition {
        BLUE_WALL,
        CENTER,
        RED_WALL
    }

    private final GenericEntry autoSwitch;

    private final SendableChooser<autoRoutines> autoRoutinesChooser = MoShuffleBoard.enumToChooser(autoRoutines.class);
    private final SendableChooser<startingPosition> initialPositionChooser = MoShuffleBoard
            .enumToChooser(startingPosition.class);

    public AutoChooser(DriveSubsystem driveTrain) {
        this.driveTrain = driveTrain;

        autoTab = Shuffleboard.getTab("Auto Chooser");

        autoSwitch = autoTab.add("Enable Auto?", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

        autoTab.add("Auto Rounines", autoRoutinesChooser);
        
    }
}
