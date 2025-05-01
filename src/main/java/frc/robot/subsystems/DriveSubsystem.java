package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.moprefs.MoPrefs;
import java.io.File;
import java.util.function.Supplier;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveSubsystem extends SubsystemBase {

    private final File directory = new File(Filesystem.getDeployDirectory(), "swerve");
    private final SwerveDrive swerveDrive;

    public DriveSubsystem() {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(MoPrefs.driveMaxSpeedMPS.get());
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        MoPrefs.driveMaxSpeedMPS.subscribe(mps -> {
            double rps = MoPrefs.turnMaxSpeedRPS.get();
            swerveDrive.setMaximumAttainableSpeeds(mps, rps);
            swerveDrive.setMaximumAllowableSpeeds(mps, rps);
        });

        MoPrefs.turnMaxSpeedRPS.subscribe(rps -> {
            double mps = MoPrefs.driveMaxSpeedMPS.get();
            swerveDrive.setMaximumAttainableSpeeds(mps, rps);
            swerveDrive.setMaximumAllowableSpeeds(mps, rps);
        });
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    public Command resetFieldOrientedFwd() {
        return runOnce(() -> swerveDrive.setGyro(Rotation3d.kZero));
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }
}
