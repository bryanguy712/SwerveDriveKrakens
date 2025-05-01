package frc.robot.subsystems;

import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.moprefs.MoPrefs;

import java.io.File;
import java.util.function.Supplier;

import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveSubsystem extends SubsystemBase {

    private final File directory = new File(Filesystem.getDeployDirectory(), "swerve");
    private final SwerveDrive swerveDrive;

    private PIDFConfig moveConfig;
    private PIDFConfig turnConfig;

    public DriveSubsystem() {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(MoPrefs.driveMaxSpeedMPS.get());
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        SwerveModule firstModule = swerveDrive.getModules()[0];
        moveConfig = firstModule.getDrivePIDF();
        turnConfig = firstModule.getAnglePIDF();

        MoPrefs.moveP.set(moveConfig.p);
        MoPrefs.moveI.set(moveConfig.i);
        MoPrefs.moveD.set(moveConfig.d);
        MoPrefs.moveFF.set(moveConfig.f);
        MoPrefs.moveIZone.set(moveConfig.iz);

        MoPrefs.turnP.set(moveConfig.p);
        MoPrefs.turnI.set(moveConfig.i);
        MoPrefs.turnD.set(moveConfig.d);
        MoPrefs.turnIZone.set(moveConfig.iz);

        MoPrefs.moveP.subscribe(kP -> {
            moveConfig.p = kP;
            refreshPIDF();
        });

        MoPrefs.moveI.subscribe(kI -> {
            moveConfig.i = kI;
            refreshPIDF();
        });

        MoPrefs.moveD.subscribe(kD -> {
            moveConfig.d = kD;
            refreshPIDF();
        });

        MoPrefs.moveFF.subscribe(kFF -> {
            moveConfig.f = kFF;
            refreshPIDF();
        });

        MoPrefs.moveIZone.subscribe(kIZone -> {
            moveConfig.iz = kIZone;
            refreshPIDF();
        });

        MoPrefs.turnP.subscribe(kP -> {
            turnConfig.p = kP;
            refreshPIDF();
        });

        MoPrefs.turnI.subscribe(kI -> {
            turnConfig.i = kI;
            refreshPIDF();
        });

        MoPrefs.turnD.subscribe(kD -> {
            turnConfig.d = kD;
            refreshPIDF();
        });

        MoPrefs.turnIZone.subscribe(kIZone -> {
            turnConfig.iz = kIZone;
            refreshPIDF();
        });

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

    private void refreshPIDF() {
        setModulePID(moveConfig, turnConfig);
    }

    private void setModulePID(PIDFConfig moveConfig, PIDFConfig turnConfig) {
        for(SwerveModule module : swerveDrive.getModules()) {
            module.setDrivePIDF(moveConfig);
            module.setAnglePIDF(turnConfig);
        }
     }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }
}
