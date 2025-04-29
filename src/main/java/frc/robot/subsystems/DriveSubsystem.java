package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.io.File;
import java.util.function.Supplier;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class DriveSubsystem extends SubsystemBase {

    File directory = new File(Filesystem.getDeployDirectory(), "swerve");
    SwerveDrive swerveDrive;

    public DriveSubsystem() {
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maximumSpeed, Pose2d.kZero);
        } catch (Exception e) {
            throw new RuntimeException(e);
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
