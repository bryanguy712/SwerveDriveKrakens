package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.component.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class AutoAlign extends Command {
    private PIDController xPIDController, yPIDController, rotationPIDController;
    private boolean scoreRightSide;
    private Timer dontSeeTagTimer, stopTimer;
    private DriveSubsystem drive;
    private double tagID = 0;

    public void AlignToReefTagRelative(boolean scoreRightSide, DriveSubsystem drive) {
        xPIDController = new PIDController(Constants.X_REEF_ALIGNMENT_P, 0.0, 0); // Vertical movement
        yPIDController = new PIDController(Constants.Y_REEF_ALIGNMENT_P, 0.0, 0); // Horitontal movement
        rotationPIDController = new PIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0); // Rotation
        this.scoreRightSide = scoreRightSide;
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        this.stopTimer = new Timer();
        this.stopTimer.start();
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();

        xPIDController.setSetpoint(Constants.X_SETPOINT_REEF_ALIGNMENT);
        xPIDController.setTolerance(Constants.X_TOLERANCE_REEF_ALIGNMENT);

        yPIDController.setSetpoint(
                scoreRightSide ? Constants.Y_SETPOINT_REEF_ALIGNMENT : -Constants.Y_SETPOINT_REEF_ALIGNMENT);
        yPIDController.setTolerance(Constants.Y_TOLERANCE_REEF_ALIGNMENT);

        rotationPIDController.setSetpoint(Constants.ROT_SETPOINT_REEF_ALIGNMENT);
        rotationPIDController.setTolerance(Constants.ROT_TOLERANCE_REEF_ALIGNMENT);

        tagID = LimelightHelpers.getFiducialID("");
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
            this.dontSeeTagTimer.reset();

            double[] postions = LimelightHelpers.getBotPose_TargetSpace("");
            SmartDashboard.putNumber("x", postions[2]);

            double xSpeed = xPIDController.calculate(postions[2]);
            SmartDashboard.putNumber("xspee", xSpeed);
            double ySpeed = -yPIDController.calculate(postions[0]);
            double rotValue = -rotationPIDController.calculate(postions[4]);

            drive.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

            if (!rotationPIDController.atSetpoint() || !yPIDController.atSetpoint() || !xPIDController.atSetpoint()) {
                stopTimer.reset();
            }
        } else {
            drive.drive(new Translation2d(), 0, false);
        }

        SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new Translation2d(), 0, false);
    }

    @Override
    public boolean isFinished() {
        // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
        return this.dontSeeTagTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME)
                || stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
    }
}
