package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.component.LimelightHelpers;
import frc.robot.moprefs.MoPrefs;
import frc.robot.subsystems.DriveSubsystem;

public class AutoAlign extends Command {
    private PIDController xPIDController, yPIDController, rotationPIDController;
    private boolean scoreRightSide;
    private Timer dontSeeTagTimer, stopTimer;
    private DriveSubsystem drive;
    private int tagID = 0;

    private final DoublePublisher xPos;
    private final DoublePublisher yPos;
    private final DoublePublisher rotPos;
    private final DoublePublisher xOutput;
    private final DoublePublisher yOutput;
    private final DoublePublisher rotOutput;
    private final DoublePublisher dontSeeTagTimerPublisher;
    private final DoublePublisher stopTimerPublisher;
    private final IntegerPublisher tagIdPublisher;

    public AutoAlign(boolean scoreRightSide, DriveSubsystem drive) {
        xPIDController = new PIDController(MoPrefs.autoAlignXP.get(), 0.0, 0); // Vertical movement
        MoPrefs.autoAlignXP.subscribe(kP -> {
            xPIDController.setP(kP);
        });
        
        yPIDController = new PIDController(MoPrefs.autoAlignYP.get(), 0.0, 0); // Horitontal movement
        MoPrefs.autoAlignYP.subscribe(kP -> {
            yPIDController.setP(kP);
        }); 

        rotationPIDController = new PIDController(MoPrefs.autoAlignRoationP.get(), 0, 0); // Rotation
        MoPrefs.autoAlignRoationP.subscribe(kp -> {
            rotationPIDController.setP(kp);
        });


        MoPrefs.autoAlignXTolerance.subscribe(tolerance -> {
            xPIDController.setTolerance(tolerance);
        }, true);

        MoPrefs.autoAlignYTolerance.subscribe(tolerance -> {
            yPIDController.setTolerance(scoreRightSide ? tolerance : -tolerance);
        }, true);

        MoPrefs.autoAlignRotTolerance.subscribe(tolerance -> {
            rotationPIDController.setTolerance(tolerance);
        }, true);

        this.scoreRightSide = scoreRightSide;
        this.drive = drive;
        
        this.stopTimer = new Timer();
        this.dontSeeTagTimer = new Timer();

        NetworkTable table = NetworkTableInstance.getDefault().getTable("Auto Align");

        xPos = table.getDoubleTopic("xPos").publish();
        yPos = table.getDoubleTopic("yPos").publish();
        rotPos = table.getDoubleTopic("rotPos").publish();
        xOutput = table.getDoubleTopic("xOutput").publish();
        yOutput = table.getDoubleTopic("yOutput").publish();
        rotOutput = table.getDoubleTopic("rotOutput").publish();
        dontSeeTagTimerPublisher = table.getDoubleTopic("dontSeeTagTimer").publish();
        stopTimerPublisher = table.getDoubleTopic("stopTimer").publish();
        tagIdPublisher = table.getIntegerTopic("tagId").publish();

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        this.stopTimer.restart();
        this.dontSeeTagTimer.restart();

        xPIDController.setSetpoint(scoreRightSide? -MoPrefs.autoAlignXSetPoint.get() : MoPrefs.autoAlignXSetPoint.get());

        yPIDController.setSetpoint(MoPrefs.autoAlignYSetpoint.get());

        rotationPIDController.setSetpoint(MoPrefs.autoAlignRotSetpoint.get());

        tagID = (int) LimelightHelpers.getFiducialID(Constants.LIMELIGHT_NAME);
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTV(Constants.LIMELIGHT_NAME) && ((int)LimelightHelpers.getFiducialID(Constants.LIMELIGHT_NAME)) == tagID) {
            this.dontSeeTagTimer.reset();

            double[] positions = LimelightHelpers.getBotPose_TargetSpace(Constants.LIMELIGHT_NAME);
            double x = positions[0];
            double y = positions[2];
            double rot = positions[4];

            double xSpeed = -xPIDController.calculate(x);
            double ySpeed = yPIDController.calculate(y);
            double rotValue = -rotationPIDController.calculate(rot);

            xSpeed = MathUtil.clamp(xSpeed, -MoPrefs.autoAlignMaxSpeed.get(), MoPrefs.autoAlignMaxSpeed.get());
            ySpeed = MathUtil.clamp(ySpeed, -MoPrefs.autoAlignMaxSpeed.get(), MoPrefs.autoAlignMaxSpeed.get());

            xPos.accept(x);
            yPos.accept(y);
            rotPos.accept(rot);
            xOutput.accept(xSpeed);
            yOutput.accept(ySpeed);
            rotOutput.accept(rotValue);

            drive.drive(new Translation2d(ySpeed, xSpeed), rotValue, false);

            if (!rotationPIDController.atSetpoint() || !yPIDController.atSetpoint() || !xPIDController.atSetpoint()) {
                stopTimer.reset();
            }
        } else {
            drive.drive(new Translation2d(), 0, false);
            stopTimer.reset();
        }

        dontSeeTagTimerPublisher.accept(dontSeeTagTimer.get());
        stopTimerPublisher.accept(stopTimer.get());
        tagIdPublisher.accept(tagID);
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
