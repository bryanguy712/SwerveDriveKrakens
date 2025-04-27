package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class DriveCommand extends Command {

    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(Constants.MaxSpeed * 0.1).withRotationalDeadband(Constants.MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private final Joystick joystick = new Joystick(Constants.JOYSTICK.hidport());
    public final SwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();


    public DriveCommand() {
        
    }

    @Override
    public void execute() {
        m_drivetrain.setControl(
            m_driveRequest.withVelocityX(-joystick.getX())
               .withVelocityY(-joystick.getY())
               .withRotationalRate(-joystick.getZ())
         );
    }
}
