package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase{
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder moduleEncoder;

    public static final Distance SWERVE_WHEEL_OFFSET = Units.Inch.of(11.5);


    public DriveSubsystem(int driveMotorCANID, int steerMotorCANID, int cancoderCANID) {


        driveMotor = new TalonFX(driveMotorCANID);
        steerMotor = new TalonFX(steerMotorCANID);

        moduleEncoder = new CANcoder(cancoderCANID);
        
        Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
        Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
        Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
        Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

        SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
        );

        ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5);
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);
        SwerveModuleState frontLeft = moduleStates[0];
        SwerveModuleState frontRight = moduleStates[1];
        SwerveModuleState backLeft = moduleStates[2];
        SwerveModuleState backRight = moduleStates[3];
    }    
}
