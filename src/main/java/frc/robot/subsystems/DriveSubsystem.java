package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase{
    //GAINS NEED TO BE EDITED!!!!!!!!!
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(0).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0).withKI(0).withKD(0)
        .withKS(0).withKV(0);
        
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    //To do:
    private static final Current kSlipCurrent = Units.Amps.of(30);

    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                // Swerve azimuth does not require much torque output, so we can set a relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .withStatorCurrentLimit(Units.Amps.of(20))
                .withStatorCurrentLimitEnable(true)
        );

    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

    public static final CANBus kCANBus = new CANBus("canivore", "./logs/example.hoot");

    public static final LinearVelocity kSpeedAt12Volts = Units.MetersPerSecond.of(4.69);

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    private static final double kCoupleRatio = 0;

    //might be wrong, double check this!!!!
    private static final double kDriveGearRatio = 5.4;
    private static final double kSteerGearRatio = 12.1;
    private static final Distance kWheelRadius = Units.Inches.of(2);

    //EDIT THIS!!!!!
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = false;

    // These are only used for simulation, could need some changing
    private static final MomentOfInertia kSteerInertia = Units.KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = Units.KilogramSquareMeters.of(0.01);
    // Simulated voltage necessary to overcome friction
    private static final Voltage kSteerFrictionVoltage = Units.Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Units.Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName());

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);

    //ENCODER OFFSETS, X & Y POSE MUST BE CHANGED!!!!!!!!!!!!!!!!!!
    //INVERSIONS TOO!!!!!
    
    //Front Left Swerve Module
    private static final int kFrontLeftDriveMotorId = Constants.SWERVE_DRIVE_LEFT_FRONT.address();
    private static final int kFrontLeftSteerMotorId = Constants.SWERVE_TURN_LEFT_FRONT.address();
    private static final int kFrontLeftEncoderId = Constants.SWERVE_MODULE_ENCODER_LEFT_FRONT.address();
    private static final Angle kFrontLeftEncoderOffset = Units.Rotations.of(0);

    private static final boolean kFrontLeftSteerMotorInverted = false;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Units.Inches.of(0); 
    private static final Distance kFrontLeftYPos = Units.Inches.of(0);

    //Front Right Swerve Module
    private static final int kFrontRightDriveMotorId = Constants.SWERVE_DRIVE_RIGHT_FRONT.address();
    private static final int kFrontRightSteerMotorId = Constants.SWERVE_TURN_RIGHT_FRONT.address();
    private static final int kFrontRightEncoderId = Constants.SWERVE_MODULE_ENCODER_RIGHT_FRONT.address();
    private static final Angle kFrontRightEncoderOffset = Units.Rotations.of(0);

    private static final boolean kFrontRightSteerMotorInverted = false;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Units.Inches.of(0);
    private static final Distance kFrontRightYPos = Units.Inches.of(0);

    //Rear Left Swerve Module
    private static final int kRearLeftDriveMotorId = Constants.SWERVE_DRIVE_LEFT_REAR.address();
    private static final int kRearLeftSteerMotorId = Constants.SWERVE_TURN_LEFT_REAR.address();
    private static final int kRearLeftEncoderId = Constants.SWERVE_MODULE_ENCODER_LEFT_REAR.address();
    private static final Angle kRearLeftEncoderOffset = Units.Rotations.of(0);

    private static final boolean kRearLeftSteerMotorInverted = false;
    private static final boolean kRearLeftEncoderInverted = false;

    private static final Distance kRearLeftXPos = Units.Inches.of(0);
    private static final Distance kRearLeftYPos = Units.Inches.of(0);

    //Rear Right Swerve Module
    private static final int kRearRightDriveMotorId = Constants.SWERVE_DRIVE_RIGHT_REAR.address();
    private static final int kRearRightSteerMotorId = Constants.SWERVE_TURN_RIGHT_REAR.address();
    private static final int kRearRightEncoderId = Constants.SWERVE_MODULE_ENCODER_RIGHT_REAR.address();
    private static final Angle kRearRightEncoderOffset = Units.Rotations.of(0);

    private static final boolean kRearRightSteerMotorInverted = false;
    private static final boolean kRearRightEncoderInverted = false;

    private static final Distance kRearRightXPos = Units.Inches.of(0);
    private static final Distance kRearRightYPos = Units.Inches.of(0);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontLeft = ConstantCreator.createModuleConstants(kFrontLeftSteerMotorId, 
    kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontRight = ConstantCreator.createModuleConstants(kFrontRightSteerMotorId,
    kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> rearLeft = ConstantCreator.createModuleConstants(kRearLeftSteerMotorId,
    kRearLeftDriveMotorId, kRearLeftEncoderId, kRearLeftEncoderOffset, kRearLeftXPos, kRearLeftYPos, kInvertLeftSide, kRearLeftSteerMotorInverted, kRearLeftEncoderInverted);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> rearRight = ConstantCreator.createModuleConstants(kRearRightSteerMotorId,
    kRearRightDriveMotorId, kRearRightEncoderId, kRearRightEncoderOffset, kRearRightXPos, kRearRightYPos, kInvertRightSide, kRearRightSteerMotorInverted, kRearRightEncoderInverted);

    public static final Distance SWERVE_WHEEL_OFFSET = Units.Inch.of(11.5);

    public final SwerveDriveKinematics kinematics;
    
    public DriveSubsystem() {
        
        double offset_meters = SWERVE_WHEEL_OFFSET.in(Units.Meters);
        Translation2d fl = new Translation2d(offset_meters, offset_meters);
        Translation2d fr = new Translation2d(offset_meters, -offset_meters);
        Translation2d rl = new Translation2d(-offset_meters, offset_meters);
        Translation2d rr = new Translation2d(-offset_meters, -offset_meters);

        this.kinematics = new SwerveDriveKinematics(fl, fr, rl, rr);

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            2.0, 2.0, Math.PI / 2.0, Rotation2d.fromDegrees(45.0));

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        SwerveModuleState frontLeft = moduleStates[0];
        SwerveModuleState frontRight = moduleStates[1];
        SwerveModuleState rearLeft = moduleStates[2];
        SwerveModuleState rearRight = moduleStates[3];

        //Module Angle Optimization and Cosine compensation are not worked on yet.
        
        var frontLeftState = new SwerveModuleState(23.43, Rotation2d.fromDegrees(-140.19));
        var frontRightState = new SwerveModuleState(23.43, Rotation2d.fromDegrees(-39.81));
        var backLeftState = new SwerveModuleState(54.08, Rotation2d.fromDegrees(-109.44));
        var backRightState = new SwerveModuleState(54.08, Rotation2d.fromDegrees(-70.56)); 
        
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(
        frontLeftState, frontRightState, backLeftState, backRightState);
        double forward = chassisSpeeds.vxMetersPerSecond;
        double sideways = chassisSpeeds.vyMetersPerSecond;
        double angular = chassisSpeeds.omegaRadiansPerSecond;
    }

}    

