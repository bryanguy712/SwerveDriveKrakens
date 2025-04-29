// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

    private final DriveSubsystem driveBase = new DriveSubsystem();
    private final XboxController driveController = new XboxController(Constants.XBOXCONTORLLER.hidport());

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
                    driveBase.getSwerveDrive(),
                    () -> driveController.getLeftY() * -1,
                    () -> driveController.getLeftX() * -1)
            .withControllerRotationAxis(driveController::getRightX)
            .deadband(Constants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    SwerveInputStream driveDirectAngle = driveAngularVelocity
            .copy()
            .withControllerHeadingAxis(driveController::getRightX, driveController::getRightY)
            .headingWhile(true);

    Command driveFieldOrientedDirectAngle = driveBase.driveFieldOriented(driveDirectAngle);

    Command driveFieldOrientedAngularVelocity = driveBase.driveFieldOriented(driveAngularVelocity);

    public RobotContainer() {
        configureBindings();
        driveBase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    }

    private void configureBindings() {}

    public Command getAutonomousCommand() {
        return null;
    }
}
