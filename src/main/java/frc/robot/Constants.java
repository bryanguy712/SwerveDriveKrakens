// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final CANAddress SWERVE_TURN_LEFT_FRONT = new CANAddress(1);
  public static final CANAddress SWERVE_DRIVE_LEFT_FRONT = new CANAddress(2);
  public static final CANAddress SWERVE_MODULE_ENCODER_LEFT_FRONT = new CANAddress(9);

  public static final CANAddress SWERVE_TURN_RIGHT_FRONT = new CANAddress(3);
  public static final CANAddress SWERVE_DRIVE_RIGHT_FRONT = new CANAddress(4);
  public static final CANAddress SWERVE_MODULE_ENCODER_RIGHT_FRONT = new CANAddress(10);
   
  public static final CANAddress SWERVE_DRIVE_LEFT_REAR = new CANAddress(5);
  public static final CANAddress SWERVE_TURN_LEFT_REAR = new CANAddress(6);
  public static final CANAddress SWERVE_MODULE_ENCODER_LEFT_REAR =  new CANAddress(11);

  public static final CANAddress SWERVE_DRIVE_RIGHT_REAR = new CANAddress(7);
  public static final CANAddress SWERVE_TURN_RIGHT_REAR = new CANAddress(8);
  public static final CANAddress SWERVE_MODULE_ENCODER_RIGHT_REAR = new CANAddress(12);

  public static record CANAddress(int address) {}
}
