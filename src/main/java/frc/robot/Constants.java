// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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
  //IHAVENOIDEAWHATIAMDOING
  //New Module ID
  public static class Drivetrain{
    public static class SwerveModule0{
      public static final int drive_id = 36;
      public static final int turn_id = 61;
      public static final int CAN_coder_id = 8;
      public static final Rotation2d turn_offset = Rotation2d.fromDegrees(339.3);
    }
    public static class SwerveModule1{
      public static final int drive_id = 53;
      public static final int turn_id = 52;
      public static final int CAN_coder_id = 1;
      public static final Rotation2d turn_offset = Rotation2d.fromDegrees(187.3);
    }
    public static class SwerveModule2{
      public static final int drive_id = 55;
      public static final int turn_id = 59;
      public static final int CAN_coder_id = 2;
      public static final Rotation2d turn_offset = Rotation2d.fromDegrees(166.1);
    }
    public static class SwerveModule3{
      public static final int drive_id = 54;
      public static final int turn_id = 58;
      public static final int CAN_coder_id = 3;
      public static final Rotation2d turn_offset = Rotation2d.fromDegrees(148.2);
    }
    public static class conversion_factors {
    
      public static final double translation_motor_ratio = 6.12;
      public static final double encoder_stick_ratio = 42;
      public static final double rotation_motor_ratio = 150 / 7;
  
      public static final double translation_position_conversion_factor = Units.feetToMeters(4 * Math.PI / (encoder_stick_ratio * translation_motor_ratio));
      public static final double translation_velocity_conversion_factor = translation_position_conversion_factor / 60;
      
      public static final double rotation_position_conversion_factor = 360;
    }
    public static class PID_constants {

      public static final double kP = 0.00759;
      
      public static final double kI = 0.00069;
    
      public static final double kD = 0.0001;
    }
    public static final double max_speed = 5;
    public static final double max_angular_speed = 7.0;

    public static final double robot_length = Units.inchesToMeters(25);
    public static final double robot_width = Units.inchesToMeters(25);

    public static final Translation2d FrontRight = new Translation2d(robot_length / 2, -robot_width / 2);
    public static final Translation2d BackRight = new Translation2d(-robot_length / 2, -robot_width / 2);
    public static final Translation2d BackLeft = new Translation2d(-robot_length / 2, robot_width / 2);
    public static final Translation2d FrontLeft = new Translation2d(robot_length / 2, robot_width / 2);

    public static final SwerveDriveKinematics swerve_kinematics = new SwerveDriveKinematics(
      FrontRight,
      BackRight,
      BackLeft,
      FrontLeft);
  }
  //Constants for PathPlanner
  public static class PathPlanner{
    //Translation constants for PathPlanner's Controller
    public static final double translation_kP = 0;
    public static final double translation_kI = 0;
    public static final double translation_kD = 0;

    //Rotation constants for PathPlanner's Controller
    public static final double rotation_kP = 0;
    public static final double rotation_kI = 0;
    public static final double rotation_kD = 0;
  }
}
