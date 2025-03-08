// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_AND_ALGAE_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.0025;
    public static final double TURN_CONSTANT    = 6;
  }

  public static final class Vision
  {
    // The name of the limelight
    // See http://photonvision.local:5800/ while connected to the robot, it should be in the top right corner as camera
    // This is not the host name. The docs say to change the host name to differentiate between limelights, host name may be changed.
    // Nickname is enough to differentiate so I have not changed it. I am not sure what hostname does to differentitate between cameras.
    public static final AprilTagFields FIELD = AprilTagFields.k2025ReefscapeWelded;
  }
  public static final class LimeLight1
  {
    public static final String NICKNAME = "LimeLight1";
    // This uses the https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#robot-coordinate-system courdinate system
    public static final Transform3d FIELD_TO_CAMERA = new Transform3d(
      // Forward distance from the center of the robot to the camera
      Units.inchesToMeters(16.0),
      // Distance to the left of the center of the robot from the robots perspective, i.e. when you look at the front of the robot,
      // the distance from the right of the center from YOUR perspective.
      Units.inchesToMeters(0.0),
      // Distance from the floor to the camera
      Units.inchesToMeters(3.5),
      // Rotation 3d containing the role, pitch, and yaw. This Camera only uses pitch because it simply points up so it can see more.
      new Rotation3d(0.0, Units.degreesToRadians(-30.0), 0.0));
  }
  
  public static final class FieldConstants {
    // Mostly stollen from metal crusaders https://github.com/Metal-Crusaders/Reefscape2025Code/blob/main/src/main/java/frc/robot/constants/Constants.java#L138
    // These are the apriltags that are used for pose estimation
    // Currently it is everything due to limited testing - Micah Maphet 3/8/2025
    public static final int[] GOOD_APRIL_TAGS = {
      // Red Coral station
      1, 2,
      // Red Processor
      3,
      // Red Barge
      4, 5,
      // Red Reef
      6, 7, 8, 9, 10, 11,
      // Blue Coral station
      12, 13,
      // Blue Barge
      14, 15,
      // Blue Processor
      16,
      // Blue Reef
      17, 18, 19, 20, 21, 22,
    };

    public static final int[] REEF_APRIL_TAGS = {
            // Red Reef
            6, 7, 8, 9, 10, 11,
            // Blue Reef
            17, 18, 19, 20, 21, 22
    };

    // RED_REEF_POSES array for ID 6 through 11
    public static final Pose2d[] RED_REEF_POSES = new Pose2d[] {
        new Pose2d(13.917, 2.859, new Rotation2d(2.0943951023931957)), // ID 6
        new Pose2d(14.49476434251882, 4.182964006546419, new Rotation2d(3.141592653589793)), // ID 7
        new Pose2d(13.636158342518822, 5.344784006546417, new Rotation2d(-2.0943951023931957)), // ID 8
        new Pose2d(12.200804, 5.182639999999999, new Rotation2d(-1.0471975511965979)), // ID 9
        new Pose2d(11.623039657481177, 3.8586759934535824, new Rotation2d(0.0)), // ID 10
        new Pose2d(12.481645657481177, 2.696855993453582, new Rotation2d(1.0471975511965974)) // ID 11
    };
    
    // BLUE_REEF_POSES array for ID 17 through 22
    public static final Pose2d[] BLUE_REEF_POSES = new Pose2d[] {
        new Pose2d(3.9121936574811764, 2.696855993453582, new Rotation2d(1.0471975511965974)), // ID 17
        new Pose2d(3.0533336574811782, 3.8586759934535824, new Rotation2d(0.0)), // ID 18
        new Pose2d(3.6313519999999997, 5.182639999999999, new Rotation2d(-1.0471975511965979)), // ID 19
        new Pose2d(5.066452342518822, 5.344784006546417, new Rotation2d(-2.0943951023931957)), // ID 20
        new Pose2d(5.925312342518822, 4.182964006546419, new Rotation2d(3.141592653589793)), // ID 21
        new Pose2d(5.347293999999999, 2.859, new Rotation2d(2.0943951023931957)) // ID 22
    };
  }
}
