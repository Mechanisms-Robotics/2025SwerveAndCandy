// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.HashMap;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
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
  public static boolean isBlueAlliance = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue);
  static {
    updateAlliance();
  };
  public static void updateAlliance() {
    if (Robot.isSimulation()) {
      isBlueAlliance = DriverStationSim.getAllianceStationId().toString().contains("Blue");
    } else {
      isBlueAlliance = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue);
    }
  }
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
      Units.inchesToMeters(16.0) - 0.085,
      // Distance to the left of the center of the robot from the robots perspective, i.e. when you look at the front of the robot,
      // the distance from the right of the center from YOUR perspective.
      Units.inchesToMeters(-1.25),
      // Distance from the floor to the camera
      0.16,
      // Rotation 3d containing the role, pitch, and yaw. This Camera only uses pitch because it simply points up so it can see more.
      new Rotation3d(0.0, Units.degreesToRadians(-30.0), 0.0));
  }

  public static final class LimeLight2
  {
    public static final String NICKNAME = "LimeLight2";
    public static final Transform3d FIELD_TO_CAMERA = new Transform3d(
      // Forward distance from the center of the robot to the camera
      Units.inchesToMeters(-(32.0 + 1.0/8.0)/2.0 + 1.0),
      // Distance to the left of the center of the robot from the robots perspective, i.e. when you look at the front of the robot,
      // the distance from the right of the center from YOUR perspective.
      Units.inchesToMeters(-(27.0 + 3.0/4.0)/2.0 + 7.5),
      // Distance from the floor to the camera
      Units.inchesToMeters(27.0 + 3.0/4.0),
      // Rotation 3d containing the role, pitch, and yaw. This Camera only uses pitch because it simply points up so it can see more.
      new Rotation3d(0.0, 0.0, 180)
    );
  }
  
  public static final class FieldConstants {
    public static final double FIELD_LENGTH = 17.548; // meters
    public static final double FIELD_WIDTH = 8.052; // meters
    public static final double PIPE_DISTANCE = Units.inchesToMeters(13);
    // the pipe distance for our reef is an inch to long
    public static final double MECHANISMS_PIPE_DISTANCE = Units.inchesToMeters(14);
    public static final Translation2d RED_REEF_CENTER = new Translation2d(
      // calculated to be the midpoint between the position of apriltag 10 and 7
      (12.227305999999999 + 13.890498)/2, 
      FIELD_WIDTH / 2.0);
    public static final Translation2d BLUE_REEF_CENTER = new Translation2d(
      // calculated to be the midpoint between the position of apriltag 21 and 7
      (5.321046 + 3.6576)/2, 
      FIELD_WIDTH / 2.0);
    // teh distance between apriltag 10 and 7 divided by 2
    public static final double REEF_RADIUS = (13.890498 - 12.2273059999999990)/2.0;
    public static final double REEF_CENTER_DISTANCE = RED_REEF_CENTER.getX() - BLUE_REEF_CENTER.getX();

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

    public static final int[] RED_REEF_APRIL_TAGS = {
      6, 7, 8, 9, 10, 11,
    };

    public static final int[] BLUE_REEF_APRIL_TAGS = {
      17, 18, 19, 20, 21, 22
    };

    public static final double BLUE_BARG_POSE = 7.0;
    public static final double RED_BARG_POSE = FIELD_LENGTH - BLUE_BARG_POSE;
    public static final Pose2d BLUE_CORAL_STATION_LEFT = new Pose2d(0.8, 7.5, Rotation2d.fromDegrees(-145.0));
    public static final Pose2d BLUE_CORAL_STATION_RIGHT;
    public static final Pose2d RED_CORAL_STATION_LEFT;
    public static final Pose2d RED_CORAL_STATION_RIGHT;
    public static final Pose2d RED_PROCESSOR_POSE = new Pose2d(11.18, 7.43, Rotation2d.fromDegrees(90));
    public static final Pose2d BLUE_PROCESSOR_POSE = new Pose2d(FIELD_LENGTH - RED_PROCESSOR_POSE.getX(), FIELD_WIDTH - RED_PROCESSOR_POSE.getY(), Rotation2d.fromDegrees(-90));

    
    // Reef positions pairs, first for left, second for right, mapped to the apriltag ids
    public static final HashMap<Integer, Pair<Pose2d, Pose2d>> BLUE_REEF_POSES = new HashMap<Integer, Pair<Pose2d, Pose2d>>();
    public static final HashMap<Integer, Pair<Pose2d, Pose2d>> RED_REEF_POSES = new HashMap<Integer, Pair<Pose2d, Pose2d>>();
    static {
      // Note for doing the math, it is counter clockwise positive, and the right side of the blue side is the origin
      // Rotating around the center of the reef by 60 degrees (counter clockwise positive) is how we calculate the reef positions
      // from one measured reef position
      // Here, we measured the position that worked using apriltag id 7. We rotate around 7 for the other positions
      Pose2d tag7refLeft = new Pose2d(14.5, 3.52, Rotation2d.fromDegrees(180));

      // Offseting the left position by the reef pipe center to center distance computes the right position
      // Saving the offset to is good because it is easier to rotate around the offset rather than rotate around the left peg side
      final double FUDGE_FACTOR = -Units.inchesToMeters(2.5);
      Pose2d tag7refRight = tag7refLeft.transformBy(new Transform2d(0.0, -PIPE_DISTANCE + FUDGE_FACTOR, Rotation2d.kZero));

      // I think that we should make a list of all the apriltag values on each reef, and in the place of the keys below, it would be x in the reference, 
      //x-1 in the one before, x+1 in the one after, etc, where x it the item number of the id on the list



      RED_REEF_POSES.put(6, new Pair<>(
        tag7refLeft.rotateAround(RED_REEF_CENTER, Rotation2d.fromDegrees(-60.0)),
        tag7refRight.rotateAround(RED_REEF_CENTER, Rotation2d.fromDegrees(-60.0))
      ));
      RED_REEF_POSES.put(7, new Pair<>(
        tag7refLeft,
        tag7refRight
      ));
      RED_REEF_POSES.put(8, new Pair<>(
        tag7refLeft.rotateAround(RED_REEF_CENTER, Rotation2d.fromDegrees(60.0)),
        tag7refRight.rotateAround(RED_REEF_CENTER, Rotation2d.fromDegrees(60.0))
      ));
      RED_REEF_POSES.put(9, new Pair<>(
        tag7refLeft.rotateAround(RED_REEF_CENTER, Rotation2d.fromDegrees(60.0*2.0)),
        tag7refRight.rotateAround(RED_REEF_CENTER, Rotation2d.fromDegrees(60.0*2.0))
      ));
      RED_REEF_POSES.put(10, new Pair<>(
        tag7refLeft.rotateAround(RED_REEF_CENTER, Rotation2d.fromDegrees(60.0*3.0)),
        tag7refRight.rotateAround(RED_REEF_CENTER, Rotation2d.fromDegrees(60.0*3.0))
      ));
      RED_REEF_POSES.put(11, new Pair<>(
        tag7refLeft.rotateAround(RED_REEF_CENTER, Rotation2d.fromDegrees(60.0*4.0)),
        tag7refRight.rotateAround(RED_REEF_CENTER, Rotation2d.fromDegrees(60.0*4.0))
      ));

      // I'm just creating a reference point at 11 so I can reflect it across the field
      Pose2d tag11ref = 
        tag7refLeft.rotateAround(RED_REEF_CENTER, Rotation2d.fromDegrees(60.0*4.0));
      Pose2d tag11refOffset = 
        tag7refRight.rotateAround(RED_REEF_CENTER, Rotation2d.fromDegrees(60.0*4.0));

      Pose2d tag17ref = new Pose2d(tag11ref.getX() - REEF_CENTER_DISTANCE, tag11ref.getY(), Rotation2d.fromDegrees(60.0));
      Pose2d tag17refOffset = new Pose2d(tag11refOffset.getX() - REEF_CENTER_DISTANCE, tag11refOffset.getY(), Rotation2d.fromDegrees(60.0));
      BLUE_REEF_POSES.put(17, new Pair<>(
        tag17ref, 
        tag17refOffset
      ));
      BLUE_REEF_POSES.put(18, new Pair<>(
        tag17ref.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-60.0)),
        tag17refOffset.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-60.0))
      ));
      BLUE_REEF_POSES.put(19, new Pair<>(
        tag17ref.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-60.0*2.0)),
        tag17refOffset.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-60.0*2.0))
      ));
      BLUE_REEF_POSES.put(20, new Pair<>(
        tag17ref.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-60.0*3.0)),
        tag17refOffset.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-60.0*3.0))
      ));
      BLUE_REEF_POSES.put(21, new Pair<>(
        tag17ref.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-60.0*4.0)),
        tag17refOffset.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-60.0*4.0))
      ));
      BLUE_REEF_POSES.put(22, new Pair<>(
        tag17ref.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-60.0*5.0)),
        tag17refOffset.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-60.0*5.0))
      ));

      // unary minus reflects the angle across the length of the field for rotation
      BLUE_CORAL_STATION_RIGHT = new Pose2d(
        BLUE_CORAL_STATION_LEFT.getX(),
        FIELD_WIDTH - BLUE_CORAL_STATION_LEFT.getY(), 
        BLUE_CORAL_STATION_LEFT.getRotation().unaryMinus().rotateBy(Rotation2d.k180deg)
      );
      RED_CORAL_STATION_LEFT = new Pose2d(
        FIELD_LENGTH - BLUE_CORAL_STATION_LEFT.getX(),
        BLUE_CORAL_STATION_RIGHT.getY(),
        BLUE_CORAL_STATION_LEFT.getRotation().rotateBy(Rotation2d.k180deg)
      );
      RED_CORAL_STATION_RIGHT = new Pose2d(
        RED_CORAL_STATION_LEFT.getX(),
        FIELD_WIDTH - RED_CORAL_STATION_LEFT.getY(),
        RED_CORAL_STATION_LEFT.getRotation().unaryMinus().rotateBy(Rotation2d.k180deg)
      );
    }
  }
}
