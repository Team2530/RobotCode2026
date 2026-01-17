// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms                                                                                                                                                                
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 1;
    public static final int OPERATOR_CONTROLLER_PORT = 2;
  }

  public static class RobotConstants {
    // TODO: UPDATE BASED ON REAL ROBOT
    public static final double robotWidthMeters = Units.inchesToMeters(29.5);
    public static final double robotLengthMeters = Units.inchesToMeters(29.5);

    // TODO: UPDATE BASED ON REAL ROBOT
    public static final double TOTAL_MASS_KG = 74.088;
    public static final double MOMENT_OF_INERTIA = 6.883;
  }

  public static final class FieldConstants {
    public static final double GRAVITY = 9.81;

    public static final double FIELD_LENGTH = Units.inchesToMeters(690.876);
    public static final double FIELD_WIDTH = Units.inchesToMeters(317);

    public static Alliance getAlliance() {
      if (DriverStation.getAlliance().isPresent()) {
        return DriverStation.getAlliance().get();
      }

      return Alliance.Blue;
    }
  }

  public static class SwerveModuleConstants {

    // TODO: UPDATE BASED ON REAL ROBOT
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.85); // ~4 in
    public static final double STEERING_GEAR_RATIO = 1.d / (150d / 7d);

    // TODO: UPDATE BASED ON REAL ROBOT
    // This is for L2 modules with 16T pinions
    public static final double DRIVE_GEAR_RATIO = (1.d / 6.75d);

    // TODO: UPDATE BASED ON REAL ROBOT
    public static final double DRIVE_ROTATION_TO_METER = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER;
    public static final double STEER_ROTATION_TO_RADIANS = STEERING_GEAR_RATIO * Math.PI * 2d;
    public static final double DRIVE_METERS_PER_MINUTE = DRIVE_ROTATION_TO_METER / 60d;
    public static final double STEER_RADIANS_PER_MINUTE = STEER_ROTATION_TO_RADIANS / 60d;
    public static final double STEER_MAX_RAD_SEC = 0.8 * STEERING_GEAR_RATIO * ((5880.f * 2.f * Math.PI) / 60.f);

    // TODO: UPDATE BASED ON REAL ROBOT
    public static final double WHEEL_FRICTION_COEFFICIENT = 1.2;

    // TODO: UPDATE BASED ON REAL ROBOT
    public static final double MODULE_KP = 0.46368;
    public static final double MODULE_KD = 0.0050806;

    // --------- Front Left Module --------- \\
    public static final int FL_DRIVE_ID = 10;
    public static final int FL_STEER_ID = 11;
    public static final int FL_ABSOLUTE_ENCODER_PORT = 30;

    // TODO: UPDATE BASED ON REAL ROBOT
    public static final double FL_OFFSET_RADIANS = Units.rotationsToRadians(0);
    public static final boolean FL_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean FL_MOTOR_REVERSED = true;
    public static final boolean FL_STEERING_MOTOR_REVERSED = true;

    // --------- Front Right Module --------- \\
    public static final int FR_DRIVE_ID = 12;
    public static final int FR_STEER_ID = 13;
    public static final int FR_ABSOLUTE_ENCODER_PORT = 31;

    // TODO: UPDATE BASED ON REAL ROBOT
    public static final double FR_OFFSET_RADIANS = Units.rotationsToRadians(0);
    public static final boolean FR_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean FR_MOTOR_REVERSED = true;
    public static final boolean FR_STEERING_MOTOR_REVERSED = true;

    // --------- Back Right Module --------- \\
    public static final int BR_DRIVE_ID = 14;
    public static final int BR_STEER_ID = 15;
    public static final int BR_ABSOLUTE_ENCODER_PORT = 32;

    // TODO: UPDATE BASED ON REAL ROBOT
    public static final double BR_OFFSET_RADIANS = Units.rotationsToRadians(0);
    public static final boolean BR_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean BR_MOTOR_REVERSED = true;
    public static final boolean BR_STEERING_MOTOR_REVERSED = true;

    // --------- Back Left Module --------- \\
    public static final int BL_DRIVE_ID = 16;
    public static final int BL_STEER_ID = 17;
    public static final int BL_ABSOLUTE_ENCODER_PORT = 33;

    // TODO: UPDATE BASED ON REAL ROBOT
    public static final double BL_OFFSET_RADIANS = Units.rotationsToRadians(0);
    public static final boolean BL_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean BL_MOTOR_REVERSED = true;
    public static final boolean BL_STEERING_MOTOR_REVERSED = true;
  }



  /*|-----------WARNING-----------------|
   *|edit with caution, used in all subs|
   *|-----------------------------------|
   *|Motor Config (YOU HAVE BEEN WARNED)|
   *|-----------------------------------|
   *|edits can have catastafic falure---|
*///|-----------WARNING-----------------|
  //---Intake Motors---\\
  public static class IntakeMotors {
    public static final int INTAKESUBSYSTEM_MOTOR = 4;//TODO: IDs and speeds need to be changed
    public static final double INTAKE_SPEED = 0.6;
    public static final int INTAKE_MOTOR1 = 5;
    public static final double INTAKE_SPEED1 = 0.6;
  }
  //----Climb Motors---\\
  public static class ClimbMotors {
    public static final int CLIMBSUBSYSTEM_MOTOR = 6;//TODO: IDs and speeds need to be changed
    public static final double CLIMB_SPEED = 0.6;
  }



  
  public static class DriveConstants {
    // TODO: Make sure that this is correct - this is from the SDS website but needs
    // empirical verification
    public static final double MAX_MODULE_VELOCITY = 4.2;
    public static final double MAX_ROBOT_VELOCITY = 4.2;
    public static final double MAX_ROBOT_RAD_VELOCITY = 12.0;

    public static final double MAX_MODULE_CURRENT = 70;

    // (((((65kg×1(m/s^2))/4)×(4in/2))/6.75)/(0.0194Nm/A))×(0.033ohm)
    public static final double GLOBAL_kA = 0.135; // V/(m/ss)

    
    // TODO: UPDATE BASED ON REAL ROBOT
    public static final double TRACK_WIDTH = Units.inchesToMeters(19.675);
    public static final double WHEEL_BASE = Units.inchesToMeters(19.675);
    public static final double FULL_ROBOT_WIDTH = Units.inchesToMeters(37.50);

    // TODO: UPDATE BASED ON REAL ROBOT
    public static final PIDConstants TRANSLATION_ASSIST = new PIDConstants(8, 0, 0.01);
    public static final PIDConstants ROTATION_ASSIST = new PIDConstants(7.0, 0, 0.02);

    public static final class ModuleIndices {
      public static final int FRONT_LEFT = 0;
      public static final int FRONT_RIGHT = 2;
      public static final int REAR_LEFT = 1;
      public static final int REAR_RIGHT = 3;
    }

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
        new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0));

    public static final double XY_SPEED_LIMIT = 1.0;
    public static final double Z_SPEED_LIMIT = 1.0;
  }

  public static class CommonConstants {
    public static final boolean LOG_INTO_FILE_ENABLED = true;
    public static final boolean LOG_TO_NETWORKTABLES = true;
  }

  public static final class PathPlannerConstants {

    // TODO: UPDATE BASED ON REAL ROBOT
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(15.0, 0, 0.005);
    public static final PIDConstants ROTATION_PID = new PIDConstants(9.0, 0, 0.03); // was 10 and 9

    public static final PPHolonomicDriveController HOLONOMIC_FOLLOWER_CONTROLLER = new PPHolonomicDriveController(
        TRANSLATION_PID,
        ROTATION_PID);

    public static final RobotConfig ROBOT_CONFIG = new RobotConfig(
        RobotConstants.TOTAL_MASS_KG,
        RobotConstants.MOMENT_OF_INERTIA,
        new ModuleConfig(
            SwerveModuleConstants.WHEEL_DIAMETER / 2,
            DriveConstants.MAX_MODULE_VELOCITY,
            SwerveModuleConstants.WHEEL_FRICTION_COEFFICIENT,
            DCMotor.getKrakenX60(1),
            DriveConstants.MAX_MODULE_CURRENT,
            1),
        DriveConstants.KINEMATICS.getModules());
  }

  public static final class PoseConstants {

    public static final double kPositionStdDevX = 0.1;
    public static final double kPositionStdDevY = 0.1;
    public static final double kPositionStdDevTheta = 10;
    public static final double kVisionStdDevY = 5;
    public static final double kVisionStdDevX = 5;
    public static final double kVisionStdDevTheta = 500;

    private static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public final static HashMap<Integer, Pose2d> TAG_POSES = new HashMap<Integer, Pose2d>() {{
        for (int i = 0; i < TAG_LAYOUT.getTags().size(); ++i) {
          if (TAG_LAYOUT.getTagPose(i + 1).isPresent())
            put(i, TAG_LAYOUT.getTagPose(i + 1).get().toPose2d());
        }
      }};

  }
}
