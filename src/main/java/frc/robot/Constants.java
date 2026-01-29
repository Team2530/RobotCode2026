// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms                                                                                                                                                                
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
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
import edu.wpi.first.wpilibj.Filesystem;
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
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static class RobotConstants {
    // TODO: UPDATE BASED ON REAL ROBOT
    public static final double robotWidthMeters = Units.inchesToMeters(29.5);
    public static final double robotLengthMeters = Units.inchesToMeters(29.5);

    public static final double TOTAL_MASS_KG = 1.0;
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

  public static final class DriveConstants {
    // TODO: Make sure that this is correct - this is from the SDS website but needs
    // empirical verification
    public static final double MAX_ROBOT_VELOCITY = 4.2;
    public static final double MAX_ROBOT_RAD_VELOCITY = 12.0;


    // (((((65kg×1(m/s^2))/4)×(4in/2))/6.75)/(0.0194Nm/A))×(0.033ohm)
    public static final double GLOBAL_kA = 0.135; // V/(m/ss)

    
    // TODO: UPDATE BASED ON REAL ROBOT
    public static final double TRACK_WIDTH = Units.inchesToMeters(19.675);
    public static final double WHEEL_BASE = Units.inchesToMeters(19.675);
    public static final double FULL_ROBOT_WIDTH = Units.inchesToMeters(37.50);

    // TODO: UPDATE BASED ON REAL ROBOT
    public static final PIDConstants TRANSLATION_ASSIST = new PIDConstants(8, 0, 0.01);
    public static final PIDConstants ROTATION_ASSIST = new PIDConstants(7.0, 0, 0.02);

    public static final class ControlConstants {
        public static final double REGULAR_DRIVE_MULT = 1.0;
        public static final double TURTLE_DRIVE_MULT = 0.25;

        public static final double DRIVE_MULTIPLIER_SLEW_RATE = 0.5;

        public static class Deadband {
            public static final double X = 0.1;
            public static final double Y = X; // WARNING: can i do this?
            public static final double Z = 0.08;

            // the radius from 0 to 1 after which the angle-based heading 
            // control activates
            public static final double HEADING = 0.1;
        };
    };

    public static final boolean USE_COSINE_COMPENSATION = true;

    public static final class AngularCompensation {
        // correction for heading skew when rotating
        // (see)[https://yet-another-software-suite.github.io/YAGSL/javadocs/swervelib/SwerveDrive.html#setAngularVelocityCompensation(boolean,boolean,double)]
        public static final boolean ENABLE_IN_TELEOP = true;
        public static final boolean ENABLE_IN_AUTO = true;
        
        // expected values are between -0.15 and 0.15
        public static final double COMPENSATION_COEFFICIENT = 0.1;
    }

    public static final class PIDs {
        // TODO: tune all

        public static final class Drive {
            public static final double P = 0.002;
            public static final double I = 0;
            public static final double D = 0;
            public static final double F = 0;
            public static final double IZ = 0;
        }

        // for the steer motors on the modules
        public static final class Angle {
            public static final double P = 0.01;
            public static final double I = 0;
            public static final double D = 0;
            public static final double F = 0;
            public static final double IZ = 0;
        }

        // for angle-based heading control
        public static final class Heading {
            public static final double P = 0.4;
            public static final double I = 0;
            public static final double D = 0;
            public static final double F = 0;
            public static final double IZ = 0.01;
        }
    };

    public static final class IMU {
        public static final int CANID = 0;
        public static final boolean INVERTED = false;
    }

    public static final class SwerveModules {
        // TODO: i've moved the swerve configuration to the `deploy/swerve` 
        // directory, but i'm thinking now that I should change it back
        // to the constants file with how much I have to duplicate the
        // values.
        
        // in inches
        public static final double WHEEL_DIAMETER = 4;
        public static final double WHEEL_FRICTION_COEFFICIENT = 1.19;

        public static final double OPTIMAL_VOLTAGE = 12;
        // both in amps
        // TODO: check with someone who does electronics
        public static final int DRIVE_CURRENT_LIMIT = 70;
        public static final int STEER_CURRENT_LIMIT = 70;
        // the minimum number of seconds it takes the motor to go from 0 to full 
        // throttle
        public static final double DRIVE_RAMP_RATE = 0;
        public static final double STEER_RAMP_RATE = 0;
        // the minimum voltage it takes for the given motor to move
        // TODO: get real values
        public static final double DRIVE_FRICTION_VOLTAGE = 0;
        public static final double STEER_FRICTION_VOLTAGE = 0;


        public static final double ANGLE_GEARING = 26.09;
        public static final double DRIVE_GEARING = 6.03;

        public static final class CanIDs {
            public static final int FL_DRIVE = 1;
            public static final int FL_STEER = 2;
            public static final int FL_CANCODER = 3;

            public static final int FR_DRIVE = 4;
            public static final int FR_STEER = 5;
            public static final int FR_CANCODER = 6;


            public static final int BL_DRIVE = 7;
            public static final int BL_STEER = 8;
            public static final int BL_CANCODER = 9;

            public static final int BR_DRIVE = 10;
            public static final int BR_STEER = 11;
            public static final int BR_CANCODER = 12;
        };
    
        public static final class Offsets {
            // inches left of the center of the robot
            public static final double FL_X = 9.75;
            // inchest forward of the center of the robot
            public static final double FL_Y = 12.263;
            // as in absolute encoder offset
            public static final double FL_ANGLE = 0; // TODO: update

            public static final double FR_X = -9.75;
            public static final double FR_Y = 12.263;
            public static final double FR_ANGLE = 0; // TODO: update

            public static final double BL_X = 9.75;
            public static final double BL_Y = -12.263;
            public static final double BL_ANGLE = 0; // TODO: update

            public static final double BR_X = -9.75;
            public static final double BR_Y = -12.263;
            public static final double BR_ANGLE = 0; // TODO: update
        };
    }

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
            DriveConstants.SwerveModules.WHEEL_DIAMETER / 2,
            DriveConstants.MAX_MODULE_VELOCITY,
            DriveConstants.SwerveModules.WHEEL_FRICTION_COEFFICIENT,
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
