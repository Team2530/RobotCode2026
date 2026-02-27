package frc.robot;

import java.io.File;
import java.util.HashMap;
import java.util.Optional;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DriveConstants.SwerveModules.Offsets;
import swervelib.parser.PIDFConfig;
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
  /** PID Constant that can set PID Values and get needed controllers and config. */
  public static class PID {
    public final double P, I, D, F, IZ;
    /** PID Constants that can set PID Values and get needed controllers and config. */
    public PID(double P, double I, double D) {this.P=P; this.I=I; this.D=D; this.F=0; this.IZ=0;}
    /** PID Constants that can set PID Values and get needed controllers and config. */
    public PID(double P, double I, double D, double F, double IZ) {this.P=P; this.I=I; this.D=D; this.F=F; this.IZ=IZ;}
    public PIDController getPIDController() {return new PIDController(P, I, D);}
    public PIDFConfig getPIDFConfig() {return new PIDFConfig(P,I,D,F,IZ);}
  }
  public static class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static class GameConstants {
    public static class Fuel {
        // meters 
        public static final double DIAMETER = Units.inchesToMeters(5.9);
        public static final double RADIUS = DIAMETER / 2;
    }
  }

  public static class RobotConstants {
    // TODO: UPDATE BASED ON REAL ROBOT. DONE: FALSE
    public static final double robotWidthMeters = Units.inchesToMeters(29.5);
    public static final double robotLengthMeters = Units.inchesToMeters(29.5);

    // TODO: UPDATE BASED ON REAL ROBOT. DONE: FALSE
    public static final double TOTAL_MASS_KG = Units.lbsToKilograms(107);
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

  public static class HopperConstants {
      // TODO: update
      
      // allese in meters

      // i only really need to know about the static face of the hopper
      public static final double WIDTH = 1;
      // distance to the top of the hopper, relative to the bottom 
      // (like the bottom of the wheels) of the robot
      public static final double ABSOLUTE_HEIGHT = 1;
      // distance backwards from the center of the robot
      public static final double OFFSET_Y = 1;
      // assume the hopper is 
  }

  /*|-----------WARNING-----------------|
   *|edit with caution, used in all subs|
   *|-----------------------------------|
   *|Motor Config (YOU HAVE BEEN WARNED)|
   *|-----------------------------------|
   *|edits can have catastafic falure---|
*///|-----------WARNING-----------------|
  //---Intake Motors---\\
  public static class IntakeConstants {
      //TODO: IDs and speeds need to be changed. DONE: FALSE
      // as in all the wheels on the front that grab the fuel
      public static class Feeder {
        public static final boolean REVERSE = true;  
        public static final int CAN_ID = 13;
        // WARNING: uh?
        public static final double MAXIMUM_VELOCITY = 100;

        public static final class PID {
            public static final double P = 4;
            public static final double I = 0;
            public static final double D = 0;
        }

        public static final class Feedforward {
            public static final double kS = 0.7;
            public static final double kV = 1.04;
        }
      }

      // as in the motor that lifts the whole structure
      public static class Pivot {
          public static final int CAN_ID = 23;
          // output applied when the pivot is moving between stowed / deployed
          // value from -1.0 to 1.0
          public static final double DEPLOY_OUTPUT = 0.2;
          public static final class Zeroing {
              // amps ocourse
              public static final double CURRENT_LIMIT = 35;
              public static final double DEBOUNCE_TIME = 0.15;
          }

          public static final class PID {
            public static final double P = 0.2;
            public static final double I = 0;
            public static final double D = 0;
          }
      }
  }

  public static final class DriveConstants {
    // TODO: Make sure that this is correct - this is from the SDS website but needs
    // empirical verification
    public static final double MAX_ROBOT_VELOCITY = 4.2;
    public static final double MAX_ROBOT_RAD_VELOCITY = 0.25;

    

    // (((((65kg×1(m/s^2))/4)×(4in/2))/6.75)/(0.0194Nm/A))×(0.033ohm)
    public static final double GLOBAL_kA = 0.135; // V/(m/ss)

    
    // TODO: UPDATE BASED ON REAL ROBOT. DONE: FALSE
    public static final double TRACK_WIDTH = Units.inchesToMeters(19.675);
    public static final double WHEEL_BASE = Units.inchesToMeters(19.675);
    public static final double FULL_ROBOT_WIDTH = Units.inchesToMeters(37.50);

    // TODO: UPDATE BASED ON REAL ROBOT. DONE: FALSE
    public static final PIDConstants TRANSLATION_ASSIST = new PIDConstants(8, 0, 0.01);
    public static final PIDConstants ROTATION_ASSIST = new PIDConstants(7.0, 0, 0.02);

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
    new Translation2d(Offsets.FL_X, Offsets.FL_Y), 
    new Translation2d(Offsets.FR_X, Offsets.FR_Y), 
    new Translation2d(Offsets.BL_X, Offsets.BL_Y), 
    new Translation2d(Offsets.BR_X, Offsets.BR_Y)  
);
      
    public static final class ControlConstants {
        public static final double REGULAR_DRIVE_MULT = 1.0;
        public static final double TURTLE_DRIVE_MULT = 0.25;

        public static final double DRIVE_MULTIPLIER_SLEW_RATE = 4.0;

        public static class Deadband {
            public static final double X = 0.1;
            public static final double Y = 0.1;
            public static final double Z = 0.08;

            // the radius from 0 to 1 after which the angle-based heading 
            // control activates
            public static final double HEADING = 0;
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

        public static final PID Drive = new PID(5, 0, 0, 0, 0);
        /** for the steer motors on the modules */
        public static final PID Angle = new PID(100, 0, 0, 0, 0);
        /** for angle-based heading control */
        public static final PID Heading = new PID(0.01, 0, 0, 0, 0);
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
        public static final double DRIVE_RAMP_RATE = 0.05;
        public static final double STEER_RAMP_RATE = 0;
        // the minimum voltage it takes for the given motor to move
        // TODO: get real values
        public static final double DRIVE_FRICTION_VOLTAGE = 0.23;
        public static final double STEER_FRICTION_VOLTAGE = 0.19;

        public static final double DRIVE_GEARING = 6.03;
        public static final double ANGLE_GEARING = 26.09; // 26.09

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
                // as in absolute encoder offset in terms of a full rotation
                public static final double FL_ANGLE = 0.245850;
            public static final boolean FL_ENCODER_INVERTED = false;
            public static final boolean FL_DRIVE_INVERTED = false;
            public static final boolean FL_ANGLE_INVERTED = true;

            public static final double FR_X = 9.75;
            public static final double FR_Y = -12.263;
            public static final double FR_ANGLE = 0.152832;
            public static final boolean FR_ENCODER_INVERTED = false;
            public static final boolean FR_DRIVE_INVERTED = true;
            public static final boolean FR_ANGLE_INVERTED = true;

            public static final double BL_X = -9.75;
            public static final double BL_Y = 12.263;
            public static final double BL_ANGLE = 0.917969;
            public static final boolean BL_ENCODER_INVERTED = false;
            public static final boolean BL_DRIVE_INVERTED = false;
            public static final boolean BL_ANGLE_INVERTED = true;

            public static final double BR_X = -9.75;
            public static final double BR_Y = -12.263;
            public static final double BR_ANGLE = 0.075928;
            public static final boolean BR_ENCODER_INVERTED = false;
            public static final boolean BR_DRIVE_INVERTED = false;
            public static final boolean BR_ANGLE_INVERTED = true;
        };
    }

  }
  public static final class choreoConstants {
    public static final File AUTO_PATH_DIRECTORY = new File(Filesystem.getDeployDirectory(), "choreo");
    public static final PID x_CONTROLLER = new PID(10, 0, 0);
    public static final PID y_CONTROLLER = new PID(10, 0, 0);
    public static final PID heading_CONTROLLER = new PID(7.5, 0, 0);
  }
  public static class CommonConstants {
    public static final boolean LOG_INTO_FILE_ENABLED = true;
    public static final boolean LOG_TO_NETWORKTABLES = true;
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
  public static final class TurretConstants {
    // TODO: UPDATE BASED ON REAL ROBOT
    public static final class Launcher {
        /** In Inches */
        public static final double WHEEL_DIAMETER = 4;
        public static final boolean REVERSE = true;

        public static final double VELOCITY_DEADBAND = 10;

        /** in rotations/s */
        public static final double MAXIMUM_VELOCITY = 500;
        /** In Rotations/s */
        public static final double MINIMUM_VELOCITY = 25;

        public static final double MAXIMUM_VELOCITY_ERROR = 10;

        public static final class PID {
            public static final class Holding {
                public static final double P = 10;
                public static final double I = 0;
                public static final double D = 0;
            }

            public static final class Launching {
                public static final double P = 40;
                public static final double I = 0;
                public static final double D = 0;
            }
        }

        public static final class Feedforward {
            public static final class Holding {
                public static final double kS = 0.44;
                public static final double kV = 0.645;
            }

            public static final class Launching {
                public static final double kS = Holding.kS;
                public static final double kV = Holding.kV;
            }
        }

        public static final class VelocityRegression {
            public static final double A = 0.748916;
            public static final double B = -0.678921;
        }
    }

    public static final class Yaw {
        public static final double GEAR_RATIO = 45;
        public static final boolean REVERSE = false;

        // angular limits on yaw movement
        // 180 is pointed directly out the turret side
        // public static final double ANGLE_MIN = 1.2;
        public static final double ANGLE_MIN = 0;
        public static final double ANGLE_MAX = 340;
        
        // rps
        public static final double MAX_VELOCITY = 45;
        public static final double MAX_ACCELERATION= 800;


        public static final class Zeroing {
            // amps
            public static final double ROUGHPASS_CURRENT_LIMIT = 45;
            public static final double ROUGHPASS_VOLTAGE = -2;
            public static final double FINEPASS_CURRENT_LIMIT = 35;
            public static final double FINEPASS_VOLTAGE = -1;
            // time above current limit to register as hitting the hard limit
            public static final double DEBOUNCE_TIME = 0.1;
        }

        public static final class PID {
            public static final double P = 4;
            public static final double I = 0;
            public static final double D = 0;
        }

        public static final class Feedforward {
            public static final double kS = 0;
            public static final double kV = 0;
            public static final double kA = 0;
        }
    }

    public static final class Pitch {
        public static final double GEAR_RATIO = 1;

        public static final double ANGLE_CONSTANT = 70.5;

        // 90 would have the "face" of the turret as vertical
        // 0 would have the "face" be horizontal (outputting up)
        /* 
         * the minimum angle is dependent on the yaw
         * @param yaw - the angle of the turret yaw in degrees
         */
        public static double ANGLE_MIN(double yaw) {
            if (
                Math.abs(yaw - 180) > 0 
            ) {
                // dependent on the yaw so we can shoot over the hopper
                return Math.atan(
                    (
                        HopperConstants.ABSOLUTE_HEIGHT 
                        + GameConstants.Fuel.RADIUS
                        - TurretConstants.Offsets.Z
                    ) / (
                        Math.tan(
                            Units.degreesToRadians(yaw) 
                        ) * (
                            TurretConstants.Offsets.Y 
                            - HopperConstants.OFFSET_Y
                        )
                    )
                );
            
            } else {
                // physical hardstop of the hood
                return 90;
            }
        }
        public static final double ANGLE_MAX = 180;

        public static final class PID {
            public static final double P = 1;
            public static final double I = 0;
            public static final double D = 0;
        }
    }

    public static final class CanIDs {
        public static final int LAUNCHER_MOTOR = 15;

        public static final int YAW_MOTOR = 14;

        public static final int PITCH_MOTOR = 41;
        public static final int PITCH_ENCODER = 40;
    }

    public static final class Offsets {
        public static final double YAW_ENCODER_ANGLE = 0;
        public static final double PITCH_ENCODER_ANGLE = 0;

        // TODO: update
        // position relative to the center of the robot, in meters
        public static final double X = Units.inchesToMeters(8.625);
        public static final double Y = Units.inchesToMeters(1.375);
        // Height of Shooter from Ground, in meters
        public static final double Z = Units.inchesToMeters(9.375); 
        public static final Translation3d TRANSLATION = new Translation3d(
            X,
            Y,
            Z
        );
    }

    public static final class TargetingOptimizer {
        public static final int INTERPOLATION_POINTS = 9;
        public static final int MAX_EVALUATIONS = 5000;
        
        public static final double MAXIMUM_TIME = 5;
    }

  }

  public static final class IndexerConstants {
      // TODO: UPDATE BASED ON REAL ROBOT
      public static final boolean REVERSE = true;
      public static final int CAN_ID = 24;
      public static final double SPEED = 0.80;
  }

  public static final class LoaderConstants {
      public static final boolean REVERSE = true;
      public static final int CAN_ID = 16;
      // TODO: get a better number
      public static final double MAXIMUM_VELOCITY = 100;

      public static final class PID {
        public static final double P = 1;
        public static final double I = 0;
        public static final double D = 0;
      }
  }
}
