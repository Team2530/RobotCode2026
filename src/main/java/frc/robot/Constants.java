package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.File;
import java.util.HashMap;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public final class Constants {

    public static final class MetaConstants {
        public static Alliance ALLIANCE =
            DriverStation.getAlliance().isPresent()
                ? DriverStation.getAlliance().get()
                : Alliance.Blue;

        public static BooleanSupplier isRed = new BooleanSupplier() {
                @Override
                public boolean getAsBoolean() {
                    return ALLIANCE == Alliance.Red;
                }
            };

        public static final class Controllers {
            public static final int DRIVER_PORT = 0;
            public static final int OPERATOR_PORT = 1;
        }

        public static final class Logging {
            public static final boolean LOG_INTO_FILE_ENABLED = true;
            public static final boolean LOG_TO_NETWORKTABLES = false;
        }

        public static final class Field {
            public static final LinearAcceleration GRAVITY =
                MetersPerSecondPerSecond.of(9.81);

            public static final Distance LENGTH = Inches.of(690.876);
            public static final Distance WIDTH = Inches.of(317);
        }

        public static final class Game {
            public static final class Fuel {
                public static final Distance DIAMETER = Inches.of(5.9);
                public static final Distance RADIUS =  DIAMETER.div(2);
            }
        }
    }

    public static final class RobotConstants {
        public static final Distance WIDTH = Inches.of(25);
        public static final Distance LENGTH = Inches.of(30);

        // TODO: UPDATE BASED ON REAL ROBOT.
        public static final Mass TOTAL_MASS = Kilogram.of(107);
        public static final MomentOfInertia MOMENT_OF_INERTIA =
            KilogramSquareMeters.of(6.883);
    }

    // TODO: this
    public static final class HopperConstants {
        public static final Distance ABSOLUTE_HEIGHT = Inches.of(22);
        public static final class Offsets {
            public static final Distance X = Inches.of(0);
            public static final Distance Y = Inches.of(1);
        }
    }

    public static final class IntakeConstants {
        // as in all the wheels on the front that grab the fuel
        public static final class Feeder {
            public static final boolean REVERSE = true;
            
            // WARNING: uh?
            public static final AngularVelocity MAXIMUM_VELOCITY =
                RotationsPerSecond.of(100);

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
        public static final class Pivot {
            public static final boolean REVERSE = false;

            public static final Current ABSOLUTE_CURRENT_LIMIT = Amps.of(60);

            public static final class Zeroing {
                // output applied when the pivot is moving between stowed
                // / deployed
                public static final Dimensionless DEPLOY_OUTPUT =
                    Percent.of(20);

                public static final Current CURRENT_LIMIT = Amps.of(30);
                public static final Time DEBOUNCE_TIME = Seconds.of(0.15);

                public static final class PID {
                    public static final double P = 0.05;
                    public static final double I = 0;
                    public static final double D = 0;
                }
            }


            public static final class Waving {
                public static final Time PERIOD = Seconds.of(2);
                // an arbitrary value
                //
                // i think this is dependent on the gear ratio of the pivot, but
                // i really don't know what units this is in
                public static final double HEIGHT = 3;

                public static final class PID {
                    public static final double P = 0.25;
                    public static final double I = 0;
                    public static final double D = 0;
                }
            }
        }

        public static final class CANIDs {
            public static final int PIVOT = 23;
            public static final int FEEDER = 13;
        }
    }

    public static final class DriveConstants {
        // TODO: Make sure that this is correct - this is from the SDS website
        // but needs empirical verification
        public static final LinearVelocity MAX_ROBOT_VELOCITY =
            MetersPerSecond.of(4.2);
        public static final AngularVelocity MAX_ROBOT_RAD_VELOCITY =
            RadiansPerSecond.of(1);

        public static final class Control {
            public static final Dimensionless REGULAR_DRIVE_MULT =
                Percent.of(100);
            public static final Dimensionless TURTLE_DRIVE_MULT =
                Percent.of(25);
            public static final Frequency MULTIPLIER_SLEW_RATE =
                Percent
                .of(25)
                .per(Second);

            public static final class Deadband {
                public static final double X = 0.1;
                public static final double Y = 0.1;
                public static final double Z = 0.08;

                // the radius beyond the center of the joystick--from 0 to
                // 1--after which the angle-based heading control activates
                public static final double HEADING = 0;
            };

            // for angle-based heading control
            public static final class HeadingPID {
                public static final double P = 0.01;
                public static final double I = 0;
                public static final double D = 0;
                public static final double F = 0;
                public static final double IZ = 0;
            }

            // multiply the output of the drive motor by cos(angular_error)
            public static final boolean USE_COSINE_COMPENSATION = true;

            // correction for heading skew when rotating
            // (see)[https://yet-another-software-suite.github.io/YAGSL/javadocs/swervelib/SwerveDrive.html#setAngularVelocityCompensation(boolean,boolean,double)]
            public static final class AngularCompensation {
                public static final boolean ENABLE_IN_TELEOP = false;
                public static final boolean ENABLE_IN_AUTO = false;

                // expected values are between -0.15 and 0.15
                public static final double COMPENSATION_COEFFICIENT = 0.1;
            }
        }

        public static final class IMU {
            public static final boolean INVERTED = false;
        }

        public static final class Modules {
            public static final Distance WHEEL_DIAMETER = Inches.of(4);
            public static final double WHEEL_FRICTION_COEFFICIENT = 2.255;

            public static final Voltage OPTIMAL_VOLTAGE = Volts.of(12);
            public static final Current DRIVE_CURRENT_LIMIT = Amps.of(120);
            public static final Current STEER_CURRENT_LIMIT = Amps.of(120);

            // the minimum number of seconds it takes the motor to go from 0 to
            // full throttle
            public static final Time DRIVE_RAMP = Seconds.of(0);
            public static final Time STEER_RAMP = Seconds.of(0);

            // the minimum voltage it takes for the given motor to move
            public static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.23);
            public static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.19);

            public static final class Gearing {
                public static final class Drive{
                    public static final double LIGHT = 7.03;
                    public static final double RIDICULUS = 6.03;
                    public static final double LUDICRUS = 5.27;
                }

                public static final double ANGLE = 26.09;
            }


            public static final class Offsets {
                // distance left of the center of the robot
                private static final Distance BASE_X = Inches.of(9.75);
                // distance forward of the center of the robot
                private static final Distance BASE_Y = Inches.of(12.263);


                // im making the assumption that the swerve module placement is
                // symmetrical on both axes
                public static final class FL {
                    public static final Distance X = BASE_X;
                    public static final Distance Y = BASE_Y;
                    // the absolute encoder offset
                    public static final Angle ANGLE = Rotations.of(0.073975);
                    public static final boolean ENCODER_INVERTED = false;
                    public static final boolean DRIVE_INVERTED = false;
                    public static final boolean ANGLE_INVERTED = true;
                }
                public static final class FR {
                    public static final Distance X = BASE_X;
                    public static final Distance Y = BASE_Y.times(-1);
                    public static final Angle ANGLE = Rotations.of(0.918457);
                    public static final boolean ENCODER_INVERTED = false;
                    public static final boolean DRIVE_INVERTED = false;
                    public static final boolean ANGLE_INVERTED = true;
                }
                public static final class BL {
                    public static final Distance X = BASE_X.times(-1);
                    public static final Distance Y = BASE_Y;
                    public static final Angle ANGLE = Rotations.of(0.653564);
                    public static final boolean ENCODER_INVERTED = false;
                    public static final boolean DRIVE_INVERTED = false;
                    public static final boolean ANGLE_INVERTED = true;
                }
                public static final class BR {
                    public static final Distance X = BASE_X.times(-1);
                    public static final Distance Y = BASE_Y.times(-1);
                    public static final Angle ANGLE = Rotations.of(0.246826);
                    public static final boolean ENCODER_INVERTED = false;
                    public static final boolean DRIVE_INVERTED = false;
                    public static final boolean ANGLE_INVERTED = true;
                }
            }
            public static final class PID {
                // for the drive motors on the modules
                public static final class Drive {
                    public static final double P = 6.5;
                    public static final double I = 0;
                    public static final double D = 0.03;
                    public static final double F = 0.2;
                    public static final double IZ = 0;
                }

                // for the steer motors on the modules
                public static final class Angle {
                    public static final double P = 80;
                    public static final double I = 0;
                    public static final double D = 0;
                    public static final double F = 0;
                    public static final double IZ = 0;
                }
            }
        }

        public static class CANIDs {
            public static final int IMU = 0;

            public static class Modules {
                public static final class FL {
                    public static final int DRIVE = 1;
                    public static final int STEER = 2;
                    public static final int CANCODER = 3;
                }

                public static final class FR {
                    public static final int DRIVE = 4;
                    public static final int STEER = 5;
                    public static final int CANCODER = 6;
                }

                public static final class BL {
                    public static final int DRIVE = 7;
                    public static final int STEER = 8;
                    public static final int CANCODER = 9;
                }

                public static final class BR {
                    public static final int DRIVE = 10;
                    public static final int STEER = 11;
                    public static final int CANCODER = 12;
                }
            }
        }
    }


    public static final class LimelightConstants {
        public static final double kPositionStdDevX = 0.1;
        public static final double kPositionStdDevY = 0.1;
        public static final double kPositionStdDevTheta = 10;
        public static final double kVisionStdDevY = 5;
        public static final double kVisionStdDevX = 5;
        public static final double kVisionStdDevTheta = 500;

        private static final AprilTagFieldLayout TAG_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        public final static HashMap<Integer, Pose2d> TAG_POSES =
            new HashMap<Integer, Pose2d>() {{
                for (int i = 0; i < TAG_LAYOUT.getTags().size(); ++i) {
                    if (TAG_LAYOUT.getTagPose(i + 1).isPresent())
                        put(i, TAG_LAYOUT.getTagPose(i + 1).get().toPose2d());
                }
            }};
    }

    public static final class TurretConstants {
        public static final class Launcher {
            public static final Distance WHEEL_DIAMETER = Inches.of(4);

            public static final boolean REVERSE = true;

            // in rotations/s
            public static final AngularVelocity MAXIMUM_VELOCITY =
                RotationsPerSecond.of(80);
            public static final AngularVelocity MINIMUM_VELOCITY =
                RotationsPerSecond.of(15);

            public static final AngularVelocity MAXIMUM_VELOCITY_ERROR = 
                RotationsPerSecond.of(1);

            public static final class PID {
                public static final double P = 27;
                public static final double I = 0.025;
                public static final double D = 0.06;
            }

            public static final class Feedforward {
                public static final double kS = 13;
                public static final double kV = 1;
            }

            public static final class VelocityRegression {
                public static final double A = 0.825772;
                public static final double B = -1.73633; // 0.33633; //-1.03633;
            }
        }

        public static final class Yaw {
            public static final double GEAR_RATIO = 45;
            public static final boolean REVERSE = false;

            // angular limits on yaw movement
            // 180 is pointed directly out the turret side
            public static final Angle ANGLE_MIN = Degrees.of(0);
            // public static final double ANGLE_MIN = 0;
            public static final Angle ANGLE_MAX = Degrees.of(340);

            // rps
            public static final double MAX_VELOCITY = 80;
            public static final double MAX_ACCELERATION= 800;


            public static final class Zeroing {
                public static final Current ROUGHPASS_CURRENT_LIMIT =
                    Amps.of(45);
                public static final Voltage ROUGHPASS_VOLTAGE = Volts.of(-3);

                public static final Current FINEPASS_CURRENT_LIMIT = Amps.of(35);
                public static final Voltage FINEPASS_VOLTAGE = Volts.of(-1);

                // time above current limit to register as hitting the hard limit
                public static final Time DEBOUNCE_TIME = Seconds.of(0.1);
            }

            public static final class PID {
                public static final double P = 20;
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

            public static final Angle ANGLE_CONSTANT = Degrees.of(51.5);

            // 90 would have the "face" of the turret as vertical
            // 0 would have the "face" be horizontal (outputting up)
            /* 
             * the minimum angle is dependent on the yaw
             * @param yaw - the angle of the turret yaw in degrees
             */
            public static Angle ANGLE_MIN(Angle yaw) {
                if (
                    Math.abs(
                        yaw.minus(
                            Degrees.of(180)
                        ).in(Degrees)
                    ) > 0
                ) {
                    // dependent on the yaw so we can shoot over the hopper
                    return Radians.of(
                        Math.atan(
                            HopperConstants.ABSOLUTE_HEIGHT
                            .plus(MetaConstants.Game.Fuel.RADIUS)
                            .minus(TurretConstants.Offsets.Z)
                            .div(
                                TurretConstants.Offsets.Y
                                .minus(HopperConstants.Offsets.Y)
                                .times(
                                    Math.tan(yaw.in(Radians))
                                )
                            ).in(Value)
                        )
                    );
                } else {
                    // physical hardstop of the hood
                    return Degrees.of(90);
                }
            }
            public static final Angle ANGLE_MAX = Degrees.of(180);

            public static final class PID {
                public static final double P = 1;
                public static final double I = 0;
                public static final double D = 0;
            }
        }

        public static final class CANIDs {
            public static final class Launcher {
                public static final int PORT = 17;
                public static final int STARBOARD = 15;
            }

            public static final int YAW = 14;

            public static final int PITCH = 41;
            public static final int PITCH_ENCODER = 40;
        }

        public static final class Offsets {
            public static final Angle PITCH_ENCODER = Rotations.of(0);

            // position relative to the center of the robot
            public static final Distance X = Inches.of(-8.625);
            public static final Distance Y = Inches.of(1.375);
            // Height of Shooter from Ground
            public static final Distance Z = Inches.of(9.375);

            public static final Translation3d TRANSLATION = new Translation3d(
                X,
                Y,
                Z
            );
        }

        public static final class TargetingOptimizer {
            public static final int INTERPOLATION_POINTS = 9;
            public static final int MAX_EVALUATIONS = 5000;

            public static final Time MAXIMUM_TIME = Seconds.of(5);
        }

    }

    public static final class ChoreoConstants {
        public static final File AUTO_PATH_DIRECTORY = new File(
            Filesystem.getDeployDirectory(),
            "choreo"
        );

        public static final class PID {
            public static final class Translation {
                public static final double P = 9.0;
                public static final double I = 0.0;
                public static final double D = 0.0;
            }

            public static final class Heading {
                public static final double P = 7.0;
                public static final double I = 0.0;
                public static final double D = 0.02;
            }
        }
    }

  public static final class IndexerConstants {
      public static final boolean REVERSE = true;
      public static final int CAN_ID = 24;
      public static final Dimensionless SPEED = Percent.of(80);
  }

  public static final class LoaderConstants {
      public static final boolean REVERSE = true;
      public static final int CAN_ID = 16;
      public static final AngularVelocity MAXIMUM_VELOCITY =
          RotationsPerSecond.of(100);

      public static final class PID {
        public static final double P = 1;
        public static final double I = 0;
        public static final double D = 0;
      }
  }
}
