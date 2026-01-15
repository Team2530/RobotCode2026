package frc.robot.subsystems;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDrive;
import swervelib.telemetry.SwerveDrive.TelemetryVerbosity;

import frc.robot.Constants.DriveConstants;


public class YAGSLSubsystem extends SubsystemBase {

    private final SwerveDrive swerveDrive;


    public YAGSLSubsystem(File swerveConfigDirectory) {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(swerveConfigDirectory)
                .createSwerveDrive(
                    DriveConstants.MAX_ROBOT_VELOCITY,
                    new Pose2d()
                );
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        /* TODO: things the example code had - i need to research these options
         *    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
         *  swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
         *   swerveDrive.setModuleEncoderAutoSynchronize(false,
         *                                       1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
         * swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible
         */
        swerveDrive.setAngularVelocityCompenstation(
                DriveConstants.AngularCompenstation.ENABLE_IN_TELEOP,
                DriveConstants.AngularCompenstation.ENABLE_IN_AUTO,
                DriveConstants.AngularCompenstation.COMPENSATION_COEFFICIENT
        );

    };

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}

    /**
     * drive field-oriented
     * @param translation field-oriented translation; m / s
     * @param rotation angular rate; rads / s
     */
    public void drive(Translation2d translation, double rotation) {
        swerveDrive.drive(
            translation,
            rotation,
            false,
            false
        );
    }

    /**
     * drive robot-oriented
     * @param translation robot-oriented translation; m / s
     * @param rotation angular rate; rads / s
     */
    public void driveRobotRelative(
        Translation2d translation, 
        double rotation
    ) {
        swerveDrive.drive(
            translation,
            rotation,
            true,
            false
        );
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }
    
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void setMotorBrake(boolean isBraking) {
        swerveDrive.setMotorIdleMode(isBraking);
    }
}
