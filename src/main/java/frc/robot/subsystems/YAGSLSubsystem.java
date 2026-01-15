package frc.robot.subsystems;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;

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
}
