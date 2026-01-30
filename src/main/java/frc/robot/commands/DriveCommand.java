package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;

@Logged
public class DriveCommand extends Command {
    @NotLogged
    private final SwerveSubsystem swerveSubsystem;
    private final XboxController driverXbox;

    private SlewRateLimiter dsratelimiter = new SlewRateLimiter(4);

    private double DRIVE_MULT = 1.0;
    private final double SLOWMODE_MULT = 0.25;

    public static enum DriveStyle {
        FIELD_ORIENTED,
        ROBOT_ORIENTED,
        INTAKE_ASSIST,
        SCORE_ASSIST
    };

    private DriveStyle driveStyle = DriveStyle.FIELD_ORIENTED;
    private PIDController rotationAssist = new PIDController(
            DriveConstants.ROTATION_ASSIST.kP,
            DriveConstants.ROTATION_ASSIST.kI,
            DriveConstants.ROTATION_ASSIST.kD);

    private PIDController translationAssist = new PIDController(
            DriveConstants.TRANSLATION_ASSIST.kP,
            DriveConstants.TRANSLATION_ASSIST.kI,
            DriveConstants.TRANSLATION_ASSIST.kD);

    private boolean isXstance = false;

    public DriveCommand(SwerveSubsystem swerveSubsystem, XboxController xbox) {
        this.swerveSubsystem = swerveSubsystem;
        this.driverXbox = xbox;

        rotationAssist.enableContinuousInput(-Math.PI, Math.PI);
        dsratelimiter.reset(SLOWMODE_MULT);

        addRequirements(swerveSubsystem);
    }

    double clamp(double v, double mi, double ma) {
        return (v < mi) ? mi : (v > ma ? ma : v);
    }

    public Translation2d DeadBand(Translation2d input, double deadzone) {
        double mag = input.getNorm();
        Translation2d norm = input.div(mag);

        if (mag < deadzone) {
            return new Translation2d(0.0, 0.0);
        } else {
            // TODO: Check is it sqrt2 or 1.0...
            Translation2d result = norm.times((mag - deadzone) / (1.0 - deadzone));
            return new Translation2d(
                    clamp(result.getX(), -1.0, 1.0),
                    clamp(result.getY(), -1.0, 1.0));
        }
    }

    public double DeadBand(double input, double deadband) {
        return Math.abs(input) < deadband ? 0.0 : (input - Math.signum(input) * deadband) / (1.0 - deadband);
    }

    @Override
    public void execute() {
        Translation2d xyRaw = new Translation2d(-driverXbox.getLeftY(), -driverXbox.getLeftX());
        double zSpeed = -MathUtil.applyDeadband(driverXbox.getRightX(), 0.1);
        double xSpeed = MathUtil.applyDeadband(xyRaw.getX(), 0.08); // xbox.getLeftX();
        double ySpeed = MathUtil.applyDeadband(xyRaw.getY(), 0.08); // xbox.getLeftY();

        xSpeed *= DriveConstants.XY_SPEED_LIMIT * DriveConstants.MAX_ROBOT_VELOCITY;
        ySpeed *= DriveConstants.XY_SPEED_LIMIT * DriveConstants.MAX_ROBOT_VELOCITY;
        zSpeed *= DriveConstants.Z_SPEED_LIMIT * DriveConstants.MAX_ROBOT_RAD_VELOCITY;

        double dmult = dsratelimiter
                .calculate((DRIVE_MULT - SLOWMODE_MULT) * driverXbox.getRightTriggerAxis() + SLOWMODE_MULT);
        xSpeed *= dmult;
        ySpeed *= dmult;
        zSpeed *= dmult;

        if (driverXbox.getStartButton()) {
            swerveSubsystem.zeroHeading();
            Translation2d pospose = swerveSubsystem.getOdometryPose().getTranslation();
            swerveSubsystem.odometry.resetPosition(swerveSubsystem.getGyroRotation2d(),
                    swerveSubsystem.getModulePositions(),
                    new Pose2d(pospose, new Rotation2d(FieldConstants.getAlliance() == Alliance.Blue ? 0.0 : Math.PI)));
        }

        double assistMixer = MathUtil.clamp(driverXbox.getLeftTriggerAxis(), 0.0, 1.0);
        SmartDashboard.putNumber("Assist Mixer", assistMixer);
        SmartDashboard.putString("Assist Mode", driveStyle.toString());

        ChassisSpeeds speeds = new ChassisSpeeds();
        switch (driveStyle) {
            case FIELD_ORIENTED:
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, zSpeed,
                        swerveSubsystem.getGyroRotation2d());
                break;
            case ROBOT_ORIENTED:
                speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                        xSpeed, ySpeed, zSpeed,
                        new Rotation2d(Math.PI));
                break;
            default:
                break;
        }

        // State transition logic
        isXstance = false;
        if (isXstance)
            isXstance = !((xyRaw.getNorm() > 0.08) && !driverXbox.getBButton());

        // Drive execution logic

        swerveSubsystem.zeroFeedforwards();
        if (isXstance) {
            swerveSubsystem.setXstance();
        } else {
            swerveSubsystem.setChassisSpeeds(speeds);
        }
    }

    public void setDriveStyle(DriveStyle style) {
        this.driveStyle = style;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopDrive();
    }

    public boolean isFinished() {
        return false;
    }
}