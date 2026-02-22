package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.util.swerve.Requester;
import frc.robot.util.swerve.RotationSocket;
import frc.robot.util.swerve.TranslationSocket;
import frc.robot.util.swerve.RotationSocket.RotationRequest;
import frc.robot.util.swerve.TranslationSocket.TranslationRequest;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ControlConstants;

public class DriveCommand extends Command {

    private final TranslationSocket translationSocket;
    private final RotationSocket rotationSocket;

    private final XboxController driverXbox;

    /** Limit the speed of change */
    private SlewRateLimiter driveMultiplierSlewLimiter = new SlewRateLimiter(
            ControlConstants.DRIVE_MULTIPLIER_SLEW_RATE
    );


    private boolean isRequestingActive;
    private TranslationRequest translationRequest;
    private RotationRequest rotationRequest;

    public DriveCommand(
        TranslationSocket translationSocket,
        RotationSocket rotationSocket,
        XboxController driverXbox
    ) {
        this.translationSocket = translationSocket;
        this.rotationSocket = rotationSocket;

        this.driverXbox = driverXbox;
        
        driveMultiplierSlewLimiter.reset(
            ControlConstants.TURTLE_DRIVE_MULT
        );
    }

    @Override
    public void initialize() {
        possessTranslation();
        possessRotation();
    }

    private void possessTranslation() {
        translationSocket.possess(
            this,
            new Requester<TranslationRequest>() {
                @Override
                public boolean isRequestingActive() {
                    return isRequestingActive;
                }

                // weird ahh syntax but whatever
                @Override
                public TranslationRequest getRequest() {
                    return translationRequest;
                };
            }
        );

    }

    private void possessRotation() {
        rotationSocket.possess(
            this,
            new Requester<RotationRequest>() {
                @Override
                public boolean isRequestingActive() {
                    return isRequestingActive;
                };

                @Override
                public RotationRequest getRequest() {
                    return rotationRequest;
                };
            }
        );
    }

    @Override
    public void execute() {
        if (driverXbox.getRightBumperButtonPressed()) {
            subsystem.xStance();
        } else {
            // get input values and apply deadband
            // these'll range form -1.0 to 1.0; we'll convert them to m/s later
            // x,y translate to correspond to literal x,y translation; z corresponds 
            // rotation
            double x = MathUtil.applyDeadband(
                driverXbox.getLeftY(),
                ControlConstants.Deadband.X
            );
            double y = MathUtil.applyDeadband(
                driverXbox.getLeftX(),
                ControlConstants.Deadband.Y
            );
            double z = MathUtil.applyDeadband(
                driverXbox.getRightX(),
                ControlConstants.Deadband.Z
            );
            
            // trigger-base slow / fast mode
            double driveMultiplier = (
                driveMultiplierSlewLimiter.calculate(
                    driverXbox.getRightTriggerAxis()
                ) * (
                    ControlConstants.REGULAR_DRIVE_MULT 
                    - ControlConstants.TURTLE_DRIVE_MULT
                )
            ) + ControlConstants.TURTLE_DRIVE_MULT;
        // get input values and apply deadband
        // these'll range form -1.0 to 1.0; we'll convert them to m/s later
        // x,y translate to correspond to literal x,y translation; z corresponds 
        // rotation
        double x = MathUtil.applyDeadband(
            driverXbox.getLeftY(),
            ControlConstants.Deadband.X
        );
        double y = MathUtil.applyDeadband(
            driverXbox.getLeftX(),
            ControlConstants.Deadband.Y
        );
        double z = MathUtil.applyDeadband(
            driverXbox.getRightX(),
            ControlConstants.Deadband.Z
        );
        double trigger = MathUtil.applyDeadband(
            driverXbox.getRightTriggerAxis(),
            ControlConstants.Deadband.TRIGGER
        );
        
        // trigger-base slow / fast mode
        double driveMultiplier = (
            driveMultiplierSlewLimiter.calculate(
                trigger
            ) * (
                ControlConstants.REGULAR_DRIVE_MULT 
                - ControlConstants.TURTLE_DRIVE_MULT
            )
        ) + ControlConstants.TURTLE_DRIVE_MULT;

            x *= driveMultiplier;
            y *= driveMultiplier;
            z *= driveMultiplier;

            // convert to m / s
            x *= DriveConstants.MAX_ROBOT_VELOCITY;
            y *= DriveConstants.MAX_ROBOT_VELOCITY;
            z *= DriveConstants.MAX_ROBOT_RAD_VELOCITY;

            // send em off
            translationRequest = translationSocket
                .new AbsoluteTranslationRequest(
                    new Translation2d(
                        x,
                        y
                    )
                );
            SmartDashboard.putNumber(
                "Swerve/DriveCommand/drive_multiplier",
                driveMultiplier
            );
        }
        
        if (z > 0) {
            if (!rotationSocket.isPossessed()) {
                possessRotation();
            }
            rotationRequest = rotationSocket
                .new RelativeRotationRequest(
                    new Rotation2d(z)
                );
        } else {
            rotationSocket.depossess();
        }
        
        isRequestingActive = trigger > 0;
        SmartDashboard.putNumber(
            "Swerve/DriveCommand/drive_multiplier",
            driveMultiplier
        );
    }

    @Override
    public void end(boolean interrupted) {
        // TODO: semantics?
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
