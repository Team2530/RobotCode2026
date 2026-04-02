package frc.robot.commands.control;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MetaConstants;

public class DriveCommand extends Command {

    private final SwerveSubsystem subsystem;
    private final XboxController driverXbox;

    private double headingOffset;

    /** Limit the speed of change */
    private SlewRateLimiter driveMultiplierSlewLimiter = new SlewRateLimiter(
            DriveConstants.Control.MULTIPLIER_SLEW_RATE.in(Percent.per(Seconds))
        );

    public DriveCommand(
            SwerveSubsystem subsystem,
            XboxController driverXbox) {
        this.subsystem = subsystem;
        addRequirements(this.subsystem);

        this.driverXbox = driverXbox;

        driveMultiplierSlewLimiter.reset(
                DriveConstants.Control.TURTLE_DRIVE_MULT.in(Value)
        );

        headingOffset = MetaConstants.isRed.getAsBoolean()
            ? Math.PI
            : 0;
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
            //
            // apparently, a forward on the joystick (relative to the 
            // controller) corresponds to a negative value
            double rawX = MathUtil.applyDeadband(
                -driverXbox.getLeftY(),
                DriveConstants.Control.Deadband.X
            );
            double rawY = MathUtil.applyDeadband(
                -driverXbox.getLeftX(),
                DriveConstants.Control.Deadband.Y
            );

            // to yagsl, a positive rotation value corresponds to a ccw 
            // rotation
            double x = (rawX * Math.cos(headingOffset)) - (rawY * Math.sin(headingOffset));
            double y = (rawX * Math.sin(headingOffset)) + (rawY * Math.cos(headingOffset));
            double z = MathUtil.applyDeadband(
                    -driverXbox.getRightX(),
                    DriveConstants.Control.Deadband.Z);


            // trigger-base slow / fast mode
            double driveMultiplier;
            if (driverXbox.getLeftTriggerAxis() > 0.1) {
                driveMultiplier = (
                            DriveConstants.Control.TURTLE_DRIVE_MULT
                            .minus(DriveConstants.Control.ROCK_DRIVE_MULT)
                        ).times(
                            Value.of(1 - driverXbox.getLeftTriggerAxis())
                        ).plus(DriveConstants.Control.ROCK_DRIVE_MULT)
                        .in(Value);
                z *= DriveConstants.Control.TURTLE_DRIVE_MULT.in(Value);
            } else {
                driveMultiplier = driveMultiplierSlewLimiter.calculate(
                        (
                            DriveConstants.Control.REGULAR_DRIVE_MULT
                            .minus(DriveConstants.Control.TURTLE_DRIVE_MULT)
                        ).times(
                            Value.of(driverXbox.getRightTriggerAxis())
                        ).plus(DriveConstants.Control.TURTLE_DRIVE_MULT)
                        .in(Value)
                    );
                z *= driveMultiplier;
            }

            x *= driveMultiplier;
            y *= driveMultiplier;

            // convert to m / s
            x *= DriveConstants.MAX_ROBOT_VELOCITY.in(MetersPerSecond);
            y *= DriveConstants.MAX_ROBOT_VELOCITY.in(MetersPerSecond);
            z *= DriveConstants.MAX_ROBOT_RAD_VELOCITY.in(RadiansPerSecond);

            // send em off
            subsystem.drive(
                    new Translation2d(x, y),
                    z);
            SmartDashboard.putNumber(
                    "swerve/DriveCommand/drive_multiplier",
                    driveMultiplier);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // TODO: semantics?
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void resetHeading() {
        headingOffset = subsystem.getRotation().getRadians();
    }
}
