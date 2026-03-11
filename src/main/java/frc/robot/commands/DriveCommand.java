package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ControlConstants;

public class DriveCommand extends Command {

    private final SwerveSubsystem subsystem;
    private final XboxController driverXbox;

    private double headingOffset;

    /** Limit the speed of change */
    private SlewRateLimiter driveMultiplierSlewLimiter = new SlewRateLimiter(
            ControlConstants.DRIVE_MULTIPLIER_SLEW_RATE);

    public DriveCommand(
            SwerveSubsystem subsystem,
            XboxController driverXbox) {
        this.subsystem = subsystem;
        addRequirements(this.subsystem);

        this.driverXbox = driverXbox;

        driveMultiplierSlewLimiter.reset(
                ControlConstants.TURTLE_DRIVE_MULT);

        headingOffset = 0;
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
                ControlConstants.Deadband.X
            );
            double rawY = MathUtil.applyDeadband(
                -driverXbox.getLeftX(),
                ControlConstants.Deadband.Y
            );

            // to yagsl, a positive rotation value corresponds to a ccw 
            // rotation
            double x = (rawX * Math.cos(headingOffset)) - (rawY * Math.sin(headingOffset));
            double y = (rawX * Math.sin(headingOffset)) + (rawY * Math.cos(headingOffset));
            double z = MathUtil.applyDeadband(
                    -driverXbox.getRightX(),
                    ControlConstants.Deadband.Z);


            // trigger-base slow / fast mode
            double driveMultiplier = driveMultiplierSlewLimiter
                    .calculate((ControlConstants.REGULAR_DRIVE_MULT - ControlConstants.TURTLE_DRIVE_MULT)
                            * driverXbox.getRightTriggerAxis() + ControlConstants.TURTLE_DRIVE_MULT);

            x *= driveMultiplier;
            y *= driveMultiplier;
            z *= driveMultiplier;

            // convert to m / s
            x *= DriveConstants.MAX_ROBOT_VELOCITY;
            y *= DriveConstants.MAX_ROBOT_VELOCITY;
            z *= DriveConstants.MAX_ROBOT_RAD_VELOCITY;

            // send em off
            subsystem.drive(
                    new Translation2d(x, y),
                    z);
            SmartDashboard.putNumber(
                    "Swerve/DriveCommand/drive_multiplier",
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
