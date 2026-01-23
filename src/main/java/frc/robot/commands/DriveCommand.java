package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ControlConstants;

public class DriveCommand extends Command {

    private final SwerveSubsystem subsystem;
    private final XboxController driverXbox;

    private SlewRateLimiter driveMultiplierSlewLimiter = new SlewRateLimiter(
            ControlConstants.DRIVE_MULTIPLIER_SLEW_RATE
    );

    public DriveCommand(
        SwerveSubsystem subsystem,
        XboxController driverXbox
    ) {
        this.subsystem = subsystem;
        addRequirements(this.subsystem);

        this.driverXbox = driverXbox;
        
        driveMultiplierSlewLimiter.reset(
            ControlConstants.TURTLE_DRIVE_MULT
        );
    }

    @Override
    public void execute() {
        // get input values and apply deadband
        // these'll range form -1.0 to 1.0; we'll convert them to m/s later
        // x,y translate to correspond to literal x,y translation; z corresponds 
        // rotation
        double x = MathUtil.applyDeadband(
            driverXbox.getLeftY(),
            ControlConstants.Deadband.x
        );
        double y = MathUtil.applyDeadband(
            driverXbox.getLeftX(),
            ControlConstants.Deadband.y
        );
        double z = MathUtil.applyDeadband(
            driverXbox.getRightX(),
            ControlConstants.Deadband.z
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
            z
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
