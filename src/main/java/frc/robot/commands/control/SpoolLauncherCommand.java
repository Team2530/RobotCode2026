package frc.robot.commands.control;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SpoolLauncherCommand extends Command {

    @Override
    public void execute() {
        RobotContainer.turretSubsystem.spoolLauncher();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.turretSubsystem.stopLauncher();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
