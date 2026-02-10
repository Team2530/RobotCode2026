package frc.robot.commands;

import frc.robot.subsystems.LoaderSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RunLoaderCommand extends Command {
    private final LoaderSubsystem loaderSubsystem;

    public RunLoaderCommand(LoaderSubsystem loaderSubsystem) {
        this.loaderSubsystem = loaderSubsystem;
        addRequirements(loaderSubsystem);
    }
    
    @Override
    public void initialize() {
        loaderSubsystem.run();
    }

    @Override
    public void end(boolean interrupted) {
        loaderSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

