package frc.robot.commands;

import frc.robot.subsystems.LoaderSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RunLoaderCommand extends Command {
    private final LoaderSubsystem loaderSubsystem;

    private boolean m_Reversed;

    public RunLoaderCommand(LoaderSubsystem loaderSubsystem, boolean Reversed) {
        this.loaderSubsystem = loaderSubsystem;
        addRequirements(loaderSubsystem);
        m_Reversed = Reversed;
    }

    public RunLoaderCommand(LoaderSubsystem loaderSubsystem) {
        this(loaderSubsystem, false);
    }
    
    @Override
    public void initialize() {
        if (m_Reversed){
            loaderSubsystem.runVelocity(-100);
        }
        else{
            loaderSubsystem.runVelocity(100);
        }
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

