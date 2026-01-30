package frc.robot.commands;

import frc.robot.subsystems.LoaderSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


/**
 * A command that runs the intake at a set speed for collection or ejection.
 */
public class LoaderCommand extends Command {
    private final LoaderSubsystem m_loaderSubsystem;

    /**
     * Creates a new RunIntakeCommand.
     *
     * @param intakeSubsystem The subsystem this command will run on.
     * @param isReversed If true, the motor will run in reverse (eject).
     */
    public LoaderCommand(LoaderSubsystem loaderSubsystem) {
        m_loaderSubsystem = loaderSubsystem;
        // Require the subsystem to prevent other commands from running on it concurrently
        addRequirements(loaderSubsystem);
    }

    // Called once when the command is initially scheduled.
    @Override
    public void initialize() {
        // Motor action is performed in execute() for continuous speed control, 
        // but we'll call the method here too for clarity
        m_loaderSubsystem.runLoader();
    }

    // Called repeatedly while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // This is crucial: stop the motor when the command ends (i.e., button is released)
        m_loaderSubsystem.stopLoader();
    }

    // Returns true when the command should end.
    // We set this to false so the command only stops when interrupted by the button release.
    @Override
    public boolean isFinished() {
        return false;
    }
}

//Modifyed by 2530 Team 30 