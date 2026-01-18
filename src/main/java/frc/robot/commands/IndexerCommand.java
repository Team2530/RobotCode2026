package frc.robot.commands;

import frc.robot.subsystems.IndexerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IndexerCommand extends Command {
    private final IndexerSubsystem m_IndexerSubsystem;

    public IndexerCommand(IndexerSubsystem indexerSubsystem) {
        m_IndexerSubsystem = indexerSubsystem;
        // Require the subsystem to prevent other commands from running on it concurrently
        addRequirements(indexerSubsystem);
    }
    
    // Called once when the command is initially scheduled.
    @Override
    public void initialize() {
        // Motor action is performed in execute() for continuous speed control, 
        // but we'll call the method here too for clarity.
    }

    // Called repeatedly while the command is scheduled.
    @Override
    public void execute() {
        m_IndexerSubsystem.runIndexer();

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // This is crucial: stop the motor when the command ends (i.e., button is released)
        m_IndexerSubsystem.stopIndexer();
    }

    // Returns true when the command should end.
    // We set this to false so the command only stops when interrupted by the button release.
    @Override
    public boolean isFinished() {
        return false;
    }
}

