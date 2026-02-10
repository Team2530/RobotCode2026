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
    
    @Override
    public void initialize() {
        m_IndexerSubsystem.run();
    }

    @Override
    public void end(boolean interrupted) {
        // This is crucial: stop the motor when the command ends (i.e., button is released)
        m_IndexerSubsystem.stop();
    }

    // Returns true when the command should end.
    // We set this to false so the command only stops when interrupted by the button release.
    @Override
    public boolean isFinished() {
        return false;
    }
}

