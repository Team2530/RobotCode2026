package frc.robot.commands.control;

import frc.robot.subsystems.IndexerSubsystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class RunIndexerCommand extends Command {
    private final IndexerSubsystem indexerSubsystem;
    private final boolean isReversed;

    public RunIndexerCommand(
        IndexerSubsystem subsystem,
        boolean reversed
    ) {
        this.indexerSubsystem = subsystem;
        this.isReversed = reversed;
        // Require the subsystem to prevent other commands from running on it concurrently
        addRequirements(indexerSubsystem);
        
    }

    public RunIndexerCommand(IndexerSubsystem indexerSubsystem) {
        this(
            indexerSubsystem,
            false
        );
    }
    
    @Override
    public void initialize() {
        indexerSubsystem.run(isReversed);
    }

    @Override
    public void end(boolean interrupted) {
        // This is crucial: stop the motor when the command ends (i.e., button is released)
        indexerSubsystem.stop();
    }

    // Returns true when the command should end.
    // We set this to false so the command only stops when interrupted by the button release.
    @Override
    public boolean isFinished() {
        return false;
    }
}

