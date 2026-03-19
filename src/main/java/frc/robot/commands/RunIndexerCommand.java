package frc.robot.commands;

import frc.robot.subsystems.IndexerSubsystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class RunIndexerCommand extends Command {
    private final IndexerSubsystem indexerSubsystem;
    private final BooleanSupplier conditional;
    private final boolean isReversed;

    public RunIndexerCommand(
        IndexerSubsystem subsystem,
        BooleanSupplier conditional,
        boolean reversed
    ) {
        this.indexerSubsystem = subsystem;
        this.conditional = conditional;
        this.isReversed = reversed;
        // Require the subsystem to prevent other commands from running on it concurrently
        addRequirements(indexerSubsystem);
        
    }

    public RunIndexerCommand(
        IndexerSubsystem subsystem,
        BooleanSupplier conditional
    ) {
        this(
            subsystem,
            conditional,
            false
        );
    }

    public RunIndexerCommand(
        IndexerSubsystem indexerSubsystem,
        boolean isReversed
    ) {
        this(
            indexerSubsystem,
            new BooleanSupplier() {
                @Override
                public boolean getAsBoolean() {
                    return true;
                }
            },
            isReversed
        );
    }

    public RunIndexerCommand(IndexerSubsystem indexerSubsystem) {
        this(
            indexerSubsystem,
            false
        );
    }
    
    @Override
    public void execute() {
        if (conditional.getAsBoolean()) {
            indexerSubsystem.run(isReversed);
        } else {
            indexerSubsystem.stop();
        }
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

