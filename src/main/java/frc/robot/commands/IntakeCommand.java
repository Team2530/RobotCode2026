package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem.IntakePreset;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    
    private final IntakeSubsystem subsystem;
    private final IntakePreset onStart;
    private final IntakePreset onEnd;

    public IntakeCommand(IntakeSubsystem subsystem) {
        this(
            subsystem,
            IntakePreset.INTAKING,
            IntakePreset.STOWED
        );
    }

    public IntakeCommand(
        IntakeSubsystem subsystem,
        IntakePreset preset
    ) {
        this(
            subsystem,
            preset,
            subsystem.getPreset()
        );
    }
    
    public IntakeCommand(
        IntakeSubsystem subsystem,
        IntakePreset onStart,
        IntakePreset onEnd
    ) {
        this.subsystem = subsystem;
        this.onStart = onStart;
        this.onEnd = onEnd;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setPreset(onStart);
    }
     
    @Override
    public void end(boolean interrupted) {
        subsystem.setPreset(onEnd);
    }
}
