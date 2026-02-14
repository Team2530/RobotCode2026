package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem.IntakePreset;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    
    private final IntakeSubsystem subsystem;

    public IntakeCommand(IntakeSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setPreset(IntakePreset.INTAKING);
    }
     
    @Override
    public void end(boolean interrupted) {
        subsystem.setPreset(IntakePreset.STOWED);
    }
}
