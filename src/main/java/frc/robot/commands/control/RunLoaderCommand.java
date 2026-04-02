package frc.robot.commands.control;

import static edu.wpi.first.units.Units.*;

import frc.robot.subsystems.LoaderSubsystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class RunLoaderCommand extends Command {
    private final LoaderSubsystem loaderSubsystem;

    private BooleanSupplier conditional;

    private boolean m_Reversed;

    public RunLoaderCommand(LoaderSubsystem loaderSubsystem, boolean Reversed) {
        this(
            loaderSubsystem,
            new BooleanSupplier() {
                @Override
                public boolean getAsBoolean() {
                    return true;
                }
            },
            Reversed
        );
    }

    public RunLoaderCommand(LoaderSubsystem loaderSubsystem) {
        this(loaderSubsystem, false);
    }

    public RunLoaderCommand(
        LoaderSubsystem subsystem,
        BooleanSupplier conditional,
        boolean reversed
    ) {
        this.loaderSubsystem = subsystem;
        this.conditional = conditional;
        this.m_Reversed = reversed;
        
        addRequirements(loaderSubsystem);
    }
    
    @Override
    public void execute() {
        if (conditional.getAsBoolean()) {
            if (m_Reversed){
                loaderSubsystem.run(Percent.of(100));
            }
            else{
                loaderSubsystem.run(Percent.of(-100));
            }
        } else {
            loaderSubsystem.stop();
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

