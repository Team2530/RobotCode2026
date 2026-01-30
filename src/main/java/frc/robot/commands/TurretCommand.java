package frc.robot.commands;

import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command that runs the intake at a set speed for collection or ejection.
 */
public class TurretCommand extends Command {
    private final TurretSubsystem m_LauncherSubsystem;

    /**
     * Creates a new RunIntakeCommand.
     *
     * @param intakeSubsystem The subsystem this command will run on.
     * @param isReversed If true, the motor will run in reverse (eject).
     */
    public TurretCommand(TurretSubsystem LauncherSubsystem) {
        m_LauncherSubsystem = LauncherSubsystem;
        // Require the subsystem to prevent other commands from running on it concurrently
        addRequirements(LauncherSubsystem);
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
        m_LauncherSubsystem.runLauncher();

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // This is crucial: stop the motor when the command ends (i.e., button is released)
        m_LauncherSubsystem.stopLauncher();
    }

    // Returns true when the command should end.
    // We set this to false so the command only stops when interrupted by the button release.
    @Override
    public boolean isFinished() {
        return false;
    }
}

