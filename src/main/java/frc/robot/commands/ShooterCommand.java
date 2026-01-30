package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command that runs the intake at a set speed for collection or ejection.
 */
public class ShooterCommand extends Command {
    private final ShooterSubsystem m_ShooterSubsystem;

    /**
     * Creates a new RunIntakeCommand.
     *
     * @param intakeSubsystem The subsystem this command will run on.
     * @param isReversed If true, the motor will run in reverse (eject).
     */
    public ShooterCommand(ShooterSubsystem shooterSubsystem) {
        m_ShooterSubsystem = shooterSubsystem;

        // Require the subsystem to prevent other commands from running on it concurrently
        addRequirements(shooterSubsystem);
    }

    // Called once when the command is initially scheduled.
    @Override
    public void initialize() {
        // Motor action is performed in execute() for continuous speed control, 
        // but we'll call the method here too for clarity.
        m_ShooterSubsystem.runShooter();
    }

    // Called repeatedly while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // This is crucial: stop the motor when the command ends (i.e., button is released)
        m_ShooterSubsystem.stopShooter();
    }

    // Returns true when the command should end.
    // We set this to false so the command only stops when interrupted by the button release.
    @Override
    public boolean isFinished() {
        return false;
    }
}

//Modifyed by 2530 Team 30 