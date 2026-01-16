package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem.IntakePresets;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    
    private final IntakeSubsystem subsystem;

    

    public IntakeCommand(IntakeSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

     @Override
    public void end(boolean interrupted) {
        
    }

     @Override
    public void initialize() {
        subsystem.setIntakePreset(IntakePresets.INTAKE);
        SmartDashboard.putString("Intake Command", "Started");
        

        if (Robot.isSimulation()) {
            SmartDashboard.putBoolean("[SIM] Collected", true);
        
        }
    }
}
