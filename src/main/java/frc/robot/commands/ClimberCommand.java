package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbPresets;

public class ClimberCommand extends Command {
    private final ClimberSubsystem subsystem1;

    

    public ClimberCommand(ClimberSubsystem subsystem1) {
        this.subsystem1 = subsystem1;
        addRequirements(subsystem1);
    }

     @Override
    public void end(boolean interrupted) {
        subsystem1.setIntakePreset(ClimbPresets.IDLE);
        SmartDashboard.putString("Climb Ended", "Ended");
    }

     @Override
    public void initialize() {
        subsystem1.setIntakePreset(ClimbPresets.CLIMBING);
        SmartDashboard.putString("Climb Command", "Started");
        

        if (Robot.isSimulation()) {
            SmartDashboard.putBoolean("[SIM] Climbed", true);
        
        }
    }
}
