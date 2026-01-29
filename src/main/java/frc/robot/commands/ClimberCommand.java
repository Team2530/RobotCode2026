package frc.robot.commands;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.RobotContainer;

public class ClimberCommand extends Command {
    private final ClimberSubsystem subsystem1;
    // private XboxController = driverXbox; TODO: Not working, find a better solution
    // this.driverXbox = xbox;
    

    private ClimbPresets climbPreset;

    public ClimberCommand(ClimberSubsystem subsystem1) {
        this.subsystem1 = subsystem1;
        addRequirements(subsystem1);
    }
    

     @Override
    public void end(boolean interrupted) {
        subsystem1.setClimbPreset(ClimbPresets.IDLE);
        SmartDashboard.putString("Climb Ended", "Ended");
    }

     @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule();
            subsystem1.setClimbPreset(ClimbPresets.CLIMBINGUP); //TODO: Sequence command group this
            SmartDashboard.putString("Climb Command", "Started Part 1 Climb");
            new WaitCommand(Constants.Sleep.up);
            SmartDashboard.putString("Climb Command", "Finished Part 1 Climb");
            subsystem1.setClimbPreset(ClimbPresets.CLIMBINGDOWN);
            new WaitCommand(Constants.Sleep.finishUP);
            SmartDashboard.putString("Climb Command", "Finished Climb");
          

    
        
        
        

        if (Robot.isSimulation()) {
            SmartDashboard.putBoolean("[SIM] Climbed", true);
        
        }
        if (!driverXbox.getAButton()) {

        }
    }
    
    public ClimbPresets getClimbPreset() {
        return this.climbPreset;
    }
      
    public enum ClimbPresets {
        CLIMBINGUP,
        CLIMBINGUPF,
        CLIMBINGDOWN,
        IDLE,
    }
}
