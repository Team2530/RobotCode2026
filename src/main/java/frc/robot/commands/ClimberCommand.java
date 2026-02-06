package frc.robot.commands;

// import com.revrobotics.spark.SparkFlex;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import frc.robot.commands.DriveCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ClimberSubsystem;
//import frc.robot.RobotContainer;

/*
 * FILE POSSIBLE WILL BE DEPRICATED FOR TOURNAMENT
 *  -PRIORITY SWITCH, HOLD TILL FURTHER NOTICE-
 */
public class ClimberCommand extends Command {
    private final ClimberSubsystem subsystem1;
    

    private ClimbPresets climbPreset;

    public ClimberCommand(ClimberSubsystem subsystem1) {
        this.subsystem1 = subsystem1;
        addRequirements(subsystem1);
    }
    

     @Override
    public void end(boolean interrupted) {
        subsystem1.setClimbPreset(ClimbPresets.IDLE);
        SmartDashboard.putString("Climb Ended", "End Fail");
    }

     @Override
    public void initialize() {
        if (output == 1) {
            subsystem1.setClimbPreset(ClimbPresets.CLIMBINGUP); //TODO: Sequence command group this, I'm not sure how
            SmartDashboard.putString("Climb Command", "Started Part 1 Climb");
            new WaitCommand(Constants.Sleep.up);
            SmartDashboard.putString("Climb Command", "Finished Part 1 Climb");
            subsystem1.setClimbPreset(ClimbPresets.CLIMBINGDOWN);
            new WaitCommand(Constants.Sleep.finishUP);
            SmartDashboard.putString("Climb Command", "Finished Climb");
                
                if (Robot.isSimulation()) {
                    SmartDashboard.putBoolean("[SIM] Climbed", true);
                }
            new WaitUntilCommand(driveXbox.a());
            subsystem1.setClimbPreset(ClimbPresets.CLIMBINGUP);
            SmartDashboard.putString("Climb Command", "Started Part 1 Going Down");
            new WaitCommand(Constants.Sleep.down);
            SmartDashboard.putString("Climb Command", "Started Part 2 Going Down, Please Reverse to clear the bar!!!!!!!!!!!");
            subsystem1.setClimbPreset(ClimbPresets.CLIMBINGDOWN);
            new WaitCommand(Constants.Sleep.finishDN);
            subsystem1.setClimbPreset(ClimbPresets.IDLE);
            SmartDashboard.putString("Climb Command", "Congrats, you successfully unclimbed from the bar! You can return with RB.");
            ClimberCommand.setOutput(0);
        }
        if (output == 0) {
            SmartDashboard.putString("Climb Command", "Idle");
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
    public CommandXboxController getDriverXbox() {
        return getDriverXbox();
    }
    private CommandXboxController driveXbox;
    
    
    private static double output;
    
    public static void setOutput(double output) {
        ClimberCommand.output = output;
    } 
    public double getOutput() {
        return output;
    }
}
