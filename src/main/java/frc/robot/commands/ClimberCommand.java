package frc.robot.commands;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbPresets;

public class ClimberCommand extends Command {
    private final ClimberSubsystem subsystem1;
    private final ClimberCommand command1;

    private final SparkFlex m_climbsubsystemMotor;
    private ClimbPresets climbPreset;

    public ClimberCommand(ClimberSubsystem subsystem1) {
        this.subsystem1 = command1;
        addRequirements();
    }

     @Override
    public void end(boolean interrupted) {
        subsystem1.setClimbPreset(ClimbPresets.IDLE);
        SmartDashboard.putString("Climb Ended", "Ended");
    }

     @Override
    public void initialize() {
        subsystem1.setClimbPreset(ClimbPresets.CLIMBING);
        SmartDashboard.putString("Climb Command", "Started");
        

        if (Robot.isSimulation()) {
            SmartDashboard.putBoolean("[SIM] Climbed", true);
        
        }
    }
    public void setupPreset(ClimberSubsystem.ClimbPresets climb) {
        this.climbPreset = climb;
        boolean climbActive = climb == ClimbPresets.CLIMBING;
        boolean climbInactive = climb == ClimbPresets.IDLE;
        if (ClimbPresets.CLIMBING == climb) {

          m_climbsubsystemMotor.set(Constants.ClimbMotors.CLIMB_SPEED);
          SmartDashboard.putString("Climbing Subsystem", "Begin");

          new WaitCommand(Constants.Sleep.first); //TODO: find a way to make timer, DONE

          m_climbsubsystemMotor.set(0);
          //wait for button press. if auto, wait some time.
          if (DriverStation.isAutonomousEnabled()) {
            new WaitCommand(Constants.Sleep.first);
          }
          else {
            //Button press
          }
          m_climbsubsystemMotor.set(-Constants.ClimbMotors.CLIMB_SPEED);

          new WaitCommand(Constants.Sleep.last); 

          SmartDashboard.putString("Climbing Command", "Finished");
          
  
          if (Robot.isSimulation()) {
              SmartDashboard.putBoolean("[SIM] Climbing", true);
          
          }
        }
        if (SmartDashboard.putString("Climbing Command", "Finished")) {
          m_climbsubsystemMotor.set(0);
          SmartDashboard.putString("Climbing Command", "Ended");
              if (Robot.isSimulation()) {
              SmartDashboard.putBoolean("[SIM] Climbing", false);
          
          }
        }
      }
}
