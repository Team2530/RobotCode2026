package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.ClimberCommand.ClimbPresets;

public class ClimberSubsystem extends SubsystemBase {
    
    private final SparkFlex m_climbsubsystemMotor;
    private ClimbPresets climbPreset;
    
      public ClimberSubsystem() {
          m_climbsubsystemMotor = new SparkFlex(Constants.ClimbMotors.CLIMBSUBSYSTEM_MOTOR, MotorType.kBrushless);//TODO: Change to ACTUAL motor type
                
      }
    public void setClimbPreset(ClimbPresets climb) {
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

    public void runClimb() {
        m_climbsubsystemMotor.set(Constants.ClimbMotors.CLIMB_SPEED);
            
    }
      
}
