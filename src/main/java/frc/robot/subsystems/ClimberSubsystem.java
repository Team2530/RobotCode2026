package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Robot;

public class ClimberSubsystem extends SubsystemBase {
    
    private final SparkFlex m_climbsubsystemMotor;
    private ClimbPresets climbPreset;

    public ClimberSubsystem() {
        m_climbsubsystemMotor = new SparkFlex(Constants.ClimbMotors.CLIMBSUBSYSTEM_MOTOR, MotorType.kBrushless);//TODO: Change to ACTUAL motor type
              
    }

    public void runIntake() {
        m_climbsubsystemMotor.set(Constants.ClimbMotors.CLIMB_SPEED);
            
    }
    public void setIntakePreset(ClimbPresets climb) {
        this.climbPreset = climb;
        boolean climbActive = climb == ClimbPresets.CLIMBING;
        boolean climbInactive = climb == ClimbPresets.IDLE;
        if (ClimbPresets.CLIMBING == climb) {
          m_climbsubsystemMotor.set(Constants.ClimbMotors.CLIMB_SPEED);
          SmartDashboard.putString("Climbing Subsystem", "Begin");
          sleep(Constants.Sleep.first); //TODO: find a way to make timer
          m_climbsubsystemMotor.set(0);
          //wait for button press. if auto, wait some time.
          m_climbsubsystemMotor.set(-Constants.ClimbMotors.CLIMB_SPEED);
          //wait some time.
          SmartDashboard.putString("Climbing Subsystem", "Finished");
          
  
          if (Robot.isSimulation()) {
              SmartDashboard.putBoolean("[SIM] Climbing", true);
          
          }
        }
        if (SmartDashboard.putString("Climbing Subsystem", "Finished")) {
          m_climbsubsystemMotor.set(0);
          SmartDashboard.putString("Climbing Subsystem", "Ended");
          
  
          if (Robot.isSimulation()) {
              SmartDashboard.putBoolean("[SIM] Climbing", false);
          
          }
        }
      }
      
         
      public ClimbPresets getClimbPreset() {
        return this.climbPreset;
      }
      
      public enum ClimbPresets {
        CLIMBING,
        IDLE,
    }
}
