package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

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
    // -- Warning: Irreverable when climbing -- \\
    public void setClimbPreset(ClimbPresets climb) {
          this.climbPreset = climb;
      
      if (ClimbPresets.CLIMBINGUP == climb) {

        m_climbsubsystemMotor.set(Constants.ClimbMotors.CLIMB_SPEED);
        SmartDashboard.putString("Climbing Subsystem", "Begin");
        
        if (ClimbPresets.CLIMBINGDOWN == climb) {
          m_climbsubsystemMotor.set(0);
          if (DriverStation.isAutonomousEnabled()==false) {
            //Button press
          }
          m_climbsubsystemMotor.set(-Constants.ClimbMotors.CLIMB_SPEED);
          SmartDashboard.putString("Climbing Command", "Lowering to finish Climb");
          if (ClimbPresets.CLIMBINGUPF == climb) {
            m_climbsubsystemMotor.set(0);
          }
        }
         

        SmartDashboard.putString("Climbing Command", "Finished");
        

        if (Robot.isSimulation()) {
            SmartDashboard.putBoolean("[SIM] Climbing", false);
        
        }
      }
      if (SmartDashboard.putString("Climbing Command", "Finished")) {
        m_climbsubsystemMotor.set(0);
        SmartDashboard.putString("Climbing Command", "Ended");
            if (Robot.isSimulation()) {
            SmartDashboard.putBoolean("[SIM] Climbing", false);
        
        }
      }
      if (ClimbPresets.CLIMBINGDOWN == climb) {
        m_climbsubsystemMotor.set(Constants.ClimbMotors.CLIMB_SPEED);
        
      }
    }

    public void runClimb() {
        m_climbsubsystemMotor.set(Constants.ClimbMotors.CLIMB_SPEED);
            
    }
      
}
