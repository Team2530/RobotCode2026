package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Robot;

public class ClimberSubsystem extends SubsystemBase {
    
    private final SparkFlex m_climbcommandMotor;
    private ClimbPresets climbPreset;

    public ClimberSubsystem() {
        m_climbcommandMotor = new SparkFlex(Constants.ClimbMotors.CLIMBSUBSYSTEM_MOTOR, MotorType.kBrushless);//TODO: Change to ACTUAL motor type
              
    }

    public void runIntake() {
        m_climbcommandMotor.set(Constants.ClimbMotors.CLIMB_SPEED);
            
    }
         
      public ClimbPresets getClimbPreset() {
        return this.climbPreset;
      }
      
      public enum ClimbPresets {
        CLIMBING,
        IDLE,
    }
}
