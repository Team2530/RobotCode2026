package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommand;

public class IntakeSubsystem extends SubsystemBase {// Cloned From MT (NOT FINALIZED)

    

    private final SparkFlex m_intakesubsytemMotor;
    private IntakePresets intakePreset;
    
    public IntakeSubsystem() {
      m_intakesubsytemMotor = new SparkFlex(Constants.INTAKESUBSYSTEM_MOTOR, MotorType.kBrushless);//TODO: Change to ACTUAL motor type
            
    }
    public void runIntake() {
    m_intakesubsytemMotor.set(Constants.INTAKE_SPEED);
        
    }//TODO: Update m_intakeMotor when fixed
      //After intake is touching the floor, activate the intake entry motor.
      // When released, Lift Intake and reverse motor for 2 secs at x2 speed. (NOT DONE)
          
    public void setIntakePreset(IntakePresets intake) {
      this.intakePreset = intake;
    }
    
       
    public IntakePresets getIntakePreset() {
      return this.intakePreset;
    }
    
    public enum IntakePresets {
      INTAKE
  }
}

