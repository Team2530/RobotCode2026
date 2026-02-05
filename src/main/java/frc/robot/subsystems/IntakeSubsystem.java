package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.IntakeCommand;

public class IntakeSubsystem extends SubsystemBase {// Cloned From MT (NOT FINALIZED)

    

    private final SparkFlex m_intakesubsystemMotor;
    private final SparkFlex m_intakerotationMotor;
    private IntakePresets intakePreset;
    
    public IntakeSubsystem() {
      m_intakesubsystemMotor = new SparkFlex(Constants.IntakeMotors.INTAKESUBSYSTEM_MOTOR, MotorType.kBrushless);//TODO: Change to ACTUAL motor type
      m_intakerotationMotor = new SparkFlex(Constants.IntakeMotors.INTAKE_PIVOT_MOTOR, MotorType.kBrushless);
    }
    public void runIntake() {
    m_intakesubsystemMotor.set(Constants.IntakeMotors.INTAKE_SPEED);
        
    }//TODO: Update m_intakeMotor when fixed
      //After intake is touching the floor, activate the intake entry motor.
      // When released, Lift Intake and reverse motor for 2 secs at x2 speed. (NOT DONE)
          
    public void setIntakePreset(IntakePresets intake) {
      this.intakePreset = intake;
      
      if (IntakePresets.OUT == intake) {
        m_intakerotationMotor.set(Constants.IntakeMotors.INTAKE_PIVOT_SPEED);
        new WaitCommand(1);// TODO: Change so that it got to the right position
        m_intakerotationMotor.set(0);
        if (IntakePresets.INTAKE == intake) {
        m_intakesubsystemMotor.set(Constants.IntakeMotors.INTAKE_SPEED);
        SmartDashboard.putString("Intake Subsystem", "Begin");
        

        if (Robot.isSimulation()) {
            SmartDashboard.putBoolean("[SIM] Intaking", true);
        
        }
        }
      }
      if (IntakePresets.IDLE == intake) {
        m_intakesubsystemMotor.set(0);
        SmartDashboard.putString("Intake Subsystem", "Ended");
        m_intakerotationMotor.set(-Constants.IntakeMotors.INTAKE_PIVOT_SPEED);
        new WaitCommand(1);
        m_intakerotationMotor.set(0);

        if (Robot.isSimulation()) {
            SmartDashboard.putBoolean("[SIM] Intaking", false);
        
        }
      }
      if (IntakePresets.INTAKECLEAR == intake) {
        m_intakesubsystemMotor.set(-Constants.IntakeMotors.INTAKE_SPEED);
      }
    }
    
       
    public IntakePresets getIntakePreset() {
      return this.intakePreset;
    }
    
    public enum IntakePresets {
      OUT,
      INTAKECLEAR,
      INTAKE,
      IDLE,
  }
}

