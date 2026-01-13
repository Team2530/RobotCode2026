package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

public class Intake extends SubsystemBase {//Cloned From MT (NOT FINALIZED)

    

    private final SparkFlex m_intakeMotor;

    public Intake() {
        m_intakeMotor = new SparkFlex(Constants.INTAKE_MOTOR, MotorType.kBrushless);//TODO: Change to ACTUAL motor type
        
    }
    public void runIntake() {
      m_intakeMotor.set(Constants.INTAKE_SPEED);
    
    }//TODO: Update m_intakeMotor when fixed
      //After intake is touching the floor, activate the intake entry motor.
      // When released, Lift Intake and reverse motor for 2 secs at x2 speed. (NOT DONE)
}

