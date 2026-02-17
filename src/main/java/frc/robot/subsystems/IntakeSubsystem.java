// Copyright (c) CNrailfaner4039 and other contributors.
// Copyright (c) FIRST, 2530 contributors, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Robot;
//import frc.robot.commands.IntakeCommand;

public class IntakeSubsystem extends SubsystemBase {

    

    private final TalonFX m_intakesubsystemMotor;
    private final SparkMax m_intakerotationMotor;
    private IntakePresets intakePreset;
    
    public IntakeSubsystem() {
      m_intakesubsystemMotor = new TalonFX(Constants.IntakeMotors.INTAKESUBSYSTEM_MOTOR);
      m_intakerotationMotor = new SparkMax(Constants.IntakeMotors.INTAKE_PIVOT_MOTOR,MotorType.kBrushless);
    }
    public void runIntake() {
    m_intakesubsystemMotor.set(Constants.IntakeMotors.INTAKE_SPEED);
        
    }
          
    public void setIntakePreset(IntakePresets intake) {
      this.intakePreset = intake;

      if (IntakePresets.OUT == intake) {
        m_intakerotationMotor.set(Constants.IntakeMotors.INTAKE_PIVOT_SPEED);
        SmartDashboard.putString("Intake Subsystem", "Deployed");
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

