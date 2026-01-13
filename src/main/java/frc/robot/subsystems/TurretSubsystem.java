// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkFlexExternalEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import frc.robot.Robot;

public class TurretSubsystem extends SubsystemBase {
    private static final int LauncherID = Constants.TurretConstants.FlyWheelID;
    private static final int TurretID = Constants.TurretConstants.TurretID;
    private static final double LauncherSpeed = Constants.TurretConstants.FlyWheelSpeed;
    private static final double TurretGearRatio = Constants.TurretConstants.TurretGearRatio;

    // Motor types may need to change, for now they are set to Spark Flexes for Neo Vortex.
    private final SparkMax m_LauncherMotor;
    private final SparkMax m_TurretMotor;

    private final RelativeEncoder e_LauncherEncoder;

  /** Creates a new ExampleSubsystem. */  
  public TurretSubsystem() {
    m_LauncherMotor = new SparkMax(LauncherID, MotorType.kBrushless);
    m_TurretMotor = new SparkMax(TurretID, MotorType.kBrushless);
    
    e_LauncherEncoder = m_LauncherMotor.getEncoder();
    e_LauncherEncoder.setPosition(0);

    Shuffleboard.getTab("Launcher")
      .addNumber("RPM", this::getLauncherRPM)
      .withWidget("Graph");

  }

  void setCoastMode() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
  }


  public double getLauncherRPM() {
    return e_LauncherEncoder.getVelocity();
  }

  public void runLauncher() {
    m_LauncherMotor.set(LauncherSpeed);
  }
      
  public void stopLauncher() {
    m_LauncherMotor.set(0.0);
  }

  public double getLauncherAngle() {
    return e_LauncherEncoder.getPosition() * (360.0 / TurretGearRatio);
  }

  public void runTurret(double angle) {
    while (getLauncherAngle() < angle) {
      m_TurretMotor.set(0.01);
    }
    while (getLauncherAngle() > angle) {
      m_TurretMotor.set(-0.01);
    }
    m_TurretMotor.set(0.0);
  }
}

