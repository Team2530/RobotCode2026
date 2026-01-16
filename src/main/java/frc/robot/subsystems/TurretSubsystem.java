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
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Robot;

public class TurretSubsystem extends SubsystemBase {
    private static final int LauncherID = Constants.TurretConstants.FlyWheelID;
    private static final int TurretID = Constants.TurretConstants.TurretID;
    private static final int HoodID = Constants.TurretConstants.HoodID;
    private static final double LauncherSpeed = Constants.TurretConstants.FlyWheelSpeed;
    private static final double TurretGearRatio = Constants.TurretConstants.TurretGearRatio;
    private static final double Height = Constants.TurretConstants.ShooterHeight;

    // Get Distance and Robot Odometry. Needs to be updated for On The Fly calculations.
    //private static final double Distance = Limelight.getTargetDistance();
    //private static final double RobotSpeed = SwerveSubsystem.getVelocity().norm();

    // Motor types may need to change, for now they are set to Spark Flexes for Neo Vortex.
    private final SparkMax m_LauncherMotor;
    private final SparkMax m_TurretMotor;
    private final SparkMax m_HoodMotor;

    private final RelativeEncoder e_LauncherEncoder;
    private final RelativeEncoder e_TurretEncoder;
    private final RelativeEncoder e_HoodEncoder;

  /** Creates a new ExampleSubsystem. */  
  public TurretSubsystem() {
    m_LauncherMotor = new SparkMax(LauncherID, MotorType.kBrushless);
    m_TurretMotor = new SparkMax(TurretID, MotorType.kBrushless);
    m_HoodMotor = new SparkMax(HoodID, MotorType.kBrushless);
    
    e_LauncherEncoder = m_LauncherMotor.getEncoder();
    e_LauncherEncoder.setPosition(0);
    e_TurretEncoder = m_TurretMotor.getEncoder();
    e_TurretEncoder.setPosition(0);
    e_HoodEncoder = m_HoodMotor.getEncoder();
    e_HoodEncoder.setPosition(0);

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

  public double getTurretAngle() {
    double angle = e_TurretEncoder.getPosition() * (360.0 / TurretGearRatio);
    return (angle % 360 + 360) % 360;
  }

  public double getHoodAngle() {
    double angle = e_HoodEncoder.getPosition() * (360.0 / TurretGearRatio);
    return (angle % 360 + 360) % 360;
  }

  public double distance() {
    double theta = getHoodAngle();
    double velocity = getLauncherRPM() * (2 * Math.PI) / 60; // Convert RPM to m/s
    double a = -4.9, b = Math.tan(theta), c = velocity * velocity * Height * Math.cos(theta * theta);
    double determinent = b * b - 4 * a * c;
    double root1 = (-b + Math.sqrt(determinent)) / (2 * a);
    return root1;
  }

  public void angleTurret(double angle) {
    while (getTurretAngle() < angle) {
      m_TurretMotor.set(0.01);
    }
    while (getTurretAngle() > angle) {
      m_TurretMotor.set(-0.01);
    }
    m_TurretMotor.set(0.0);
  }
}

