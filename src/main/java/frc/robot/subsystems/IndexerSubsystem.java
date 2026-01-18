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

public class IndexerSubsystem extends SubsystemBase {
    private static final int IndexerID = Constants.IndexerConstants.IndexerID;
    private static final double IndexerSpeed = Constants.IndexerConstants.IndexerSpeed;

    private final SparkMax m_IndexerMotor;

  public IndexerSubsystem() {
    m_IndexerMotor = new SparkMax(IndexerID, MotorType.kBrushless);
    
  }

  void setCoastMode() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
  }

  public void runIndexer() {
    m_IndexerMotor.set(IndexerSpeed);
  }

  public void stopIndexer() {
    m_IndexerMotor.set(0.0);
  }

}

