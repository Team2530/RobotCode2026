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
    private static final int SpindexerID = Constants.IndexerConstants.IndexerID;
    private static final double SpindexerSpeed = Constants.IndexerConstants.IndexerSpeed;

    private static final int LoaderID = Constants.IndexerConstants.LoaderID;
    private static final double LoaderSpeed = Constants.IndexerConstants.LoaderSpeed;

    private final SparkMax m_SpindexerMotor;
    private final SparkMax m_LoaderMotor;

  public IndexerSubsystem() {
    m_SpindexerMotor = new SparkMax(SpindexerID, MotorType.kBrushless);
    m_LoaderMotor = new SparkMax(LoaderID, MotorType.kBrushless);

  }

  void setCoastMode() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
  }

  public void runIndexer() {
    m_SpindexerMotor.set(SpindexerSpeed);
    m_LoaderMotor.set(LoaderSpeed);
  }

  public void stopIndexer() {
    m_SpindexerMotor.set(0.0);
    m_LoaderMotor.set(0.0);
  }

}

