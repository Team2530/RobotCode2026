// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Robot;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {
    private static final int LauncherID = Constants.TurretConstants.FlyWheelID;
    private static final double LauncherSpeed = Constants.TurretConstants.FlyWheelSpeed;

    private final SparkFlex m_Motor;
    

  /** Creates a new ExampleSubsystem. */  
  public LauncherSubsystem() {
    m_Motor = new SparkFlex(LauncherID, MotorType.kBrushless);
  }

void setCoastMode() {
  SparkMaxConfig config = new SparkMaxConfig();
  config.idleMode(IdleMode.kCoast);
}

public void runLauncher() {
  m_Motor.set(LauncherSpeed);
}
    
public void stopLauncher() {
  m_Motor.set(0.0);
}

}
