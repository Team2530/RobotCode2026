// Copyright (c) FIRST, 2530 contributors, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.CAN;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import frc.robot.Robot;
// import frc.robot.commands.ClimberCommand;
// import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;

public class LauncherSubsystem extends SubsystemBase {
    private static final int LauncherID = Constants.TurretConstants.FlyWheelID;
    private static final double LauncherSpeed = Constants.TurretConstants.FlyWheelSpeed;
    public static SwerveSubsystem SwerveSubsystem;

    private final TalonFX m_Motor;
    
    

  /** Creates a new ExampleSubsystem. */  
  public LauncherSubsystem() {
    m_Motor = new TalonFX(LauncherID);
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

  public void initalize() { //Helps make trigger keys
    if (output) {
      runLauncher();
    }
    else {
      stopLauncher();
    }
  }
  
  private static boolean output;
    
  public static void setOutput(boolean output) {
    LauncherSubsystem.output = output;
  } 
  public boolean getOutput() {
    return output;
  }
  
}
