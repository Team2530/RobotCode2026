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
    private static final double Height = Constants.TurretConstants.ShooterHeight - Constants.TurretConstants.GoalHeight;
    private static final double LimitL = Constants.TurretConstants.TurretLimitLow;
    private static final double LimitR = Constants.TurretConstants.TurretLimitHigh;

    // Get Distance and Robot Odometry. Needs to be updated for On The Fly calculations.

    //private static final double Distance = Limelight.getTargetDistance();
    //private static final double RobotSpeed = SwerveSubsystem.getVelocity().norm();

    // Motor types may need to change, for now they are set to Spark Maxes for Neo 1/2/550.

    private final SparkMax m_LauncherMotor;
    private final SparkMax m_TurretMotor;
    private final SparkMax m_HoodMotor;

    private final RelativeEncoder e_LauncherEncoder;
    private final RelativeEncoder e_TurretEncoder;
    private final RelativeEncoder e_HoodEncoder;

  public TurretSubsystem() {
    // Initialize Motors and Encoders
    m_LauncherMotor = new SparkMax(LauncherID, MotorType.kBrushless);
    m_TurretMotor = new SparkMax(TurretID, MotorType.kBrushless);
    m_HoodMotor = new SparkMax(HoodID, MotorType.kBrushless);
    
    e_LauncherEncoder = m_LauncherMotor.getEncoder();
    e_LauncherEncoder.setPosition(0);
    e_TurretEncoder = m_TurretMotor.getEncoder();
    e_TurretEncoder.setPosition(0.5);
    e_HoodEncoder = m_HoodMotor.getEncoder();
    e_HoodEncoder.setPosition(0);

  }

  void setCoastMode() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
  }


  private double NormalizeAngle(double a) {
      a %= 360;
      if (a < 0) a += 360; // Turn any angle into [0, 360)
      return a;
  }

  private double getTurretAngle() {
    double angle = e_TurretEncoder.getPosition() * (360.0 / TurretGearRatio);
    return NormalizeAngle(angle);
  }

  private double getHoodAngle() {
    double angle = e_HoodEncoder.getPosition() * (360.0 / TurretGearRatio);
    return NormalizeAngle(angle);
  }

  private double targetHoodAngle() {
    // Placeholder for actual calculation based on distance to target
    double distance = 0; // Distance to Closest Part of the Goal
    double h_dif = Math.abs(Height); // ABS(Goal Height - Shooter Height)
    double bias = 0; // Placeholder  Bias
    double angle = Math.atan(1/(h_dif / (2 * distance))) + bias; // Calculate the prefered angle.
    return angle;
  }

  private double targetShootSpeed() {
    // Calculate Required Exit Velocity based on Distance and Hood Angle
    double distance = 0;// Distance to Closest Part of the Goal
    double thetaAngles = targetHoodAngle(); // Hood Angle in Degrees
    double thetaRadians = Math.toRadians(thetaAngles);
    double g = 9.81; // Gravity in m/s^2
    double h_dif = Math.abs(Height); // ABS(Goal Height - Shooter Height)
    double bias = 2.5;

    double exitVelocity = (Math.sqrt((g * distance * distance) / ((2 * Math.cos(thetaRadians) * Math.cos(thetaRadians)) * (distance * Math.tan(thetaRadians) - h_dif)))) + 2.5;
    return exitVelocity;
  }

  public void runLauncher() {
    m_LauncherMotor.set(LauncherSpeed);
  }
      
  public void stopLauncher() {
    m_LauncherMotor.set(0.0);
  }

  public void angleTurret(double targetAngle) {
    double TurretSpeed = 0.1;
    double Angle0 = getTurretAngle();
    
    if (targetAngle < LimitL) {
      //targetAngle = LimitL;
    }
  }
}

