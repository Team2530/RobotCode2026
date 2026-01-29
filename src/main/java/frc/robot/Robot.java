// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.urcl.URCL;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
@Logged(strategy = Strategy.OPT_IN)
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  @Logged
  private RobotContainer m_robotContainer;

  public static SendableChooser<String> autoChooser = new SendableChooser<>();

  double lastLoopTime = Timer.getFPGATimestamp();
  @Logged
  double loopTime = 0.02;
  @Logged
  double commandSchedulerTime = 0.02;

  DoublePublisher loopPub = NetworkTableInstance.getDefault()
      .getDoubleTopic("loopTime").publish();
  DoublePublisher csTimePublisher = NetworkTableInstance.getDefault()
      .getDoubleTopic("commandSchedulerTime").publish();

  public Robot() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    // Pheonix 6 Signal Logging
    SignalLogger.start();

    // URCL (REV) Logging
    if (Constants.CommonConstants.LOG_TO_NETWORKTABLES) {
      URCL.start();
    } else {
      URCL.start(DataLogManager.getLog());
    }

    Epilogue.configure(config -> {
      config.backend = new FileBackend(DataLogManager.getLog());
      config.minimumImportance = Logged.Importance.DEBUG;
    });

    Epilogue.bind(this);
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    SmartDashboard.putData(autoChooser);

    // Put git/code version metadata on networktables
    NetworkTable versionTable = NetworkTableInstance.getDefault().getTable("Version");
    versionTable.putValue("GIT_SHA", NetworkTableValue.makeString(BuildConstants.GIT_SHA));
    versionTable.putValue("BUILD_DATE", NetworkTableValue.makeString(BuildConstants.BUILD_DATE));
    versionTable.putValue("GIT_BRANCH", NetworkTableValue.makeString(BuildConstants.GIT_BRANCH));
    versionTable.putValue("DIRTY", NetworkTableValue.makeBoolean(BuildConstants.DIRTY != 0));

    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    double startTime = Timer.getFPGATimestamp();
    CommandScheduler.getInstance().run();

    double currentTime = Timer.getFPGATimestamp();
    loopTime = currentTime - lastLoopTime;
    commandSchedulerTime = currentTime - startTime;
    lastLoopTime = currentTime;

    loopPub.set(loopTime);
    csTimePublisher.set(commandSchedulerTime);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.getSwerveSubsystem().stopDrive();
  }

  @Override
  public void disabledPeriodic() {

  }


 
  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
       CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {

  }
}