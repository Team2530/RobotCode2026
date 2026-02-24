// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import choreo.auto.AutoChooser;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.subsystems.Limelight.LimelightType;
import frc.robot.util.LimelightContainer;
import frc.robot.subsystems.IntakeSubsystem.IntakePresets;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class RobotContainer {
    // These are initating the individual Limlight(s). The name should match the limelight internal names.
    private static final Limelight LL_BR = new Limelight(LimelightType.LL4, "limelight-br", true, true);
    private static final Limelight LL_FR = new Limelight(LimelightType.LL4, "limelight-fr", true, true);
    private static final Limelight LL_BL = new Limelight(LimelightType.LL4, "limelight-bl", true, true);
    private static final Limelight LL_BF = new Limelight(LimelightType.LL4, "limelight-bf", true, true);

    //initalizing limelight container (Group)
    public static final LimelightContainer LLContainer = new LimelightContainer(LL_BF, LL_BL, LL_BR, LL_FR);
    // @Logged
    public final CommandXboxController driverXbox = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    // @Logged
    public final CommandXboxController operatorXbox = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

    //Subsystems
    @Logged
    public final SwerveSubsystem swerveDriveSubsystem = new SwerveSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    // Autonomous chooser
    private final AutoChooser autoChooser = new AutoChooser();
    // private final LimeLightSubsystem limeLightSubsystem = new
    // LimeLightSubsystem();
    @Logged
    private final DriveCommand normalDrive = new DriveCommand(swerveDriveSubsystem, driverXbox.getHID());

    

    /*
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        
        // Configure the trigger bindings
        configureBindings();

        DataLogManager.logNetworkTables(true);
        DataLogManager.start();

        

        swerveDriveSubsystem.setDefaultCommand(normalDrive);

        // NamedCommands.registerCommand(null, getAutonomousCommand());
    }
    private boolean intakeDown = false;
    
    //TODO: Fix this with grouped commands 
    

   
    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
       //This is ment for operator controls

       //Intake down and intaking.
        operatorXbox.leftTrigger().whileTrue(new SequentialCommandGroup(
           new InstantCommand(() -> {
            
            if (!intakeDown) {  
                //putting intake down here
                
            }
            intakeDown = true;
        
            }, intakeSubsystem))).whileTrue(new IntakeCommand(intakeSubsystem));
        

        //shooting, indexer, and turret.
        operatorXbox.rightTrigger().whileTrue(new ParallelCommandGroup(
            //indexer, turret, shooter here.
        ));
        
        //Intake up.
        operatorXbox.leftBumper().whileTrue(new InstantCommand(() -> {
            //put intake up here
            intakeDown = false;
        }, intakeSubsystem));

        //shuffling.
        operatorXbox.b().whileTrue(new ParallelCommandGroup(
            //SHUFFLINGGGGGGGG
            
            ));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();
    }

    public SwerveSubsystem getSwerveSubsystem() {
        return swerveDriveSubsystem;
    }

    public CommandXboxController getDriverXbox() {
        return driverXbox;
    }

    public CommandXboxController getOperatorXbox() {
        return operatorXbox;
    }
}
    