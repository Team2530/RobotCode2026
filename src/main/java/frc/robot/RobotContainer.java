// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.IntakeSubsystem.IntakePresets;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

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

    // TODO: ADD Limelights
    // private static final Limelight LL_name = new Limelight(LimelightType.LL4, "limelight-name", true, true);

    // TODO: initalize Limelight container
    // @Logged
    // public static final LimelightContainer LLContainer = new LimelightContainer(LL_name1, LL_name2, LL_name3);

    // @Logged
    public final CommandXboxController driverXbox = new CommandXboxController(
            ControllerConstants.DRIVER_CONTROLLER_PORT);
    // @Logged
    public final CommandXboxController operatorXbox = new CommandXboxController(
            ControllerConstants.OPERATOR_CONTROLLER_PORT);

    private final SendableChooser<Command> autoChooser;

    @Logged
    public final SwerveSubsystem swerveDriveSubsystem = new SwerveSubsystem();

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

        swerveDriveSubsystem.configurePathplanner();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        autoChooser.onChange(new Consumer<Command>() {
            @Override
            public void accept(Command t) {
                if (t instanceof PathPlannerAuto) {
                    PathPlannerAuto auto = (PathPlannerAuto) t;
                    swerveDriveSubsystem.setAutoStartingPose(auto.getStartingPose());
                }
            }
        });
    }

    private IntakePresets selectedIntakePreset = IntakePresets.IDLE;
    private IntakePresets lockedIntake = IntakePresets.IDLE;


    private void IntakePreset(IntakePresets preset) {
        selectedIntakePreset = preset;
        SmartDashboard.putString("Locked Fuel Preset", preset.toString());
    }
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

       //
       operatorXbox.y().onTrue(new InstantCommand(() -> {
          selectedIntakePreset = IntakePresets.INTAKE;
        }));


    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        swerveDriveSubsystem.setGyroToEstimate();
        return autoChooser.getSelected();
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