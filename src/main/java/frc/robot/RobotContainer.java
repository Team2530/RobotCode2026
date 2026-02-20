package frc.robot;

import java.util.function.Consumer;

import com.fasterxml.jackson.databind.type.PlaceholderForType;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RunIndexerCommand;
import frc.robot.commands.RunLoaderCommand;
import frc.robot.commands.TurretCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePreset;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.subsystems.Limelight.LimelightType;
import frc.robot.subsystems.TurretSubsystem.TurretTargets;
import frc.robot.util.LimelightContainer;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LoaderSubsystem;

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

    @Logged
    public final SwerveSubsystem swerveDriveSubsystem = new SwerveSubsystem();

    // Autonomous chooser
    private final AutoChooser autoChooser = new AutoChooser();
    // private final LimeLightSubsystem limeLightSubsystem = new
    // LimeLightSubsystem();
    @Logged
    private final DriveCommand normalDrive = new DriveCommand(swerveDriveSubsystem, driverXbox.getHID());

    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
    private final LoaderSubsystem loaderSubsystem = new LoaderSubsystem();
    private final TurretSubsystem turretSubsystem = new TurretSubsystem(swerveDriveSubsystem);

    // public static final TurretSubsystem TURRET_SUBSYSTEM = new TurretSubsystem();
    /*
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        
        // Configure the trigger bindings
        configureBindings();

        DataLogManager.logNetworkTables(true);
        DataLogManager.start();

        swerveDriveSubsystem.setDefaultCommand(normalDrive);
        //turretSubsystem.setDefaultCommand(new TurretCommand(turretSubsystem));

        // NamedCommands.registerCommand(null, getAutonomousCommand());
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
        driverXbox.start()
            .onTrue(
                new InstantCommand(() -> {
                    swerveDriveSubsystem.resetOdometry();
                })
            );

        operatorXbox.leftTrigger(0.1)
            .whileTrue(
                new IntakeCommand(intakeSubsystem, IntakePreset.INTAKING, IntakePreset.OUT)
            );
        operatorXbox.leftBumper()
            .onTrue(
                new InstantCommand(
                    () -> {
                        intakeSubsystem.setPreset(
                            IntakePreset.STOWED
                        );
                    }
                )
            );

        operatorXbox.rightTrigger(0.3)
            .whileTrue(
                new RunLoaderCommand(loaderSubsystem)
            ).whileTrue(
                new RunIndexerCommand(indexerSubsystem)
            );

        
        operatorXbox.start()
            .onTrue(
                turretSubsystem.zeroYawCommand()
            );

        operatorXbox.povDown()
            .onTrue(
                new InstantCommand(()
                -> {
                    turretSubsystem.setTargetVelocity(
                        turretSubsystem.getTargetVelocity() - 1
                    );
                })
            );
        operatorXbox.povUp()
            .onTrue(
                new InstantCommand(()
                -> {
                    turretSubsystem.setTargetVelocity(
                        turretSubsystem.getTargetVelocity() + 1
                    );
                })
            );
        operatorXbox.back()
            .whileTrue(
                new RunLoaderCommand(
                    loaderSubsystem, 
                    true
                )
            ).whileTrue(
                new RunIndexerCommand(
                    indexerSubsystem, 
                    true
                )
            ).whileTrue(
                new IntakeCommand(
                    intakeSubsystem,
                    IntakePreset.SPITTING
                )
            );
        
        operatorXbox.x()
            .onTrue(
                new InstantCommand( 
                    () -> {
                        turretSubsystem.setTarget(
                            TurretTargets.SHUTTLE_LEFT
                        );
                    }
                )
            );
        operatorXbox.b()
            .onTrue(
                new InstantCommand( 
                    () -> {
                        turretSubsystem.setTarget(
                            TurretTargets.SHUTTLE_RIGHT
                        );
                    }
                )
            );
        operatorXbox.a()
            .onTrue(
                new InstantCommand( 
                    () -> {
                        turretSubsystem.setTarget(
                            TurretTargets.HUB
                        );
                    }
                )
            );
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

    public Command getInitCommand() {
        return turretSubsystem.zeroYawCommand();
    }
}
