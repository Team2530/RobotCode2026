package frc.robot;

import java.util.function.BooleanSupplier;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualTurretCommand;
import frc.robot.commands.RunIndexerCommand;
import frc.robot.commands.RunLoaderCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePreset;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.LimelightType;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.TurretSubsystem.TurretTargets;
import frc.robot.util.LimelightContainer;

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
    private static final Limelight LL_BT = new Limelight(LimelightType.LL4, "limelight-bt", false, true);
    private static final Limelight LL_BR = new Limelight(LimelightType.LL4, "limelight-br", true, true);
    private static final Limelight LL_FR = new Limelight(LimelightType.LL4, "limelight-fr", true, true);
    private static final Limelight LL_BL = new Limelight(LimelightType.LL4, "limelight-bl", true, true);

    //initalizing limelight container (Group)
    public static final LimelightContainer LLContainer = new LimelightContainer(LL_BL, LL_BR, LL_FR, LL_BT); // remove the turret limelight, should not be used for odometry.
    // @Logged
    public static final CommandXboxController driverXbox = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    // @Logged
    public static final CommandXboxController operatorXbox = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

    @Logged
    public static final SwerveSubsystem swerveDriveSubsystem = new SwerveSubsystem();
    public static final AutoChooser autoChooser = new AutoChooser();
    // LimeLightSubsystem();
    @Logged
    private static final DriveCommand normalDrive = new DriveCommand(swerveDriveSubsystem, driverXbox.getHID());

    public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public static final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
    public static final RunLoaderSubsystem loaderSubsystem = new LoaderSubsystem();
    public static final TurretSubsystem turretSubsystem = new TurretSubsystem(
        swerveDriveSubsystem,
        new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return operatorXbox.getHID()
                    .getBackButtonPressed();
            }
        }
    );

    // public static final TurretSubsystem TURRET_SUBSYSTEM = new TurretSubsystem();
    private final AutoFactory autoFactory = new AutoFactory(
          swerveDriveSubsystem::getPose, // A function that returns the current robot pose
          swerveDriveSubsystem::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
          swerveDriveSubsystem::followTrajectory, // The drive subsystem trajectory follower 
          true, // If alliance flipping should be enabled 
          swerveDriveSubsystem // The drive subsystem
    );

    private final String[] autos = {"NewAuto"};

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
        for (String trajName : autos) {
            autoChooser.addRoutine(trajName+"_routine", () -> {
                AutoRoutine routine = autoFactory.newRoutine(trajName+"_routine");
                AutoTrajectory trajectory = routine.trajectory(trajName);

                routine.active().onTrue(
                    Commands.sequence(
                        trajectory.resetOdometry(),
                        trajectory.cmd()
                    )
                );

                // Add all event marker triggers here: https://choreo.autos/choreolib/auto-factory/#using-autoroutine

                

                return routine;
            });
        }
        SmartDashboard.putData(autoChooser);
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
                    LLContainer.snapToVision(swerveDriveSubsystem.getSwerveDrive());
                })
            );
        driverXbox.back()
            .onTrue(
                getResetCommand()
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
        .and(
            new BooleanSupplier() {
                @Override
                public boolean getAsBoolean() {
                    return turretSubsystem.isAtVelocity();
                }
            }
        )
            .whileTrue(
                new ParallelCommandGroup(
                    new RunLoaderCommand(loaderSubsystem),
                    new RunIndexerCommand(indexerSubsystem)
                )
            );

        
        operatorXbox.rightBumper()
            .whileTrue(
                new InstantCommand(
                    () -> {
                        turretSubsystem.setTarget(TurretTargets.HUB);
                    }
                )
            );
        operatorXbox.x()
            .whileTrue(
                new InstantCommand(
                    () -> {
                        turretSubsystem.setTarget(TurretTargets.SHUTTLE_LEFT);
                    }
                )
            );
        operatorXbox.b()
            .whileTrue(
                new InstantCommand(
                    () -> {
                        turretSubsystem.setTarget(TurretTargets.SHUTTLE_RIGHT);
                    }
                )
            );
        operatorXbox.start()
            .onTrue(
                turretSubsystem.zeroYawCommand()
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
        
        
        ManualTurretCommand turretCommand = new ManualTurretCommand(
            turretSubsystem,
            operatorXbox.getHID()
        );
        operatorXbox.y()
                .onTrue(turretCommand);
        operatorXbox.povUp()
                .onTrue(
                    new InstantCommand(
                        () -> {
                            turretCommand.increaseVelocity();
                        }
                    )
                );
        operatorXbox.povDown()
                .onTrue(
                    new InstantCommand(
                        () -> {
                            turretCommand.decreaseVelocity();
                        }
                    )
                );
        
        operatorXbox.x()
            .onTrue(
                new InstantCommand( 
                    () -> {
                        turretSubsystem.setManualControl(52, 45);
                    }
                )
            );
        operatorXbox.b()
            .onTrue(
                new InstantCommand( 
                    () -> {
                        turretSubsystem.setManualControl(329, 42.5);
                    }
                )
            );
        operatorXbox.a()
            .onTrue(
                new InstantCommand( 
                    () -> {
                        turretSubsystem.setManualControl(0, 48.5);
                    }
                )
            );
        operatorXbox.leftStick()
            .onTrue(
                new InstantCommand(
                    () -> {
                        turretSubsystem.setManualControl(0, 0);
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

    public Command getResetCommand() {
        return new ParallelCommandGroup(
            turretSubsystem.resetCommand()
        );
    }
}
