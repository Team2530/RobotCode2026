package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;

import choreo.Choreo;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.MetaConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.control.DriveCommand;
import frc.robot.commands.control.IntakeCommand;
import frc.robot.commands.control.ManualTurretCommand;
import frc.robot.commands.control.RunIndexerCommand;
import frc.robot.commands.control.RunLoaderCommand;
import frc.robot.commands.util.HubStatusCommand;
import frc.robot.commands.util.MatchtimeStatusCommand;
import frc.robot.commands.util.ShiftAlertingCommand;
import frc.robot.commands.util.VoltageStatusCommand;
import frc.robot.Constants.DriveConstants;
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
    private static final Limelight LL_FL = new Limelight(
            LimelightType.LL4, 
            "limelight-fl", 
            true, 
            true,
            new Pose3d(
                new Translation3d(
                    Meters.of(-0.31513),
                    Meters.of(-0.25669),
                    Meters.of(0.261315)
                ),
                new Rotation3d(
                    Degrees.of(-1.6),
                    Degrees.of(-14.8),
                    Degrees.of(55)
                )
            )
        );
    private static final Limelight LL_FR = new Limelight(
            LimelightType.LL4, 
            "limelight-fr", 
            true, 
            true,
            new Pose3d(
                new Translation3d(
                    Meters.of(-0.31729),
                    Meters.of(0.268869),
                    Meters.of(0.408635)
                ),
                new Rotation3d(
                    Degrees.of(0),
                    Degrees.of(-14.30),
                    Degrees.of(-38)
                )
            )
        );
    private static final Limelight LL_BL = new Limelight(
            LimelightType.LL4, 
            "limelight-bl", 
            true, 
            true,
            new Pose3d(
                new Translation3d(
                    Meters.of(-0.1919),
                    Meters.of(-0.25003),
                    Meters.of(0.351561)
                ),
                new Rotation3d(
                    Degrees.of(3.4),
                    Degrees.of(-13),
                    Degrees.of(160.7)
                )
            )
        );

    //initalizing limelight container (Group)
    public static final LimelightContainer LLContainer = new LimelightContainer(LL_BL, LL_FL, LL_FR); // remove the turret limelight, should not be used for odometry.
    // @Logged
    public static final CommandXboxController driverXbox = new CommandXboxController(MetaConstants.Controllers.DRIVER_PORT);
    // @Logged
    public static final CommandXboxController operatorXbox = new CommandXboxController(MetaConstants.Controllers.OPERATOR_PORT);

    @Logged
    public static final SwerveSubsystem swerveDriveSubsystem = new SwerveSubsystem();
    // the factory class for this (AutoBuilder) needs to be configured first
    // before the auto chooser can be built, so this is created in the
    // constructor
    public static final SendableChooser<Command> autoChooser = new SendableChooser<>();
    // LimeLightSubsystem();
    @Logged
    public static final DriveCommand normalDrive = new DriveCommand(swerveDriveSubsystem, driverXbox.getHID());

    public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public static final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
    public static final LoaderSubsystem loaderSubsystem = new LoaderSubsystem();
    public static final TurretSubsystem turretSubsystem = new TurretSubsystem();

    /*
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    static {
        // Configure the trigger bindings
        configureBindings();

        configureAutos();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        swerveDriveSubsystem.setDefaultCommand(normalDrive);
        //turretSubsystem.setDefaultCommand(new TurretCommand(turretSubsystem));

        // i'm running out of names
        configureCommands();
    }

    /*
     * This method schedules and configures all miscellaneous commands / actions
     */
    private static void configureCommands() {
        SmartDashboard.putString(
            "Meta/Match_Type",
            DriverStation.getMatchType().toString()
        );
        RobotModeTriggers.teleop()
            .onTrue(
                new ParallelCommandGroup(
                    turretSubsystem.zeroYawCommand(),

                    new HubStatusCommand(),

                    new ShiftAlertingCommand(
                        driverXbox.getHID(),
                        Seconds.of(0.5)
                    )
                )
            );

        RobotModeTriggers.autonomous()
            .onTrue(
                new ParallelCommandGroup(
                    new MatchtimeStatusCommand(),
                    new VoltageStatusCommand()
                )
            );
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
    private static void configureBindings() {
        driverXbox.start()
            .onTrue(
                new InstantCommand(() -> {
                    LLContainer.snapToVision(swerveDriveSubsystem);
                    normalDrive.resetHeading();
                })
            );
        driverXbox.leftBumper()
            .onTrue(
                new InstantCommand(() -> {
                    LLContainer.snapToVision(swerveDriveSubsystem);
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
            new ParallelCommandGroup(
                new IntakeCommand(
                    intakeSubsystem,
                    IntakePreset.AGITATING
                ),
                new RunLoaderCommand(loaderSubsystem),
                new RunIndexerCommand(indexerSubsystem)
            )
        );

        operatorXbox.rightBumper()
            .onTrue(
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
        operatorXbox.rightStick()
            .onTrue(
                    new InstantCommand(
                        () -> {
                            LLContainer.snapToVision(swerveDriveSubsystem);
                        }
                    )
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
            swerveDriveSubsystem,
            operatorXbox.getHID()
        );
        
        operatorXbox.y().onTrue(turretCommand);

        // SHUTTLE
        operatorXbox.a()
            .onTrue(
                new InstantCommand( 
                    () -> {
                        turretSubsystem.setManualControl(
                            Degrees.of(0), 
                            RotationsPerSecond.of(45)
                        );
                        //turretSubsystem.setTarget(TurretTargets.HUB);
                    }
                )
            );

        // DEPOT
        operatorXbox.povDown()
            //.onTrue(turretCommand);
            .onTrue(
                new InstantCommand(
                    () -> {
                        turretSubsystem.setManualControl(
                            Degrees.of(62), 
                            RotationsPerSecond.of(37.5)
                        );
                    }
                )
            );


        // KILL
        operatorXbox.leftStick()
            .onTrue(
                new InstantCommand(
                    () -> {
                        turretSubsystem.stop();
                    }
                )
            );

        // LEFT TRENCH  
        operatorXbox.povLeft()
            .onTrue(
                new InstantCommand(
                    () -> {
                        turretSubsystem.setManualControl(
                            Degrees.of(295), 
                            RotationsPerSecond.of(36)
                        );
                    }
                )
            );

        // RIGHT TRENCH
        operatorXbox.povRight()
            .onTrue(
                new InstantCommand(
                    () -> {
                        turretSubsystem.setManualControl(
                            Degrees.of(69),
                            RotationsPerSecond.of(35)
                        );
                    }
                )
            );

        
    }


    private static void configureAutos() {
        AutoBuilder.configure(
                swerveDriveSubsystem::getPose,
                swerveDriveSubsystem::resetOdometry,
                swerveDriveSubsystem::getRobotRelativeVelocity,
                swerveDriveSubsystem::driveRobotRelative,
                new SwerveSubsystem.AutonomousController(),
                new RobotConfig(
                    RobotConstants.TOTAL_MASS,
                    RobotConstants.MOMENT_OF_INERTIA,
                    new ModuleConfig(
                        DriveConstants.Modules.WHEEL_DIAMETER.div(2),
                        DriveConstants.MAX_ROBOT_VELOCITY,
                        DriveConstants.Modules.WHEEL_FRICTION_COEFFICIENT,
                        DCMotor.getKrakenX60Foc(1),
                        swerveDriveSubsystem.getDriveGearRatio(),
                        DriveConstants.Modules.DRIVE_CURRENT_LIMIT,
                        1
                    ),
                    new Translation2d(
                        DriveConstants.Modules.Offsets.FL.X,
                        DriveConstants.Modules.Offsets.FL.Y
                    ),
                    new Translation2d(
                        DriveConstants.Modules.Offsets.FR.X,
                        DriveConstants.Modules.Offsets.FR.Y
                    ),
                    new Translation2d(
                        DriveConstants.Modules.Offsets.BL.X,
                        DriveConstants.Modules.Offsets.BL.Y
                    ),
                    new Translation2d(
                        DriveConstants.Modules.Offsets.BR.X,
                        DriveConstants.Modules.Offsets.BR.Y
                    )
                ),
                MetaConstants.isRed,
                swerveDriveSubsystem
            );

        // add named commands for the paths
        Map<String, Command> namedCommands = new HashMap<>() {{
            put(
                "Zero",
                turretSubsystem.zeroYawCommand()
            );
            put(
                "Shoot",
                new ParallelCommandGroup(
                    new IntakeCommand(
                        intakeSubsystem,
                        IntakePreset.AGITATING
                    ),
                    new RunLoaderCommand(loaderSubsystem),
                    new RunIndexerCommand(
                        indexerSubsystem,
                        new BooleanSupplier() {
                            @Override
                            public boolean getAsBoolean() {
                                return turretSubsystem.isAtVelocity();
                            }
                        }
                    )
                )
            );
            put(
                "Hub",
                new InstantCommand(
                    () -> { 
                        turretSubsystem.setTarget(TurretTargets.HUB); 
                    }
                )
            );
            put(
                "Start Intake",
                new InstantCommand(() -> {
                    intakeSubsystem.setPreset(IntakePreset.INTAKING);
                })
            );
            put(
                "Stop Intake",
                new InstantCommand(() -> {
                    intakeSubsystem.setPreset(IntakePreset.OUT);
                })
            );
        }};
        for (Entry<String, Command> pair : namedCommands.entrySet()) {
            NamedCommands.registerCommand(
                pair.getKey(),
                pair.getValue()
            );

            SmartDashboard.putBoolean(
                "Auto/Event Bindings/" + pair.getKey() + "/available",
                true
            );
        }

        autoChooser.onChange(new Consumer<Command>() {
            @Override
            public void accept(Command command) {
                if (command instanceof PathPlannerAuto) {
                    swerveDriveSubsystem.resetOdometry(
                        ((PathPlannerAuto) command).getStartingPose()
                    );
                }
            }
        });

        // WARNING: as specified by the [pathplanner documentation]
        // (https://pathplanner.dev/pplib-named-commands.html),
        // named commands must be registerd before paths are created / loaded
        //
        // load paths
        for (String trajectoryName : Choreo.availableTrajectories()) {
            try {
                PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(
                        trajectoryName
                    );

                autoChooser.addOption(
                        trajectoryName,
                        AutoBuilder.followPath(path)
                );
            } catch (Exception e) {
                System.out.print(
                    "Caught exception during autochooser configuration: "
                    + e
                );
            }
        }
        // "do nothing" option
        autoChooser.setDefaultOption(
            "Do nothing",
            null
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
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
