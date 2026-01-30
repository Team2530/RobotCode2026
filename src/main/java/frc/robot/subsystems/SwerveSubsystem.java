package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.LimelightContainer;
import frc.robot.util.AllianceFlipUtil;

@Logged
/**
 * The SwerveSubsystem class manages odometry with swerve and Limelights.
 * It also contains methods to control the swerve drive.
 */
public class SwerveSubsystem extends SubsystemBase {

    


    boolean isalliancereset = false;

    @NotLogged
    SwerveModule frontLeft = new SwerveModule(SwerveModuleConstants.FL_STEER_ID, SwerveModuleConstants.FL_DRIVE_ID,
            SwerveModuleConstants.FL_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.FL_OFFSET_RADIANS,
            SwerveModuleConstants.FL_ABSOLUTE_ENCODER_REVERSED,
            SwerveModuleConstants.FL_MOTOR_REVERSED,
            SwerveModuleConstants.FL_STEERING_MOTOR_REVERSED);
    @NotLogged
    SwerveModule frontRight = new SwerveModule(SwerveModuleConstants.FR_STEER_ID, SwerveModuleConstants.FR_DRIVE_ID,
            SwerveModuleConstants.FR_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.FR_OFFSET_RADIANS,
            SwerveModuleConstants.FR_ABSOLUTE_ENCODER_REVERSED,
            SwerveModuleConstants.FR_MOTOR_REVERSED,
            SwerveModuleConstants.FR_STEERING_MOTOR_REVERSED);
    @NotLogged
    SwerveModule backRight = new SwerveModule(SwerveModuleConstants.BR_STEER_ID, SwerveModuleConstants.BR_DRIVE_ID,
            SwerveModuleConstants.BR_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.BR_OFFSET_RADIANS,
            SwerveModuleConstants.BR_ABSOLUTE_ENCODER_REVERSED,
            SwerveModuleConstants.BR_MOTOR_REVERSED,
            SwerveModuleConstants.BR_STEERING_MOTOR_REVERSED);
    @NotLogged
    SwerveModule backLeft = new SwerveModule(SwerveModuleConstants.BL_STEER_ID, SwerveModuleConstants.BL_DRIVE_ID,
            SwerveModuleConstants.BL_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.BL_OFFSET_RADIANS,
            SwerveModuleConstants.BL_ABSOLUTE_ENCODER_REVERSED,
            SwerveModuleConstants.BL_MOTOR_REVERSED,
            SwerveModuleConstants.BL_STEERING_MOTOR_REVERSED);

    // public final AHRS navX = new AHRS(AHRS.NavXComType.kMXP_SPI, 50);
    private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

    private double gyroSim;
    public final Pigeon2 pigeon = new Pigeon2(11);
    private double pigeonOffset = 0.0;

    private ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds();

    private Field2d field = new Field2d();
    private FieldObject2d fieldRobot = field.getRobotObject();

    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Odometry Pose", Pose2d.struct).publish();
    StructArrayPublisher<SwerveModuleState> swerveStatesPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Swerve States", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModuleState> swerveTargetStatesPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Swerve Target States", SwerveModuleState.struct).publish();

    public final LimelightContainer limelightContainer = new LimelightContainer();

    // TODO: Properly set starting pose
    public final SwerveDrivePoseEstimator odometry;

    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;

    public SwerveSubsystem() {
        odometry = new SwerveDrivePoseEstimator(DriveConstants.KINEMATICS,
                getGyroRotation2d(),
                getModulePositions(), new Pose2d(), createStateStdDevs(
                        PoseConstants.kPositionStdDevX,
                        PoseConstants.kPositionStdDevY,
                        PoseConstants.kPositionStdDevTheta),
                createVisionMeasurementStdDevs(
                        PoseConstants.kVisionStdDevX,
                        PoseConstants.kVisionStdDevY,
                        PoseConstants.kVisionStdDevTheta));

        // --------- Path Planner Init ---------- \\
        RobotConfig config = Constants.PathPlannerConstants.ROBOT_CONFIG;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        NamedCommands.registerCommand("namedCommand", new PrintCommand("Ran namedCommand"));

        setpointGenerator = new SwerveSetpointGenerator(
                config,
                Constants.SwerveModuleConstants.STEER_MAX_RAD_SEC);

        previousSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates(),
                DriveFeedforwards.zeros(config.numModules));

        // navX.enableLogging(true);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

     public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        // Apply the generated speeds
        setChassisSpeedsAuto(speeds);
    }
    public void configurePathplanner() {
        RobotConfig config = Constants.PathPlannerConstants.ROBOT_CONFIG;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        AutoBuilder.configure(
                this::getOdometryPose, // Robot pose supplier
                this::resetOdometryAndGyro, // Method to reset odometry (will be called if your auto has a starting
                                            // pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforward) -> {
                    setChassisSpeeds(speeds);
                    setFeedforwards(feedforward);
                }, // Method that will drive the robot given ROBOT
                   // RELATIVE ChassisSpeeds
                Constants.PathPlannerConstants.HOLONOMIC_FOLLOWER_CONTROLLER,
                // Constants.PathPlannerConstants.ROBOT_CONFIG, // The robot configuration
                config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;

                },
                this // Reference to this subsystem to set requirements
        );
    }

    @Override
    public void periodic() {
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.encoders.CANCoderSwerve;
import swervelib.math.SwerveMath;
import swervelib.motors.TalonFXSwerve;
import swervelib.imu.Pigeon2Swerve;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.parser.SwerveModulePhysicalCharacteristics;
import swervelib.parser.json.modules.AngleConversionFactorsJson;
import swervelib.parser.json.modules.ConversionFactorsJson;
import swervelib.parser.json.modules.DriveConversionFactorsJson;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveDrive swerveDrive;

    private final SendableChooser<SwerveGearing> gearChooser;

    enum SwerveGearing {
        LIGHT(7.03f),
        RIDICULUS(6.03f),
        LUDICRUS(5.27f);

        public final float gearRatio;

        private SwerveGearing(float gearRatio) {
            this.gearRatio = gearRatio;
        }
    }

    public SwerveSubsystem() {
        // register gearshifter with smartdashboard
        gearChooser = new SendableChooser<>();

    public Pose2d getPose() {
        Pose2d p = odometry.getEstimatedPosition();
        return p;
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getGyroRotation2d(), getModulePositions(), pose);
    }
        gearChooser.addOption(
            SwerveGearing.LIGHT.toString(),
            SwerveGearing.LIGHT
        );
        gearChooser.setDefaultOption(
            SwerveGearing.RIDICULUS.toString(),
            SwerveGearing.RIDICULUS
        );
        gearChooser.addOption(
            SwerveGearing.LUDICRUS.toString(),
            SwerveGearing.LUDICRUS
        );

        gearChooser.onChange(gearing -> changeGearing(gearing));

        SmartDashboard.putData(
            "Swerve Drive Gearing",
            gearChooser
        );

        // register sysId commands with smartdashboard
        SmartDashboard.putData(
            "SysId Drive Motors",
            sysIdDriveCommand()
        );  
        SmartDashboard.putData(
            "SysId Angle Motors",
            sysIdAngleCommand()
        );


        // instantiate yagsl library classes
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            // one liner >:3
            ConversionFactorsJson conversionFactors = 
                new ConversionFactorsJson(){{
                    angle = new AngleConversionFactorsJson() {{
                        gearRatio = DriveConstants.SwerveModules.ANGLE_GEARING;

                        calculate();
                    }};

                    drive = new DriveConversionFactorsJson() {{
                        gearRatio = DriveConstants.SwerveModules.DRIVE_GEARING;
                        diameter = DriveConstants.SwerveModules.WHEEL_DIAMETER;

                        calculate();
                    }};
                }};

            PIDFConfig drivePID = new PIDFConfig(
                DriveConstants.PIDs.Drive.P,
                DriveConstants.PIDs.Drive.I,
                DriveConstants.PIDs.Drive.D,
                DriveConstants.PIDs.Drive.F,
                DriveConstants.PIDs.Drive.IZ
            );
            PIDFConfig anglePID = new PIDFConfig(
                DriveConstants.PIDs.Angle.P,
                DriveConstants.PIDs.Angle.I,
                DriveConstants.PIDs.Angle.D,
                DriveConstants.PIDs.Angle.F,
                DriveConstants.PIDs.Angle.IZ
            );

            SwerveModulePhysicalCharacteristics physicalCharacteristics = 
                new SwerveModulePhysicalCharacteristics(
                        conversionFactors, 
                        DriveConstants.SwerveModules.WHEEL_FRICTION_COEFFICIENT, 
                        DriveConstants.SwerveModules.OPTIMAL_VOLTAGE, 
                        DriveConstants.SwerveModules.DRIVE_CURRENT_LIMIT, 
                        DriveConstants.SwerveModules.STEER_CURRENT_LIMIT, 
                        DriveConstants.SwerveModules.DRIVE_RAMP_RATE,
                        DriveConstants.SwerveModules.STEER_RAMP_RATE,
                        DriveConstants.SwerveModules.DRIVE_FRICTION_VOLTAGE,
                        DriveConstants.SwerveModules.STEER_FRICTION_VOLTAGE,
                        RobotConstants.MOMENT_OF_INERTIA, 
                        RobotConstants.TOTAL_MASS_KG
                );

            // WARNING: if these types of motors ever change, so will this 
            // configuration
            SwerveModuleConfiguration modules[] = {
                new SwerveModuleConfiguration(
                    new TalonFXSwerve(
                        DriveConstants.SwerveModules.CanIDs.FL_DRIVE,
                        true, 
                        DCMotor.getKrakenX60(1)
                    ), 
                    new TalonFXSwerve(
                        DriveConstants.SwerveModules.CanIDs.FL_STEER,
                        false, 
                        DCMotor.getKrakenX44(1)
                    ), 
                    conversionFactors,
                    new CANCoderSwerve(
                        DriveConstants.SwerveModules.CanIDs.FL_CANCODER
                    ), 
                    Units.rotationsToDegrees(
                        DriveConstants.SwerveModules.Offsets.FL_ANGLE
                    ),
                    DriveConstants.SwerveModules.Offsets.FL_X,
                    DriveConstants.SwerveModules.Offsets.FL_Y,
                    anglePID, 
                    drivePID, 
                    physicalCharacteristics, 
                    DriveConstants.SwerveModules.Offsets.FL_ENCODER_INVERTED,
                    DriveConstants.SwerveModules.Offsets.FL_DRIVE_INVERTED,
                    DriveConstants.SwerveModules.Offsets.FL_ANGLE_INVERTED, 
                    "PORT_BOW", 
                    // Cosine compensation should not be used for simulations 
                    // since it causes discrepancies not seen in real life.
                    !RobotBase.isSimulation()
                ),
                new SwerveModuleConfiguration(
                    new TalonFXSwerve(
                        DriveConstants.SwerveModules.CanIDs.FR_DRIVE,
                        true, 
                        DCMotor.getKrakenX60(1)
                    ), 
                    new TalonFXSwerve(
                        DriveConstants.SwerveModules.CanIDs.FR_STEER,
                        false, 
                        DCMotor.getKrakenX44(1)
                    ), 
                    conversionFactors,
                    new CANCoderSwerve(
                        DriveConstants.SwerveModules.CanIDs.FR_CANCODER
                    ), 
                    Units.rotationsToDegrees(
                        DriveConstants.SwerveModules.Offsets.FR_ANGLE
                    ),
                    DriveConstants.SwerveModules.Offsets.FR_X,
                    DriveConstants.SwerveModules.Offsets.FR_Y,
                    anglePID, 
                    drivePID, 
                    physicalCharacteristics, 
                    DriveConstants.SwerveModules.Offsets.FR_ENCODER_INVERTED,
                    DriveConstants.SwerveModules.Offsets.FR_DRIVE_INVERTED,
                    DriveConstants.SwerveModules.Offsets.FR_ANGLE_INVERTED,
                    "STARBOARD_BOW", 
                    !RobotBase.isSimulation()
                ),
                new SwerveModuleConfiguration(
                    new TalonFXSwerve(
                        DriveConstants.SwerveModules.CanIDs.BL_DRIVE,
                        true, 
                        DCMotor.getKrakenX60(1)
                    ), 
                    new TalonFXSwerve(
                        DriveConstants.SwerveModules.CanIDs.BL_STEER,
                        false, 
                        DCMotor.getKrakenX44(1)
                    ), 
                    conversionFactors,
                    new CANCoderSwerve(
                        DriveConstants.SwerveModules.CanIDs.BL_CANCODER
                    ), 
                    Units.rotationsToDegrees(
                        DriveConstants.SwerveModules.Offsets.BL_ANGLE
                    ),
                    DriveConstants.SwerveModules.Offsets.BL_X,
                    DriveConstants.SwerveModules.Offsets.BL_Y,
                    anglePID, 
                    drivePID, 
                    physicalCharacteristics, 
                    DriveConstants.SwerveModules.Offsets.BL_ENCODER_INVERTED,
                    DriveConstants.SwerveModules.Offsets.BL_DRIVE_INVERTED,
                    DriveConstants.SwerveModules.Offsets.BL_ANGLE_INVERTED,
                    "PORT_QUARTER", 
                    !RobotBase.isSimulation()
                ),
                new SwerveModuleConfiguration(
                    new TalonFXSwerve(
                        DriveConstants.SwerveModules.CanIDs.BR_DRIVE,
                        true, 
                        DCMotor.getKrakenX60(1)
                    ), 
                    new TalonFXSwerve(
                        DriveConstants.SwerveModules.CanIDs.BR_STEER,
                        false, 
                        DCMotor.getKrakenX44(1)
                    ), 
                    conversionFactors,
                    new CANCoderSwerve(
                        DriveConstants.SwerveModules.CanIDs.BR_CANCODER
                    ), 
                    Units.rotationsToDegrees(
                        DriveConstants.SwerveModules.Offsets.BR_ANGLE
                    ),
                    DriveConstants.SwerveModules.Offsets.BR_X,
                    DriveConstants.SwerveModules.Offsets.BR_Y,
                    anglePID, 
                    drivePID, 
                    physicalCharacteristics, 
                    DriveConstants.SwerveModules.Offsets.BR_ENCODER_INVERTED,
                    DriveConstants.SwerveModules.Offsets.BR_DRIVE_INVERTED,
                    DriveConstants.SwerveModules.Offsets.BR_ANGLE_INVERTED,
                    "STARBOARD_QUARTER", 
                    !RobotBase.isSimulation()
                )
            };
            
            SwerveDriveConfiguration driveConfiguration = 
                new SwerveDriveConfiguration(
                    modules, 
                    new Pigeon2Swerve(DriveConstants.IMU.CANID), 
                    DriveConstants.IMU.INVERTED,
                    physicalCharacteristics
            );

            PIDFConfig headingPID = new PIDFConfig(
                DriveConstants.PIDs.Heading.P,
                DriveConstants.PIDs.Heading.I,
                DriveConstants.PIDs.Heading.D,
                DriveConstants.PIDs.Heading.F,
                DriveConstants.PIDs.Heading.IZ
            );

            SwerveControllerConfiguration controllerConfiguration = 
                new SwerveControllerConfiguration(
                    driveConfiguration,
                    headingPID,
                    DriveConstants.ControlConstants.Deadband.HEADING,
                    DriveConstants.MAX_ROBOT_VELOCITY
            );

            swerveDrive = new SwerveDrive(
                    driveConfiguration, 
                    controllerConfiguration,
                    DriveConstants.MAX_ROBOT_VELOCITY, 
                    new Pose2d() // TODO: choreo's gonna need a different pose
            );
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        
        // Heading correction should only be used while controlling the robot 
        // via angle. 
        swerveDrive.setHeadingCorrection(false);
        // Compensates for heading drift due to spinny fast
        swerveDrive.setAngularVelocityCompensation(
                DriveConstants.AngularCompensation.ENABLE_IN_TELEOP,
                DriveConstants.AngularCompensation.ENABLE_IN_AUTO,
                DriveConstants.AngularCompensation.COMPENSATION_COEFFICIENT
        );
        // Enable if you want to resynchronize your absolute encoders and motor 
        // encoders periodically when they are not moving.
        //
        // idk this seems find
        swerveDrive.setModuleEncoderAutoSynchronize(true, 1); 
    };

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}

    /**
     * drive field-oriented
     * @param translation field-oriented translation; m / s
     * @param rotation angular rate; rads / s
     */
    public void drive(Translation2d translation, double rotation) {
        swerveDrive.drive(
            translation,
            rotation,
            true,
            false
        );
    }

    /**
     * drive robot-oriented
     * @param translation robot-oriented translation; m / s
     * @param rotation angular rate; rads / s
     */
    public void driveRobotRelative(
        Translation2d translation, 
        double rotation
    ) {
        swerveDrive.drive(
            translation,
            rotation,
            false,
            false
        );
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }
    
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void setMotorBrake(boolean isBraking) {
        swerveDrive.setMotorIdleMode(isBraking);
    }

    public void xStance() {
        swerveDrive.lockPose();
    }

    public Command sysIdDriveCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setDriveSysIdRoutine(
                    new SysIdRoutine.Config(), 
                    this,
                    swerveDrive, 
                    12.0, 
                    true
                ),
                10.0, // delay between each section of the command
                5.0, // how long to run each quasistatic section
                2.0 // how long to run each dynamic section
        );
    }

    public Command sysIdAngleCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setAngleSysIdRoutine(
                    new SysIdRoutine.Config(), 
                    this,
                    swerveDrive
                ),
                10.0,
                5.0, 
                2.0 
        );
    }

    public void changeGearing(SwerveGearing gearing) {
        swerveDrive.setDriveMotorConversionFactor(
            SwerveMath.calculateMetersPerRotation(
                Units.inchesToMeters(
                    DriveConstants.SwerveModules.WHEEL_DIAMETER
                ),
                gearing.gearRatio
            )
        );  
    }

    public Field2d getField() {
        return swerveDrive.field;
    }
}
