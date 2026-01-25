package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
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
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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


        // Vision-based odometry updates via LimelightContainer
        try {
            RobotContainer.LLContainer.estimateMT1Odometry(odometry, getChassisSpeeds(), pigeon);
        } catch (Exception e) {
            // Don't let limelight errors crash periodic; surface the message for debugging
            SmartDashboard.putString("LL Error", e.toString());
        }

        odometry.update(getGyroRotation2d(), getModulePositions());
        fieldRobot.setPose(getOdometryPose());
        posePublisher.set(getOdometryPose());

        SmartDashboard.putData("Field", field);
        swerveStatesPublisher.set(getModuleStates());

        // SmartDashboard.putBoolean("NavX Connected", navX.isConnected());
        SmartDashboard.putBoolean("Pigeon 2 Connected", pigeon.isConnected());

        // SmartDashboard.putNumber("NavX Heading", -navX.getAngle());
        SmartDashboard.putNumber("Pigeon 2 Heading", pigeon.getYaw().getValueAsDouble());
    }

    public void setFeedforwards(DriveFeedforwards ffs) {
        double[] accs = ffs.accelerationsMPSSq();
        frontLeft.setAcceleration(accs[0]);
        frontRight.setAcceleration(accs[1]);
        backLeft.setAcceleration(accs[2]);
        backRight.setAcceleration(accs[3]);
    }

    public void zeroFeedforwards() {
        frontLeft.setAcceleration(0.0);
        frontRight.setAcceleration(0.0);
        backLeft.setAcceleration(0.0);
        backRight.setAcceleration(0.0);
    }

    public void zeroHeading() {
        setHeading(0);
    }

    public void setHeading(double deg) {
        if (Robot.isSimulation()) {
            gyroSim = Units.degreesToRadians(deg);
        }

        // navX.reset();
        // navX.setAngleAdjustment(deg);

        // double error = deg - navX.getAngle();
        // double new_adjustment = navX.getAngleAdjustment() + error;
        // navX.setAngleAdjustment(new_adjustment);

        pigeonOffset = deg - pigeon.getYaw().getValueAsDouble();
    }

    public void setGyroToEstimate() {
        resetOdometryAndGyro(odometry.getEstimatedPosition());
    }

    public Pose2d getOdometryPose() {
        return odometry.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getGyroRotation2d(), getModulePositions(), pose);
    }

    public void resetOdometryRotation(Rotation2d rot) {
        odometry.resetRotation(rot);
    }

    public void resetOdometryAndGyro(Pose2d pose) {
        setHeading(Units.radiansToDegrees(pose.getRotation().getRadians()
                + (FieldConstants.getAlliance() == Alliance.Red ? Math.PI : 0.0)));
        resetOdometry(pose);
    }

    public double getGyroHeading() {
        // return Robot.isSimulation() ? gyroSim :
        // Units.degreesToRadians(Math.IEEEremainder(-navX.getAngle(), 360));
        return Robot.isSimulation() ? gyroSim
                : Units.degreesToRadians(Math.IEEEremainder(pigeon.getYaw().getValueAsDouble() + pigeonOffset,
                        360));
    }

    public Rotation2d getGyroRotation2d() {
        return new Rotation2d(getGyroHeading());
    }

    public void stopDrive() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModules(SwerveModuleState[] states) {
        lastChassisSpeeds = DriveConstants.KINEMATICS.toChassisSpeeds(states);
        // Normalize speeds so they are all obtainable
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_MODULE_VELOCITY);
        frontLeft.setModuleState(states[0]);
        frontRight.setModuleState(states[1]);
        backLeft.setModuleState(states[2]);
        backRight.setModuleState(states[3]);
        swerveTargetStatesPublisher.set(states);
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] states = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
        setModules(states);
    }

    public void setChassisSpeedsAuto(ChassisSpeeds speeds) {
        previousSetpoint = setpointGenerator.generateSetpoint(
                previousSetpoint,
                speeds,
                0.02);
        setModules(previousSetpoint.moduleStates());
    }

    public void setXstance() {
        frontLeft.setModuleStateRaw(new SwerveModuleState(0,
                Rotation2d.fromDegrees(45)));
        frontRight.setModuleStateRaw(new SwerveModuleState(0,
                Rotation2d.fromDegrees(-45)));
        backLeft.setModuleStateRaw(new SwerveModuleState(0,
                Rotation2d.fromDegrees(-45)));
        backRight.setModuleStateRaw(new SwerveModuleState(0,
                Rotation2d.fromDegrees(45)));
    }

    public ChassisSpeeds getChassisSpeeds() {
        ChassisSpeeds speeds = DriveConstants.KINEMATICS.toChassisSpeeds(
                frontLeft.getModuleState(),
                frontRight.getModuleState(),
                backLeft.getModuleState(),
                backRight.getModuleState());

        return Robot.isSimulation() ? lastChassisSpeeds : speeds;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = {
                frontLeft.getModulePosition(),
                frontRight.getModulePosition(),
                backLeft.getModulePosition(),
                backRight.getModulePosition()
        };

        return states;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = {
                frontLeft.getModuleState(),
                frontRight.getModuleState(),
                backLeft.getModuleState(),
                backRight.getModuleState()
        };

        return states;
    }

    @Override
    public void simulationPeriodic() {
        frontLeft.simulate_step();
        frontRight.simulate_step();
        backLeft.simulate_step();
        backRight.simulate_step();
        gyroSim += 0.02 * lastChassisSpeeds.omegaRadiansPerSecond;
    }

    // ---------- Path Planner Methods ---------- \\

    public Command loadPath(String name) {
        return new PathPlannerAuto(name);
    }

    public Vector<N3> createStateStdDevs(double x, double y, double theta) {
        return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
    }

    public Vector<N3> createVisionMeasurementStdDevs(double x, double y, double theta) {
        return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
    }

    public void setAutoStartingPose(Pose2d pose) {
        FieldObject2d obj = field.getObject("autoStart");
        obj.setPose(AllianceFlipUtil.apply(pose));
    }
}