package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ChoreoConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.LimelightContainer;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.encoders.CANCoderSwerve;
import swervelib.imu.Pigeon2Swerve;
import swervelib.math.SwerveMath;
import swervelib.motors.TalonFXSwerve;
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


public class SwerveSubsystem extends SubsystemBase {
    //Choreo PID Controllers

    private class ChoreoControllers {
        public final PIDController XCorrection;
        public final PIDController YCorrection;
        public final PIDController HeadingCorrection;

        public ChoreoControllers() {
            XCorrection = new PIDController(
                    ChoreoConstants.PID.X.P,
                    ChoreoConstants.PID.X.I,
                    ChoreoConstants.PID.X.D
                );
            YCorrection = new PIDController(
                    ChoreoConstants.PID.Y.P,
                    ChoreoConstants.PID.Y.I,
                    ChoreoConstants.PID.Y.D
                );
            HeadingCorrection = new PIDController(
                    ChoreoConstants.PID.Heading.P,
                    ChoreoConstants.PID.Heading.I,
                    ChoreoConstants.PID.Heading.D
                );
            HeadingCorrection.enableContinuousInput(
                    -Math.PI,
                    Math.PI
                );
        }
    }
    private final ChoreoControllers choreoControllers;
    
    private final SwerveDrive swerveDrive;

    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Odometry Pose", Pose2d.struct).publish();
    
    StructPublisher<SwerveSample> autoSamplePublisher = NetworkTableInstance
        .getDefault()
        .getStructTopic("Auto sample", SwerveSample.struct).publish();
    StructPublisher<Pose2d> autoPosePublisher = NetworkTableInstance
        .getDefault()
        .getStructTopic("Auto target pose", Pose2d.struct).publish();

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
        // instantiate yagsl library classes
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            // one liner >:3
            ConversionFactorsJson conversionFactors =
                new ConversionFactorsJson(){{
                    angle = new AngleConversionFactorsJson() {{
                        gearRatio = DriveConstants.Modules.Gearing.ANGLE;
                        calculate();
                    }};

                    drive = new DriveConversionFactorsJson() {{
                        gearRatio = DriveConstants.Modules.Gearing.Drive
                            .RIDICULUS;
                        diameter = DriveConstants.Modules.WHEEL_DIAMETER
                            .in(Inches);
                        calculate();
                    }};
                }};

            PIDFConfig drivePID = new PIDFConfig(
                    DriveConstants.Modules.PID.Drive.P,
                    DriveConstants.Modules.PID.Drive.I,
                    DriveConstants.Modules.PID.Drive.D,
                    DriveConstants.Modules.PID.Drive.F,
                    DriveConstants.Modules.PID.Drive.IZ
                );
            PIDFConfig anglePID = new PIDFConfig(
                    DriveConstants.Modules.PID.Angle.P,
                    DriveConstants.Modules.PID.Angle.I,
                    DriveConstants.Modules.PID.Angle.D,
                    DriveConstants.Modules.PID.Angle.F,
                    DriveConstants.Modules.PID.Angle.IZ
                );

            SwerveModulePhysicalCharacteristics physicalCharacteristics =
                new SwerveModulePhysicalCharacteristics(
                        conversionFactors,
                        DriveConstants.Modules.WHEEL_FRICTION_COEFFICIENT,
                        DriveConstants.Modules.OPTIMAL_VOLTAGE.in(Volts),
                        (int) DriveConstants.Modules.DRIVE_CURRENT_LIMIT
                            .in(Amps),
                        (int) DriveConstants.Modules.STEER_CURRENT_LIMIT
                            .in(Amps),
                        DriveConstants.Modules.DRIVE_RAMP.in(Seconds),
                        DriveConstants.Modules.STEER_RAMP.in(Seconds),
                        DriveConstants.Modules.DRIVE_FRICTION_VOLTAGE.in(Volts),
                        DriveConstants.Modules.STEER_FRICTION_VOLTAGE.in(Volts),
                        RobotConstants.MOMENT_OF_INERTIA
                            .in(KilogramSquareMeters),
                        RobotConstants.TOTAL_MASS.in(Kilograms)
                );

            // WARNING: if these types of motors ever change, so will this 
            // configuration
            SwerveModuleConfiguration modules[] = {
                new SwerveModuleConfiguration(
                    new TalonFXSwerve(
                        DriveConstants.CANIDs.Modules.FL.DRIVE,
                        true,
                        DCMotor.getKrakenX60Foc(1)
                    ),
                    new TalonFXSwerve(
                        DriveConstants.CANIDs.Modules.FL.STEER,
                        false,
                        DCMotor.getKrakenX44Foc(1)
                    ),
                    conversionFactors,
                    new CANCoderSwerve(
                        DriveConstants.CANIDs.Modules.FL.CANCODER
                    ),
                    DriveConstants.Modules.Offsets.FL.ANGLE.in(Degrees),
                    DriveConstants.Modules.Offsets.FL.X.in(Meters),
                    DriveConstants.Modules.Offsets.FL.Y.in(Meters),
                    anglePID, drivePID, physicalCharacteristics,
                    DriveConstants.Modules.Offsets.FL.ENCODER_INVERTED,
                    DriveConstants.Modules.Offsets.FL.DRIVE_INVERTED,
                    DriveConstants.Modules.Offsets.FL.ANGLE_INVERTED,
                    "PORT_BOW",
                    // Cosine compensation should not be used for simulations
                    // since it causes discrepancies not seen in real life.
                    !RobotBase.isSimulation()
                ),
                new SwerveModuleConfiguration(
                    new TalonFXSwerve(
                        DriveConstants.CANIDs.Modules.FR.DRIVE,
                        true,
                        DCMotor.getKrakenX60Foc(1)
                    ),
                    new TalonFXSwerve(
                        DriveConstants.CANIDs.Modules.FR.STEER,
                        false,
                        DCMotor.getKrakenX44Foc(1)
                    ),
                    conversionFactors,
                    new CANCoderSwerve(
                        DriveConstants.CANIDs.Modules.FR.CANCODER
                    ),
                    DriveConstants.Modules.Offsets.FR.ANGLE.in(Degrees),
                    DriveConstants.Modules.Offsets.FR.X.in(Meters),
                    DriveConstants.Modules.Offsets.FR.Y.in(Meters),
                    anglePID, drivePID, physicalCharacteristics,
                    DriveConstants.Modules.Offsets.FR.ENCODER_INVERTED,
                    DriveConstants.Modules.Offsets.FR.DRIVE_INVERTED,
                    DriveConstants.Modules.Offsets.FR.ANGLE_INVERTED,
                    "STARBOARD_BOW",
                    !RobotBase.isSimulation()
                ),
                new SwerveModuleConfiguration(
                    new TalonFXSwerve(
                        DriveConstants.CANIDs.Modules.BL.DRIVE,
                        true,
                        DCMotor.getKrakenX60Foc(1)
                    ),
                    new TalonFXSwerve(
                        DriveConstants.CANIDs.Modules.BL.STEER,
                        false,
                        DCMotor.getKrakenX44Foc(1)
                    ),
                    conversionFactors,
                    new CANCoderSwerve(
                        DriveConstants.CANIDs.Modules.BL.CANCODER
                    ),
                    DriveConstants.Modules.Offsets.BL.ANGLE.in(Degrees),
                    DriveConstants.Modules.Offsets.BL.X.in(Meters),
                    DriveConstants.Modules.Offsets.BL.Y.in(Meters),
                    anglePID, drivePID, physicalCharacteristics,
                    DriveConstants.Modules.Offsets.BL.ENCODER_INVERTED,
                    DriveConstants.Modules.Offsets.BL.DRIVE_INVERTED,
                    DriveConstants.Modules.Offsets.BL.ANGLE_INVERTED,
                    "PORT_QUARTER",
                    !RobotBase.isSimulation()
                ),
                new SwerveModuleConfiguration(
                    new TalonFXSwerve(
                        DriveConstants.CANIDs.Modules.BR.DRIVE,
                        true,
                        DCMotor.getKrakenX60Foc(1)
                    ),
                    new TalonFXSwerve(
                        DriveConstants.CANIDs.Modules.BR.STEER,
                        false,
                        DCMotor.getKrakenX44Foc(1)
                    ),
                    conversionFactors,
                    new CANCoderSwerve(
                        DriveConstants.CANIDs.Modules.BR.CANCODER
                    ),
                    DriveConstants.Modules.Offsets.BR.ANGLE.in(Degrees),
                    DriveConstants.Modules.Offsets.BR.X.in(Meters),
                    DriveConstants.Modules.Offsets.BR.Y.in(Meters),
                    anglePID, drivePID, physicalCharacteristics,
                    DriveConstants.Modules.Offsets.BR.ENCODER_INVERTED,
                    DriveConstants.Modules.Offsets.BR.DRIVE_INVERTED,
                    DriveConstants.Modules.Offsets.BR.ANGLE_INVERTED,
                    "STARBOARD_QUARTER",
                    !RobotBase.isSimulation()
                )
            };
            
            SwerveDriveConfiguration driveConfiguration =
                new SwerveDriveConfiguration(
                    modules,
                    new Pigeon2Swerve(DriveConstants.CANIDs.IMU),
                    DriveConstants.IMU.INVERTED,
                    physicalCharacteristics
            );

            PIDFConfig headingPID = new PIDFConfig(
                    DriveConstants.Control.HeadingPID.P,
                    DriveConstants.Control.HeadingPID.I,
                    DriveConstants.Control.HeadingPID.D,
                    DriveConstants.Control.HeadingPID.F,
                    DriveConstants.Control.HeadingPID.IZ
                );

            SwerveControllerConfiguration controllerConfiguration =
                new SwerveControllerConfiguration(
                    driveConfiguration,
                    headingPID,
                    DriveConstants.Control.Deadband.HEADING,
                    DriveConstants.MAX_ROBOT_VELOCITY.in(MetersPerSecond)
            );

            swerveDrive = new SwerveDrive(
                    driveConfiguration,
                    controllerConfiguration,
                    DriveConstants.MAX_ROBOT_VELOCITY.in(MetersPerSecond),
                    new Pose2d()
            );
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        // Heading correction should only be used while controlling the robot
        // via angle.
        swerveDrive.setHeadingCorrection(false);
        // Compensates for heading drift due to spinny fast
        swerveDrive.setAngularVelocityCompensation(
                DriveConstants.Control.AngularCompensation.ENABLE_IN_TELEOP,
                DriveConstants.Control.AngularCompensation.ENABLE_IN_AUTO,
                DriveConstants.Control.AngularCompensation
                    .COMPENSATION_COEFFICIENT
        );
        // Enable if you want to resynchronize your absolute encoders and motor
        // encoders periodically when they are not moving.
        //
        // TODO: idk this seems find
        swerveDrive.setModuleEncoderAutoSynchronize(true, 1);

        choreoControllers = new ChoreoControllers();

        // register gearshifter with smartdashboard
        gearChooser = new SendableChooser<>();

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

        SmartDashboard.putData("swerve/Drive Gearing", gearChooser);

        // register sysId commands with smartdashboard
        SmartDashboard.putData("swerve/SysId Drive Motors", sysIdDriveCommand());  
        SmartDashboard.putData("swerve/SysId Angle Motors", sysIdAngleCommand());
        SmartDashboard.putData("swerve/field", swerveDrive.field);
    };


     public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = getPose();
        
        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + choreoControllers.XCorrection.calculate(
                    pose.getX(),
                    sample.x
                ),
            sample.vy
                + choreoControllers.YCorrection.calculate(
                    pose.getY(),
                    sample.y
                ),
            (
                sample.omega
                / Math.pow( // trust
                    Math.PI * 2,
                    2
                )
            ) + choreoControllers.HeadingCorrection.calculate(
                    pose.getRotation().getRadians(),
                    sample.heading
                )
        );

        autoSamplePublisher.set(
            sample
        );
        autoPosePublisher.set(
            new Pose2d(
                new Translation2d(
                    sample.x,
                    sample.y
                ),
                new Rotation2d(
                    sample.heading
                )
            )
        );
        SmartDashboard.putNumber(
            "Auto/x",
            sample.x
        );
        SmartDashboard.putNumber(
            "Auto/y",
            sample.y
        );
        SmartDashboard.putNumber(
            "Auto/vx",
            sample.vx
        );
        SmartDashboard.putNumber(
            "Auto/vy",
            sample.vy
        );
        SmartDashboard.putNumber(
            "Auto/ax",
            sample.ax
        );
        SmartDashboard.putNumber(
            "Auto/ay",
            sample.ay
        );
        SmartDashboard.putNumber(
            "Auto/heading",
            Units.radiansToRotations(sample.heading)
        );
        SmartDashboard.putNumber(
            "Auto/omega",
            sample.omega
        );
        SmartDashboard.putNumberArray(
            "Auto/moduleForcesX",
            sample.moduleForcesX()
        );
        SmartDashboard.putNumberArray(
            "Auto/moduleForcesY",
            sample.moduleForcesY()
        );
        
        
        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation());
        Force[] feedforwards = new Force[4];

        for (int i = 0; i < feedforwards.length; i++) {
            feedforwards[i] = Newtons.of(
                Math.sqrt(
                    Math.pow(
                        sample.moduleForcesX()[i],
                        2
                    ) + Math.pow(
                        sample.moduleForcesY()[i],
                        2
                    )
                )
            );
        }

        swerveDrive.drive(
            robotRelativeSpeeds,
            swerveDrive.toServeModuleStates(robotRelativeSpeeds, false),
            feedforwards
        );
    }

    @Override
    public void periodic() {
     swerveDrive.updateOdometry(); 
        if(Robot.isSimulation()) {
            LimelightContainer.estimateSimOdometry();
        } else {
            RobotContainer.LLContainer.estimateMT2Odometry(this);
            //RobotContainer.LLContainer.estimateMT1Odometry(this.swerveDrive);
        }
        posePublisher.set(getPose());

        SmartDashboard.putNumber(
            "Swerve/Heading",
            getRotation().getRotations()
        );
        SmartDashboard.putNumber(
            "Swerve/angularVelocity",
            getAngularVelocity()
        );
        SmartDashboard.putNumber(
            "Swerve/velocity_x",
            getXVelocity().in(MetersPerSecond)
        );
        SmartDashboard.putNumber(
            "Swerve/velocity_y",
            getYVelocity().in(MetersPerSecond)
        );
    }
    

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

    public void resetOdometry() {
        resetOdometry(
            new Pose2d()
        );
    }
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public Pose3d get3dPose() {
        return new Pose3d(getPose());
    }
    
    public ChassisSpeeds getVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    public LinearVelocity getXVelocity() {
        return MetersPerSecond.of(
                getVelocity().vxMetersPerSecond
            );
    }

    public LinearVelocity getYVelocity() {
        return MetersPerSecond.of(
                getVelocity().vyMetersPerSecond
            );
    }

    /*
     * angular velocity in radians per second
     */
    public AngularVelocity getAngularVelocity() {
        return getVelocity().omegaRadiansPerSecond;
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
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
                DriveConstants.Modules.WHEEL_DIAMETER.in(Meters)
                ,
                gearing.gearRatio
            )
        );
    }

    public Field2d getField() {
        return swerveDrive.field;
    }

    public void setVisionStandardDeviations(
        Matrix<N3, N1> standardDeviations
    ) {
        swerveDrive.setVisionMeasurementStdDevs(standardDeviations);
    }

    public void addVisionMeasurement(
        Pose2d pose,
        double timestamp
    ) {
        swerveDrive.addVisionMeasurement(pose, timestamp);
    }
}
