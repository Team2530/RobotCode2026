package frc.robot.subsystems;

import java.io.File;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
import swervelib.parser.SwerveParser;
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

    public SwerveSubsystem(File swerveConfigDirectory) {
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
                    DriveConstants.SwerveModules.Offsets.FL_ANGLE,
                    DriveConstants.SwerveModules.Offsets.FL_X,
                    DriveConstants.SwerveModules.Offsets.FL_Y,
                    anglePID, 
                    drivePID, 
                    physicalCharacteristics, 
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
                    DriveConstants.SwerveModules.Offsets.FR_ANGLE,
                    DriveConstants.SwerveModules.Offsets.FR_X,
                    DriveConstants.SwerveModules.Offsets.FR_Y,
                    anglePID, 
                    drivePID, 
                    physicalCharacteristics, 
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
                    DriveConstants.SwerveModules.Offsets.BL_ANGLE,
                    DriveConstants.SwerveModules.Offsets.BL_X,
                    DriveConstants.SwerveModules.Offsets.BL_Y,
                    anglePID, 
                    drivePID, 
                    physicalCharacteristics, 
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
                    DriveConstants.SwerveModules.Offsets.BR_ANGLE,
                    DriveConstants.SwerveModules.Offsets.BR_X,
                    DriveConstants.SwerveModules.Offsets.BR_Y,
                    anglePID, 
                    drivePID, 
                    physicalCharacteristics, 
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
            false,
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
            true,
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
