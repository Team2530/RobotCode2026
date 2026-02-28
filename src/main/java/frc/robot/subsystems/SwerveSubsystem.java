package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
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

import com.ctre.phoenix6.hardware.Pigeon2;
import swervelib.imu.Pigeon2Swerve;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.util.LimelightContainer;
import frc.robot.util.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.choreoConstants;


public class SwerveSubsystem extends SubsystemBase {
    //Choreo PID Controllers
    private final PIDController xController = choreoConstants.x_CONTROLLER.getPIDController();
    private final PIDController yController = choreoConstants.y_CONTROLLER.getPIDController();
    private final PIDController headingController = choreoConstants.heading_CONTROLLER.getPIDController();
    
    private SwerveDrive swerveDrive;
    
        StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("Odometry Pose", Pose2d.struct).publish();
        
    
        private SendableChooser<SwerveGearing> gearChooser;
        
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
                    swerveDrive = new SwerveDrive(
                        DriveConstants.DRIVE_CONFIGURATION, 
                        DriveConstants.CONTROLLER_CONFIGURATION,
                        DriveConstants.MAX_ROBOT_VELOCITY, 
                        new Pose2d(new Translation2d(1.0, 2.0), Rotation2d.fromDegrees(90))
                );
            
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
            // TODO: idk this seems find
            swerveDrive.setModuleEncoderAutoSynchronize(true, 1); 
    
            headingController.enableContinuousInput(-Math.PI, Math.PI);
    
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

        SmartDashboard.putData("Swerve Drive Gearing", gearChooser);

        // register sysId commands with smartdashboard
        SmartDashboard.putData("SysId Drive Motors", sysIdDriveCommand());  
        SmartDashboard.putData("SysId Angle Motors", sysIdAngleCommand());
       
        

         RobotContainer.debugTracer.addEpoch(getName());

    };


     public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = getPose();
        
        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );
        
        
        setChassisSpeedsAUTO(speeds);
    }

    public void setChassisSpeedsAUTO(ChassisSpeeds speeds) {
        double tmp = speeds.vxMetersPerSecond;
        speeds.vxMetersPerSecond = speeds.vyMetersPerSecond;
        speeds.vyMetersPerSecond = tmp;
        tmp = speeds.omegaRadiansPerSecond;
        speeds.omegaRadiansPerSecond *= -1;
        
        this.drive(
            new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
            speeds.omegaRadiansPerSecond
        );
        
    }

    @Override
    public void periodic() {

         SmartDashboard.putData("Feild", swerveDrive.field);
         SmartDashboard.putNumberArray("RobotPose", new double[] {
            swerveDrive.getPose().getX(),
            swerveDrive.getPose().getY(),
            swerveDrive.getPose().getRotation().getDegrees()
        });
        
        if (Robot.isSimulation()) {
            LimelightContainer.estimateSimOdometry();
        } else {
            RobotContainer.LLContainer.estimateMT2Odometry(this.swerveDrive);
        }
        swerveDrive.updateOdometry();
        posePublisher.set(swerveDrive.getPose());
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

    public double getXVelocity() {
        return getVelocity().vxMetersPerSecond;
    }

    public double getYVelocity() {
        return getVelocity().vyMetersPerSecond;
    }

    /*
     * angular velocity in radians per second
     */
    public double getAngularVelocity() {
        return getVelocity().omegaRadiansPerSecond;
    }

    public Rotation3d getRotation() {
        return swerveDrive.getGyroRotation3d();
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
    
        public SwerveDrive getSwerveDrive() {
        return this.swerveDrive;
    }
}
