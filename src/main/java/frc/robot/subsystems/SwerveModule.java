package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Robot;

@Logged
public class SwerveModule {
    private final TalonFX driveMotor;
    // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/api-usage/configuration.html
    private final TalonFXConfigurator driveConfigurator;
    private final MotorOutputConfigs driveConfig;
    private final SparkMax steerMotor;
    private final SparkMaxConfig steerConfig;

    // private final RelativeEncoder driveMotorEncoder;
    private final RelativeEncoder steerMotorEncoder;

    private double driveEncSim = 0;
    private double steerEncSim = -10.0;
    double drive_command = 0;
    double steer_command = 0;

    private final com.ctre.phoenix6.hardware.CANcoder absoluteEncoder;

    private final double motorOffsetRadians;
    private final boolean isAbsoluteEncoderReversed;

    private final PIDController steerPID;

    private static int moduleNumber = 0;
    int thisModuleNumber;

    SimpleMotorFeedforward steerFeedforward = new SimpleMotorFeedforward(0.425 / 12.0, 0.4184);

    public SwerveModule(int steerCanID, int driveCanID, int absoluteEncoderPort, double absEncoderOffsetRadians,
            boolean isAbsoluteEncoderReversed, boolean motorReversed, boolean steerMotorReversed) {
        // driveMotor = new CANSparkMax(driveCanID, MotorType.kBrushless);
        driveMotor = new TalonFX(driveCanID);
        driveConfigurator = driveMotor.getConfigurator();
        driveConfig = new MotorOutputConfigs();
        driveConfig.Inverted = motorReversed ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        driveConfig.NeutralMode = NeutralModeValue.Brake;
        driveConfigurator.apply(driveConfig);

        // driveMotor.setIdleMode(IdleMode.kBrake);
        steerMotor = new SparkMax(steerCanID, MotorType.kBrushless);
        steerConfig = new SparkMaxConfig();
        steerConfig
                .idleMode(IdleMode.kBrake)
                .inverted(steerMotorReversed);
        steerConfig.encoder
                .positionConversionFactor(SwerveModuleConstants.STEER_ROTATION_TO_RADIANS)
                .velocityConversionFactor(SwerveModuleConstants.STEER_RADIANS_PER_MINUTE);
        steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // TODO: Replace depricated method 

        // driveMotorEncoder = driveMotor.get();
        steerMotorEncoder = steerMotor.getEncoder();

        // Reset encoder offsets possibly set in Tuner X
        absoluteEncoder = new com.ctre.phoenix6.hardware.CANcoder(absoluteEncoderPort);
        com.ctre.phoenix6.configs.CANcoderConfiguration cfg = new com.ctre.phoenix6.configs.CANcoderConfiguration();
        cfg.MagnetSensor = new com.ctre.phoenix6.configs.MagnetSensorConfigs();
        cfg.MagnetSensor.MagnetOffset = 0.0f;
        absoluteEncoder.getConfigurator().apply(cfg);
        // CANcoderConfigurator configurator = absoluteEncoder.getConfigurator();

        this.motorOffsetRadians = absEncoderOffsetRadians;
        this.isAbsoluteEncoderReversed = isAbsoluteEncoderReversed;

        // driveMotorEncoder.setPositionConversionFactor(SwerveModuleConstants.DRIVE_ROTATION_TO_METER);
        // driveMotorEncoder.setVelocityConversionFactor(SwerveModuleConstants.DRIVE_METERS_PER_MINUTE);
        // TalonFXConfiguration talon_cfg = new TalonFXConfiguration();
        // driveMotor.getConfigurator().apply(cfg);

        steerPID = new PIDController(SwerveModuleConstants.MODULE_KP, 0, SwerveModuleConstants.MODULE_KD);
        steerPID.enableContinuousInput(-Math.PI, Math.PI);

        thisModuleNumber = moduleNumber;
        moduleNumber++;

        resetEncoders();
    }

    public void simulate_step() {
        driveEncSim += 0.02 * drive_command * (DriveConstants.MAX_MODULE_VELOCITY);
        steerEncSim += 0.02 * steer_command * (SwerveModuleConstants.STEER_MAX_RAD_SEC);
    }

    public double getDrivePosition() {
        if (Robot.isSimulation())
            return driveEncSim;
        // return driveMotorEncoder.getPosition();
        // TODO: Do the conversion in the motor
        return driveMotor.getPosition().getValueAsDouble() * SwerveModuleConstants.DRIVE_ROTATION_TO_METER;
    }

    public double getDriveVelocity() {
        // return driveMotorEncoder.getVelocity();
        return Robot.isSimulation() ? drive_command * (DriveConstants.MAX_MODULE_VELOCITY)
                : driveMotor.getVelocity().getValueAsDouble() * SwerveModuleConstants.DRIVE_ROTATION_TO_METER;
    }

    public double getSteerPosition() {
        if (Robot.isSimulation())
            return steerEncSim;
        return steerMotorEncoder.getPosition();
    }

    public double getSteerVelocity() {
        return steerMotorEncoder.getVelocity();
    }

    public double getAbsoluteEncoderPosition() {
        double angle = Units.rotationsToRadians(absoluteEncoder.getPosition().getValueAsDouble());
        angle -= motorOffsetRadians;
        return angle * (isAbsoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        // driveMotorEncoder.setPosition(0);
        driveMotor.setPosition(0.0);
        steerMotorEncoder.setPosition(getAbsoluteEncoderPosition());

        if (Robot.isSimulation()) {
            driveEncSim = 0.f;
            steerEncSim = -10.f;
        }
    }

    public SwerveModuleState getModuleState() {
        // FIXME: Negative?
        return new SwerveModuleState(getDriveVelocity(), getSteerRotation()); // Go back to old if broke
    }

    public SwerveModulePosition getModulePosition() {
        // FIXME: Negative?
        return new SwerveModulePosition(getDrivePosition(),
                getSteerRotation()); // Go back to old if broke
    }

    public Rotation2d getSteerRotation() {
        return new Rotation2d(MathUtil.angleModulus(getSteerPosition()));
    }

    double currentAcceleration = 0.0;

    public void setAcceleration(double m_ss_acc) {
        currentAcceleration = m_ss_acc;
    }

    public void setModuleStateRaw(SwerveModuleState state) {
        Rotation2d currentRotation = getSteerRotation();
        double orig_statevel = state.speedMetersPerSecond;
        state.optimize(currentRotation);
        state.cosineScale(currentRotation);

        double state_comp = state.speedMetersPerSecond / orig_statevel;

        drive_command = (state.speedMetersPerSecond / DriveConstants.MAX_MODULE_VELOCITY);

        driveMotor.setVoltage(
                drive_command * 12.0
                        + currentAcceleration * DriveConstants.GLOBAL_kA * state_comp);

        if (Robot.isSimulation()) {
            steer_command = steerPID.calculate(getSteerPosition(),
                    MathUtil.angleModulus(state.angle.getRadians()));
        } else {
            steer_command = steerPID.calculate(getSteerPosition(),
                    MathUtil.angleModulus(state.angle.getRadians()));
            steer_command += Math.abs(steer_command) < 0.025 ? 0.0
                    : Math.signum(steer_command) * steerFeedforward.getKs();
        }

        steerMotor.setVoltage(12.0 * steer_command);

        // SmartDashboard.putNumber("Steer" + thisModuleNumber, getSteerPosition());
        // SmartDashboard.putNumber("Drive" + thisModuleNumber, drive_command);
    }

    public void setModuleState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        setModuleStateRaw(state);
    }

    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
        drive_command = 0.f;
        steer_command = 0.f;
    }
}