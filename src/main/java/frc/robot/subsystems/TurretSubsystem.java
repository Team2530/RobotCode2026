package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LoaderConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.AllianceFlipUtil;

public class TurretSubsystem extends SubsystemBase {
    
    public enum TurretTargets {
        // TODO: double check these positions
        HUB(
            new Pose3d(
                Units.inchesToMeters(182.1),
                Units.inchesToMeters(162.15), 
                Units.inchesToMeters(72),
                new Rotation3d()
            )
        ),
        SHUTTLE_LEFT(
            new Pose3d(
                Units.inchesToMeters(79.3),
                Units.inchesToMeters(79.3), 
                Units.inchesToMeters(0),
                new Rotation3d()
            )
        ),
        SHUTTLE_RIGHT(
            new Pose3d(
                Units.inchesToMeters(79.3),
                Units.inchesToMeters(238.4), 
                Units.inchesToMeters(0),
                new Rotation3d()
            )
        ),
        CUSTOM(
            new Pose3d(
                Double.MAX_VALUE,
                Double.MAX_VALUE,
                Double.MAX_VALUE,
                new Rotation3d()
            )
        );

        public Pose3d position;

        private TurretTargets(
            Pose3d position
        ) {
            this.position = AllianceFlipUtil.apply(position);
        }
    }

    public enum TargetingMode {
        // like field relative
        ABSOLUTE,
        // robot absolute
        RELATIVE,
        MANUAL
    }


    private final SwerveSubsystem swerveSubsystem; 

    private final TalonFX m_LauncherMotor;
    private final TalonFX m_YawMotor;
    // TODO: this
    // private final SparkMax m_PitchMotor;   //Neo 550

    // lets just pray its a cancoder
    // TODO: this
    // private final CANcoder e_PitchEncoder;
    // targets
    private TurretTargets target = TurretTargets.HUB;
    private TargetingMode targetingMode = TargetingMode.MANUAL;
    private Pose3d targetPosition;

    // yaw logic
    private boolean yawIsZeroed;
    private boolean atVelocity;

    private double targetYaw;
    private double targetPitch;
    private double targetVelocity;
    private double targetTime;

    // logging
    private final StructPublisher<Pose3d> TargetPositionPublisher;
    private final StructPublisher<Translation3d> ToTargetPublisher;

    public TurretSubsystem(
        SwerveSubsystem swerveSubsystem
    ) {
        this.swerveSubsystem = swerveSubsystem;
        // Initialize Motors and Encoders
        m_LauncherMotor = new TalonFX(
            TurretConstants.CanIDs.LAUNCHER_MOTOR
        );
        m_YawMotor = new TalonFX(
            TurretConstants.CanIDs.YAW_MOTOR
        );
        // TODO: this
        /*
        m_PitchMotor = new SparkMax(
            TurretConstants.CanIDs.PITCH_MOTOR,
            MotorType.kBrushless
        );
        */

        m_LauncherMotor.getConfigurator()
            .apply(
                new TalonFXConfiguration()
                    .withMotorOutput(
                        new MotorOutputConfigs().withInverted(
                            TurretConstants.Launcher.REVERSE
                                ? InvertedValue.Clockwise_Positive    
                                : InvertedValue.CounterClockwise_Positive        
                        )
                    ).withSlot0(
                        new Slot0Configs()
                            .withKP(TurretConstants.Launcher.PID.P)
                            .withKI(TurretConstants.Launcher.PID.I)
                            .withKD(TurretConstants.Launcher.PID.D)
                            .withKS(
                                TurretConstants.Launcher.Feedforward.kS
                            )
                            .withKV(
                                TurretConstants.Launcher.Feedforward.kV
                            )
                    )
            );
        m_YawMotor.getConfigurator()
            .apply(
                new TalonFXConfiguration().withMotorOutput(
                    new MotorOutputConfigs().withInverted(
                        TurretConstants.Yaw.REVERSE
                            ? InvertedValue.Clockwise_Positive    
                            : InvertedValue.CounterClockwise_Positive        
                    ).withNeutralMode(NeutralModeValue.Brake)
                ).withSlot0(
                    new Slot0Configs()
                        .withKP(TurretConstants.Yaw.PID.P)
                        .withKI(TurretConstants.Yaw.PID.I)
                        .withKD(TurretConstants.Yaw.PID.D)
                        .withKS(TurretConstants.Yaw.Feedforward.kS)
                        .withKV(TurretConstants.Yaw.Feedforward.kV)
                        .withKA(TurretConstants.Yaw.Feedforward.kA)
                ).withMotionMagic(
                    new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(
                            TurretConstants.Yaw.MAX_VELOCITY
                        ).withMotionMagicAcceleration(
                            TurretConstants.Yaw.MAX_ACCELERATION
                        )
                )
            );

        // TODO: this
        // e_PitchEncoder = new CANcoder(TurretConstants.CanIDs.PITCH_ENCODER);

        setManualControl(0, 0);

        yawIsZeroed = false;

        /* Remember that NetworkTable Publishers will NOT push with SmartDashboard unless specified.*/
        TargetPositionPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("SmartDashboard/Turret/Target_position", Pose3d.struct)
            .publish();
        ToTargetPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("SmartDashboard/Turret/to_target", Translation3d.struct)
            .publish();

        targetVelocity = 0;

        atVelocity = false;
    }

    @Override
    public void periodic() {
        double periodicTimestamp = Timer.getTimestamp();
        if (yawIsZeroed) {
            // get difference to target
            Translation3d toTarget;

            switch (targetingMode) {
                case MANUAL:
                    // we don't need calculations
                    toTarget = new Translation3d();
                    break;
                case ABSOLUTE:
                    toTarget = targetPosition
                        .minus(getLauncherPosition())
                        .getTranslation();
                    break;
                case RELATIVE:
                default:
                    toTarget = targetPosition.getTranslation();
                    break;
            }

            if (targetingMode != TargetingMode.MANUAL) {
                /*
                 * ok so basically:
                 *
                 * 1. compute the total velocity vector is would take for a fuel 
                 * to score from the position of our launcher
                 *      a. obviously, for the total velocity vector, the
                 *      direction in the xy plane is straight line to the
                 *      target.
                 *      b. and given that, to calculate speed, we can assume
                 *      that the fuel travels in a 2d plane that contains both
                 *      the starting point and the target.
                 * 2. from there, subtract the velocity of the drivebase
                 * translation / rotation.
                 *      a. to do this, convert the total velocity vector to
                 *      cartesian coordinates, simply subtract, then convert
                 *      back to polar coordinates for the turret.
                 *      b. in addition to the translational velocity of the
                 *      drivebase, the rotational velocity must also be
                 *      accounted for
                 * 3. presumably, the robot is flat on the ground
                 */
                Distance groundDistance = Meters.of(
                        toTarget.toTranslation2d().getNorm()
                );
                Angle totalYaw = Radians.of(
                    Math.atan2(
                        toTarget.getY(),
                        toTarget.getX()
                    )
                );
                // this is absolutely not how you use the units classes but i
                // didn't keep track of the original units in the equation
                LinearVelocity totalVelocity = (
                        groundDistance
                        .div(
                            Math.cos(
                                Units.degreesToRadians(
                                    TurretConstants.Pitch.ANGLE_CONSTANT
                                )
                            )
                        )
                    ).div(
                        Seconds.of(
                            Math.sqrt(
                                (
                                    groundDistance.times(
                                        Math.tan(
                                            Units.degreesToRadians(
                                                TurretConstants.Pitch.ANGLE_CONSTANT
                                            )
                                        )
                                    ).minus(
                                        Meters.of(toTarget.getZ())
                                    )
                                ).times(
                                    2
                                ).div(
                                    FieldConstants.GRAVITY
                                ).in(Meters)
                            )
                        )
                    );
                        


                LinearVelocity exitVelocityX = (
                        totalVelocity
                        .times(
                            Math.cos(totalYaw.in(Radians))
                        )
                    ).minus(getLauncherPositionalVelocityX());
                LinearVelocity exitVelocityY = (
                        totalVelocity
                        .times(Math.sin(totalYaw.in(Radians)))
                    ).minus(
                        getLauncherPositionalVelocityY()
                    );

                targetYaw = (
                    (2 * Math.PI)
                    + Math.atan2(
                        exitVelocityY.in(MetersPerSecond),
                        exitVelocityX.in(MetersPerSecond)
                    )
                ) % (2 * Math.PI);
                targetPitch = TurretConstants.Pitch.ANGLE_CONSTANT;
                targetVelocity =  calculateExitToLauncherVelocity(
                    Math.sqrt(
                        Math.pow(exitVelocityX.in(MetersPerSecond), 2)
                        + Math.pow(exitVelocityY.in(MetersPerSecond), 2)
                    )
                );
            }

            double setYaw;
            double setPitch;
            double setVelocity;

            atVelocity = Math.abs(getLauncherVelocity() - targetVelocity)
                < TurretConstants.Launcher.MAXIMUM_VELOCITY_ERROR;

            /** targetVelocity Clamped in Rot/s */
            setVelocity = MathUtil.clamp(
                    targetVelocity,
                    TurretConstants.Launcher.MINIMUM_VELOCITY,
                    TurretConstants.Launcher.MAXIMUM_VELOCITY
                    );
            setYaw = MathUtil.clamp(
                    targetYaw,
                    Units.degreesToRotations(
                        TurretConstants.Yaw.ANGLE_MIN
                        ),
                    Units.degreesToRotations(
                        TurretConstants.Yaw.ANGLE_MAX
                        )
                    );
            setPitch = targetPitch;
            boolean isFueled = getLauncherCurrent()
                > TurretConstants.Launcher.FUELED_CURRENT_LIMIT;
            m_LauncherMotor.setControl(
                new VelocityTorqueCurrentFOC(setVelocity)
                    .withUpdateFreqHz(1000)
            );

            m_YawMotor.setControl(
                    new MotionMagicTorqueCurrentFOC(
                        setYaw * TurretConstants.Yaw.GEAR_RATIO
                        ).withUpdateFreqHz(1000)
                    );

            // TODO: this
            /*
               m_PitchMotor.setVoltage(
               pitchPID.calculate(
               getPitch(),
               optimalPitch
               )
               );
               */
            SmartDashboard.putNumber(
                "Turret/Yaw/Set_yaw",
                setYaw
            );
        } else {
            m_LauncherMotor.stopMotor();
        }

        TargetPositionPublisher.set(
            targetPosition
        );

        SmartDashboard.putNumber(
            "Turret/Last_was_active",
            periodicTimestamp
        );
        // targeting
        SmartDashboard.putString(
            "Turret/Targeting/current_target_name",
            target.toString()
        );
        SmartDashboard.putString(
            "Turret/Targeting/targeting_mode",
            targetingMode.toString()
        );

        // launcher
        SmartDashboard.putNumber(
            "Turret/Launcher/output", 
            m_LauncherMotor.getDutyCycle().getValueAsDouble()
        );
        SmartDashboard.putNumber(
            "Turret/Launcher/input_current",
            m_LauncherMotor.getSupplyCurrent().getValueAsDouble()
        );
        SmartDashboard.putBoolean(
            "Turret/Launcher/at_velocity",
            atVelocity
        );
        SmartDashboard.putNumber(
            "Turret/Launcher/Target_velocity",
            targetVelocity
        );
        SmartDashboard.putNumber(
            "Turret/Launcher/Current_velocity",
            getLauncherVelocity()
        );
        ;
        SmartDashboard.putNumber(
            "Turret/Launcher/voltage",
            m_LauncherMotor.getMotorVoltage().getValueAsDouble()
        );
        SmartDashboard.putNumber(
            "Turret/Launcher/current",
            getLauncherCurrent()
        );

        // yaw
        SmartDashboard.putNumber(
            "Turret/Yaw/output",
            m_YawMotor.getMotorOutputStatus().getValueAsDouble() 
        );
        SmartDashboard.putNumber(
            "Turret/Yaw/Target_yaw",
            targetYaw
        );
        SmartDashboard.putNumber(
            "Turret/Yaw/Current_yaw",
            getYaw()
        );
        SmartDashboard.putNumber(
            "Turret/Yaw/Current_velocity",
            getYawVelocity()
        );
        SmartDashboard.putNumber(
            "Turret/Yaw/output_current",
            getYawCurrent()
        );
        SmartDashboard.putBoolean(
            "Turret/Yaw/is_zeroed",
            yawIsZeroed
        );

        // pitch
        SmartDashboard.putNumber(
            "Turret/Pitch/Target_pitch",
            targetPitch
        );
        SmartDashboard.putNumber(
            "Turret/Pitch/Current_pitch",
            getPitch()
        );
        SmartDashboard.putNumber(
            "Turret/Pitch/Current_velocity",
            getPitchVelocity()
        );
    }
    

    /*
     * rotations
     */
    public double getYaw() {
        if (yawIsZeroed) {
            return
                (m_YawMotor.getPosition().getValueAsDouble())
                / TurretConstants.Yaw.GEAR_RATIO
                + Units.degreesToRotations(TurretConstants.Yaw.ANGLE_MIN);
        } else {
            return 0;
        }
    }

    public double getYawVelocity() {
        return Units.rotationsToRadians(
            m_YawMotor.getVelocity().getValueAsDouble() 
                / TurretConstants.Yaw.GEAR_RATIO
        );
    }

    public double getYawCurrent() {
        return m_YawMotor.getTorqueCurrent()
            .getValueAsDouble();
    }

    // TODO: this
    public double getPitch() {
        return 0;
        /*
        return MathUtil.angleModulus(
            Units.rotationsToRadians(
                e_PitchEncoder.getPosition().getValueAsDouble() 
                    / TurretConstants.Pitch.GEAR_RATIO
            )
        );
        */
    }

    // TODO: this
    public double getPitchVelocity() {
        return 0;
        /*
        return Units.rotationsToRadians(
            e_PitchEncoder.getVelocity().getValueAsDouble()
                / TurretConstants.Pitch.GEAR_RATIO
        );
        */
    }

    public double getLauncherVelocity() {
        return m_LauncherMotor.getVelocity().getValueAsDouble();
    }

    // WARNING: oh baby talonfx i don't know if this is the right calls
    public double getLauncherCurrent() {
        return m_LauncherMotor.getTorqueCurrent().getValueAsDouble();
    }

    private static double calculateLauncherToExitVelocity(double launcherVelocity) {
        return Units.feetToMeters(
            (TurretConstants.Launcher.VelocityRegression.A * launcherVelocity)
            + TurretConstants.Launcher.VelocityRegression.B
        ); 
    }

    private static double calculateExitToLauncherVelocity(double exitVelocity) {
        return 
        (
            Units.metersToFeet(exitVelocity)
            - TurretConstants.Launcher.VelocityRegression.B
        ) / TurretConstants.Launcher.VelocityRegression.A;
    }
    
    public Command zeroYawCommand() {
        class zeroingPassCommand extends Command{
            private final Debouncer hardLimitDebouncer;
            private final double passVoltage;
            private final double currentLimit;
            
            public zeroingPassCommand(
                double passVoltage,
                double currentLimit
            ) {
                hardLimitDebouncer = new Debouncer(
                    TurretConstants.Yaw.Zeroing.DEBOUNCE_TIME,
                    DebounceType.kRising
            );
                this.passVoltage = passVoltage;
                this.currentLimit = currentLimit;
            }

            @Override
            public void initialize() {
                m_YawMotor.setVoltage(passVoltage);
            };

            @Override
            public boolean isFinished() {
                return hardLimitDebouncer.calculate(
                    Math.abs(
                        getYawCurrent() 
                    ) > currentLimit
                );
            };

            @Override
            public void end(boolean interrupted) {
                m_YawMotor.setVoltage(0);
            }
        };

        return new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                    yawIsZeroed = false;
                }
            ),
            // move to zero, rough pass
            new zeroingPassCommand(
                TurretConstants.Yaw.Zeroing.ROUGHPASS_VOLTAGE,
                TurretConstants.Yaw.Zeroing.ROUGHPASS_CURRENT_LIMIT
            ),
            // back it up a little
            new InstantCommand(() -> {
                m_YawMotor.setVoltage(3);
            }),
            new WaitCommand(0.2),
            // move to zero, fine pass
            new zeroingPassCommand(
                TurretConstants.Yaw.Zeroing.FINEPASS_VOLTAGE,
                TurretConstants.Yaw.Zeroing.FINEPASS_CURRENT_LIMIT
            ),
            new InstantCommand(
                () -> {
                    yawIsZeroed = true;
                    m_YawMotor.setPosition(0);
                }
            )
        );
    }

    public void setTarget(TurretTargets target) {
        this.targetingMode = TargetingMode.ABSOLUTE;
        this.target = target;
        this.targetPosition = target.position;
    }

    public void setTarget(Pose3d target) {
        this.target = TurretTargets.CUSTOM;
        this.targetPosition = target;
    }

    public void setTarget(Pose2d target) {
        this.target = TurretTargets.CUSTOM;
        this.targetPosition = new Pose3d(target);
    }

    public void setRelativeTarget(Pose3d target) {
        setTarget(target);
        this.targetingMode = TargetingMode.RELATIVE;
    }

    public void setRelativeTarget(Pose2d target) {
        setTarget(target);
        this.targetingMode = TargetingMode.RELATIVE;
    }

    public TargetingMode getTargetingMode() {
        return targetingMode;
    }

    public TurretTargets getTargetName() {
        return target;
    }
    
    public Pose3d getTargetPosition() {
        return targetPosition;
    }

    /*
     * Returns the field-relative position of the launcher
     *
     * @return A field relative Pose3d
     */
    public Pose3d getLauncherPosition() {
        return new Pose3d(
                swerveSubsystem.getPose()
        ).plus(
            new Transform3d(
                TurretConstants.Offsets.TRANSLATION.rotateBy(
                    new Rotation3d(
                        swerveSubsystem.getRotation())
                ),
                new Rotation3d()
            )
        );
    }

    public LinearVelocity getLauncherPositionalVelocityX() {
        return MetersPerSecond.of(
                 -TurretConstants.Offsets.Y
                 * swerveSubsystem.getAngularVelocity()
            ).plus(
                MetersPerSecond.of(
                    swerveSubsystem.getXVelocity()
                )
            );
    }

    public LinearVelocity getLauncherPositionalVelocityY() {
        return MetersPerSecond.of(
                 TurretConstants.Offsets.Y
                 * swerveSubsystem.getAngularVelocity()
            ).plus(
                MetersPerSecond.of(
                    swerveSubsystem.getYVelocity()
                )
            );
    }

    public void setTargetVelocity(double target) {
        targetVelocity = MathUtil.clamp(target, 0, 500);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void setManualControl(
        double yaw,
        double velocity
    ) {
        this.targetingMode = TargetingMode.MANUAL;
        this.targetVelocity = velocity;
        this.targetYaw = Units.degreesToRotations(yaw);
    }
    
    public boolean isAtVelocity() {
        return atVelocity && yawIsZeroed;
    }

    public void stop() {
        this.yawIsZeroed = false;
    }
}
