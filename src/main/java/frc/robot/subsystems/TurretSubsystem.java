package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

import frc.robot.Constants.TurretConstants;
import frc.robot.RobotContainer;
import frc.robot.Constants.MetaConstants;
import frc.robot.util.AllianceFlipUtil;

public class TurretSubsystem extends SubsystemBase {
    
    public enum TurretTargets {
        HUB(
            new Pose3d(
                Inches.of(182.1),
                Inches.of(162.15),
                Inches.of(72),
                new Rotation3d()
            )
        ),
        SHUTTLE_LEFT(
            new Pose3d(
                Inches.of(79.3),
                Inches.of(79.3),
                Inches.of(0),
                new Rotation3d()
            )
        ),
        SHUTTLE_RIGHT(
            new Pose3d(
                Inches.of(79.3),
                Inches.of(238.4),
                Inches.of(0),
                new Rotation3d()
            )
        ),
        CUSTOM(
            new Pose3d(
                Inches.of(Double.MAX_VALUE),
                Inches.of(Double.MAX_VALUE),
                Inches.of(Double.MAX_VALUE),
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

    private final TalonFX m_LauncherPortMotor;
    private final TalonFX m_LauncherStarboardMotor;
    private final TalonFX m_YawMotor;
    private TurretTargets target = TurretTargets.HUB;
    private TargetingMode targetingMode = TargetingMode.MANUAL;
    private Pose3d targetPosition;

    // yaw logic
    private boolean yawIsZeroed;
    private boolean atVelocity;

    private Angle targetYaw;
    private AngularVelocity targetVelocity;
    private Time targetTime;

    // logging
    private final StructPublisher<Pose3d> TargetPositionPublisher;
    private final StructPublisher<Translation3d> ToTargetPublisher;
    
    // debug
    private final SendableChooser<Boolean> forceEnableFiringChooser;

    public TurretSubsystem() {
        // Initialize Motors and Encoders
        m_LauncherPortMotor = new TalonFX(
            TurretConstants.CANIDs.Launcher.PORT
        );
        m_LauncherStarboardMotor = new TalonFX(
            TurretConstants.CANIDs.Launcher.STARBOARD
        );
        m_YawMotor = new TalonFX(
            TurretConstants.CANIDs.YAW
        );

        // the output values are directly copied from the leader (`port`) and
        // replicated by the follower (`starboard`), thus we shouldn't have to
        // configure the follower motor
        m_LauncherPortMotor.getConfigurator()
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
        // WARNING: this might need to be set in periodic?
        m_LauncherStarboardMotor.setControl(
            new Follower(
                TurretConstants.CANIDs.Launcher.PORT,
                MotorAlignmentValue.Opposed
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

        setManualControl(
            Rotations.of(0),
            RotationsPerSecond.of(0)
        );

        yawIsZeroed = false;

        /* Remember that NetworkTable Publishers will NOT push with SmartDashboard unless specified.*/
        TargetPositionPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("SmartDashboard/Turret/Target_position", Pose3d.struct)
            .publish();
        ToTargetPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("SmartDashboard/Turret/to_target", Translation3d.struct)
            .publish();

        atVelocity = false;

        forceEnableFiringChooser = new SendableChooser<>();
        forceEnableFiringChooser.setDefaultOption(
            "Not Force Enabled",
            false
        );
        forceEnableFiringChooser.addOption(
            "Force Enabled",
            true
        );
        SmartDashboard.putData(
            "Force Enable Firing",
            forceEnableFiringChooser
        );
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
                                TurretConstants.Pitch.ANGLE_CONSTANT.in(Radians)
                            )
                        ).div(
                            Seconds.of(
                                Math.sqrt(
                                    (
                                        (
                                            (
                                                groundDistance.in(Meters)
                                                * Math.tan(
                                                        TurretConstants.Pitch
                                                        .ANGLE_CONSTANT.in(Radians)
                                                )
                                            ) - toTarget.getZ()
                                        ) * (
                                            2
                                            / MetaConstants.Field.GRAVITY
                                                .in(MetersPerSecondPerSecond)
                                        )
                                    )
                                )
                            )
                        )
                    );
                LinearVelocity exitVelocityX = (
                        totalVelocity
                        .times(Math.cos(totalYaw.in(Radians)))
                    ).minus(getLauncherPositionalVelocityX());
                LinearVelocity exitVelocityY = (
                        totalVelocity
                        .times(Math.sin(totalYaw.in(Radians)))
                    ).minus(getLauncherPositionalVelocityY());

                targetYaw = Radians.of(
                    (
                        (2 * Math.PI)
                        + Math.atan2(
                            exitVelocityY.in(MetersPerSecond),
                            exitVelocityX.in(MetersPerSecond)
                        )
                    ) % (2 * Math.PI)
                );
                targetVelocity = calculateExitToLauncherVelocity(
                        MetersPerSecond.of(
                            Math.sqrt(
                                Math.pow(exitVelocityX.in(MetersPerSecond), 2)
                                + Math.pow(exitVelocityY.in(MetersPerSecond), 2)
                            )
                        )
                    );
            }

            Angle setYaw;
            AngularVelocity setVelocity;

            atVelocity = (
                    getLauncherVelocity()
                    .minus(targetVelocity)
                ).abs(RotationsPerSecond)
                > TurretConstants.Launcher.MAXIMUM_VELOCITY_ERROR
                    .in(RotationsPerSecond);

            /** targetVelocity Clamped in Rot/s */
            setVelocity = RotationsPerSecond.of(
                    MathUtil.clamp(
                        targetVelocity.in(RotationsPerSecond),
                        TurretConstants.Launcher.MINIMUM_VELOCITY.in(RotationsPerSecond),
                        TurretConstants.Launcher.MAXIMUM_VELOCITY.in(RotationsPerSecond)
                    )
                );
            setYaw = Rotations.of(
                    MathUtil.clamp(
                        targetYaw.in(Rotations),
                        TurretConstants.Yaw.ANGLE_MIN.in(Rotations),
                        TurretConstants.Yaw.ANGLE_MAX.in(Rotations)
                    )
                );

            m_LauncherPortMotor.setControl(
                new VelocityTorqueCurrentFOC(setVelocity)
                    .withUpdateFreqHz(1000)
            );

            m_YawMotor.setControl(
                    new MotionMagicTorqueCurrentFOC(
                        setYaw.times(TurretConstants.Yaw.GEAR_RATIO)
                        ).withUpdateFreqHz(100)
                    );

            ToTargetPublisher.set(
                toTarget
            );

            SmartDashboard.putNumber(
                "Turret/Yaw/Set_yaw",
                setYaw.in(Rotations)
            );
        } else {
            m_LauncherPortMotor.stopMotor();
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
            m_LauncherPortMotor.getDutyCycle().getValueAsDouble()
        );
        SmartDashboard.putNumber(
            "Turret/Launcher/input_current",
            m_LauncherPortMotor.getSupplyCurrent().getValueAsDouble()
        );
        SmartDashboard.putBoolean(
            "Turret/Launcher/at_velocity",
            atVelocity
        );
        SmartDashboard.putNumber(
            "Turret/Launcher/Target_velocity",
            targetVelocity.in(RotationsPerSecond)
        );
        SmartDashboard.putNumber(
            "Turret/Launcher/Current_velocity",
            getLauncherVelocity().in(RotationsPerSecond)
        );
        ;
        SmartDashboard.putNumber(
            "Turret/Launcher/voltage",
            m_LauncherPortMotor.getMotorVoltage().getValueAsDouble()
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
            targetYaw.in(Rotations)
        );
        SmartDashboard.putNumber(
            "Turret/Yaw/Current_yaw",
            getYaw().in(Rotations)
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
        SmartDashboard.putBoolean(
            "Turret/allowFiring",
            allowFiring()
        );
    }
    
    public Angle getYaw() {
        if (yawIsZeroed) {
            return
                Rotations.of(m_YawMotor.getPosition().getValueAsDouble())
                .div(TurretConstants.Yaw.GEAR_RATIO)
                .plus(TurretConstants.Yaw.ANGLE_MIN);
        } else {
            return Rotations.of(0);
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

    public AngularVelocity getLauncherVelocity() {
        return RotationsPerSecond.of(
            m_LauncherPortMotor.getVelocity().getValueAsDouble()
        );
    }

    public LinearVelocity getLauncherPositionalVelocityX() {
        return MetersPerSecond.of(
            -RobotContainer.swerveDriveSubsystem.getAngularVelocity()
                .in(RadiansPerSecond)
            * (
                (
                    TurretConstants.Offsets.X
                    .times(
                        Math.sin(
                            RobotContainer.swerveDriveSubsystem.getRotation().getRadians()
                        )
                    )
                ).plus(
                    TurretConstants.Offsets.Y
                    .times(
                        Math.cos(
                            RobotContainer.swerveDriveSubsystem.getRotation().getRadians()
                        )
                    )
                )
            ).in(Meters)
        );
    }

    public LinearVelocity getLauncherPositionalVelocityY() {
        return MetersPerSecond.of(
            RobotContainer.swerveDriveSubsystem.getAngularVelocity()
                .in(RadiansPerSecond)
            * (
                (
                    TurretConstants.Offsets.X
                    .times(
                        Math.cos(
                            RobotContainer.swerveDriveSubsystem.getRotation().getRadians()
                        )
                    )
                ).minus(
                    TurretConstants.Offsets.Y
                    .times(
                        Math.sin(
                            RobotContainer.swerveDriveSubsystem.getRotation().getRadians()
                        )
                    )
                )
            ).in(Meters)
        );
    }

    // WARNING: oh baby talonfx i don't know if this is the right calls
    public double getLauncherCurrent() {
        return m_LauncherPortMotor.getTorqueCurrent().getValueAsDouble();
    }

    private static LinearVelocity calculateLauncherToExitVelocity(
        AngularVelocity launcherVelocity
    ) {
        return MetersPerSecond.of(
            (
                TurretConstants.Launcher.VelocityRegression.A 
                * launcherVelocity.in(RotationsPerSecond)
            )
            + TurretConstants.Launcher.VelocityRegression.B
        );
    }

    private static AngularVelocity calculateExitToLauncherVelocity(
            LinearVelocity exitVelocity
    ) {
        return RotationsPerSecond.of(
                (
                    exitVelocity.in(MetersPerSecond)
                    - TurretConstants.Launcher.VelocityRegression.B
                ) / TurretConstants.Launcher.VelocityRegression.A
            );
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
                    TurretConstants.Yaw.Zeroing.DEBOUNCE_TIME.in(Seconds),
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
                TurretConstants.Yaw.Zeroing.ROUGHPASS_VOLTAGE.in(Volts),
                TurretConstants.Yaw.Zeroing.ROUGHPASS_CURRENT_LIMIT.in(Amps)
            ),
            // back it up a little
            new InstantCommand(() -> {
                m_YawMotor.setVoltage(3);
            }),
            new WaitCommand(0.1),
            // move to zero, fine pass
            new zeroingPassCommand(
                TurretConstants.Yaw.Zeroing.FINEPASS_VOLTAGE.in(Volts),
                TurretConstants.Yaw.Zeroing.FINEPASS_CURRENT_LIMIT.in(Amps)
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
                RobotContainer.swerveDriveSubsystem.getPose()
        ).plus(
            new Transform3d(
                TurretConstants.Offsets.TRANSLATION.rotateBy(
                    new Rotation3d(
                        RobotContainer.swerveDriveSubsystem.getRotation())
                ),
                new Rotation3d()
            )
        );
    }

    public void setTargetVelocity(AngularVelocity target) {
        targetVelocity = target;
    }

    public AngularVelocity getTargetVelocity() {
        return targetVelocity;
    }

    public void setManualControl(
        Angle yaw,
        AngularVelocity velocity
    ) {
        this.targetingMode = TargetingMode.MANUAL;
        this.targetVelocity = velocity;
        this.targetYaw = yaw;
    }
    
    public boolean isAtVelocity() {
        return atVelocity && yawIsZeroed;
    }

    public void stop() {
        this.yawIsZeroed = false;
    }

    public boolean allowFiring() {
        return (
            isAtVelocity()
        ) || forceEnableFiringChooser.getSelected();
    }
}
