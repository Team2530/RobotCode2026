package frc.robot.subsystems;

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
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
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

import org.apache.commons.math3.analysis.MultivariateFunction;
import org.apache.commons.math3.optim.InitialGuess;
import org.apache.commons.math3.optim.MaxEval;
import org.apache.commons.math3.optim.SimpleBounds;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.nonlinear.scalar.ObjectiveFunction;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.BOBYQAOptimizer;

import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
                Units.inchesToMeters(162.15), 
                Units.inchesToMeters(182.1),
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
                Units.inchesToMeters(238.4), 
                Units.inchesToMeters(79.3),
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
    private final BooleanSupplier isLaunching;

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

    private double targetVelocity;
    private double targetYaw;

    // yaw logic
    private boolean yawIsZeroed;
    private boolean atVelocity;

    private final BOBYQAOptimizer targetingOptimizer;

    // logging
    private final StructPublisher<Pose3d> TargetPositionPublisher;

    public TurretSubsystem(
        SwerveSubsystem swerveSubsystem,
        BooleanSupplier isLaunching
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.isLaunching = isLaunching;
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
                            .withKP(TurretConstants.Launcher.PID.Holding.P)
                            .withKI(TurretConstants.Launcher.PID.Holding.I)
                            .withKD(TurretConstants.Launcher.PID.Holding.D)
                            .withKS(
                                TurretConstants.Launcher.Feedforward.Holding.kS
                            )
                            .withKV(
                                TurretConstants.Launcher.Feedforward.Holding.kV
                            )
                            .withKA(
                                TurretConstants.Launcher.Feedforward.Holding.kA
                            )
                    ).withSlot1(
                        new Slot1Configs()
                            .withKP(TurretConstants.Launcher.PID.Launching.P)
                            .withKI(TurretConstants.Launcher.PID.Launching.I)
                            .withKD(TurretConstants.Launcher.PID.Launching.D)
                            .withKS(
                                TurretConstants.Launcher.Feedforward.Launching
                                .kS
                            )
                            .withKV(
                                TurretConstants.Launcher.Feedforward.Launching
                                .kV
                            )
                            .withKA(
                                TurretConstants.Launcher.Feedforward.Launching
                                .kA
                            )
                    ).withMotionMagic(
                        new MotionMagicConfigs()
                            .withMotionMagicExpo_kV(
                                TurretConstants.Launcher.Feedforward
                                .Magic.kV
                            )
                            .withMotionMagicExpo_kA(
                                TurretConstants.Launcher.Feedforward
                                .Magic.kA
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
                    )
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
                            TurretConstants.Yaw.VELOCITY_MAX
                        ).withMotionMagicAcceleration(
                            TurretConstants.Yaw.ACCELERATION_MAX
                        )
                )
            );

        // TODO: this
        // e_PitchEncoder = new CANcoder(TurretConstants.CanIDs.PITCH_ENCODER);

        targetingOptimizer = new BOBYQAOptimizer(
            TurretConstants.TargetingOptimizer.INTERPOLATION_POINTS
        );

        setManualControl(0, 0);

        yawIsZeroed = false;

        TargetPositionPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Turret/Target_position", Pose3d.struct)
            .publish();

        targetVelocity = 0;

        atVelocity = false;
    }

    private double relativeAngularVelocityFromLinear(
        Translation2d toTarget,
        Translation2d velocity
    ) {
        return Math.acos(
            toTarget.dot(
                toTarget.plus(velocity)
            ) / (
                toTarget.getNorm() 
                    * toTarget.plus(velocity).getNorm()
            )
        );
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("shooter_target", targetVelocity);
        //SmartDashboard.putNumber("shooter_velocity", m_LauncherMotor)
        double startTime = Timer.getTimestamp();
        if (yawIsZeroed) {
            // get difference to target
            Translation3d toTarget;
            // TODO: i'm not sure about these poses and rotations
            switch (targetingMode) {
                case MANUAL:
                    // we don't need calculations                
                    toTarget = new Translation3d();
                    break;
                case ABSOLUTE:
                    toTarget = targetPosition
                        .minus(getLauncherPosition())
                        .getTranslation()
                        .rotateBy(
                            swerveSubsystem.getRotation()
                        );
                    break;
                case RELATIVE:
                default:
                    toTarget = targetPosition.getTranslation();
                    break;
            }

            MultivariateFunction optimizerFunction = new MultivariateFunction() {
                @Override
                // TODO: constant yaw
                public double value(double[] point) {
                    double yaw = point[0];
                    double pitch = point[1];
                    double speed = point[2];
                    double time = point[3];

                    // X component of the ball's position relative to the robot. 
                    double dbx = speed 
                        * Math.cos(pitch) 
                        * Math.cos(yaw) 
                        * time;
                    dbx += swerveSubsystem.getXVelocity() * time; // probably doesnt work
                    // Y component of the ball's position relative to the robot. 
                    double dby = speed 
                        * Math.cos(pitch) 
                        * Math.sin(yaw) 
                        * time;
                    dby += swerveSubsystem.getYVelocity() * time; // probably doesnt work
                    // Z (vertical) component of the ball's position relative to the robot. 
                    double dbz = (
                        -0.5 
                        * FieldConstants.GRAVITY
                        * Math.pow(time, 2)
                    ) + (
                        speed 
                        * Math.sin(pitch) 
                        * time
                    );

                    // Return error squared to avoid sqrt for optimization. 
                    // For this, minimizing squared error should be equivalent to minimizing error. 
                    return Math.pow(
                        dbx - toTarget.getX(), 
                        2
                    ) + Math.pow(
                        dby - toTarget.getY(), 
                        2
                    ) + Math.pow(
                        dbz - toTarget.getZ(),
                        2
                    ); 
                }
            };
           
            // use difference to set guess and optimize from there
            double[] guess = {
                // Start optimizer at current position to reduce optimization 
                // time.
                MathUtil.clamp(
                    getYaw(),
                    Units.degreesToRadians(
                        TurretConstants.Yaw.ANGLE_MIN

                    ),
                    Units.degreesToRadians(
                        TurretConstants.Yaw.ANGLE_MAX
                    )
                ),               
                Units.degreesToRadians(
                    TurretConstants.Pitch.ANGLE_CONSTANT
                ),
                MathUtil.clamp(
                    getLauncherVelocity(),
                    TurretConstants.Launcher.MINIMUM_VELOCITY,
                    TurretConstants.Launcher.MAXIMUM_VELOCITY
                ),
                1
            };

            // WARNING: the minimum pitch of the turret is dependent on the yaw, 
            // which presents a problem: we can't update the bounds once they're 
            // sent to the optimizer.
            //
            // my solution to this is to just get the theoretical minimum pitch 
            // at a straightline to the target; my assumption is that the 
            // optimizer will be rerun at a frequency that the discrepancies 
            // wont matter
            double [] lowerBounds = {
                Units.degreesToRadians(
                    TurretConstants.Yaw.ANGLE_MIN
                ),
                Units.degreesToRadians(
                    TurretConstants.Pitch.ANGLE_CONSTANT
                ),
                // WARNING: overridden for constant pitch
                /*TurretConstants.Pitch.ANGLE_MIN(
                    Units.radiansToDegrees(
                        Math.atan2(
                            toTarget.getY(),
                            toTarget.getX()
                        )
                    )
                ),*/
                TurretConstants.Launcher.MINIMUM_VELOCITY,
                0
            };
            double [] upperBounds = {
                Units.degreesToRadians(
                    TurretConstants.Yaw.ANGLE_MAX
                ),
                Units.degreesToRadians(
                    TurretConstants.Pitch.ANGLE_CONSTANT
                ),
                /*TurretConstants.Pitch.ANGLE_MAX,*/
                TurretConstants.Launcher.MAXIMUM_VELOCITY,
                TurretConstants.TargetingOptimizer.MAXIMUM_TIME
            };

            double optimalYaw;
            double optimalPitch;
            double optimalVelocity;
            double optimalTime;

            if (targetingMode != targetingMode.MANUAL) {
                double[] targetOptimum = targetingOptimizer.optimize(
                    new MaxEval(TurretConstants.TargetingOptimizer.MAX_EVALUATIONS),
                    new ObjectiveFunction(optimizerFunction),
                    GoalType.MINIMIZE,
                    new InitialGuess(guess),
                    new SimpleBounds(
                        lowerBounds,
                        upperBounds
                    )
                ).getPoint();

                // double[] targetOptimum = guess;            

                optimalYaw = targetOptimum[0];
                optimalPitch = targetOptimum[1];
                // double optimalVelocity = targetOptimum[2];
                optimalVelocity = targetVelocity;
                optimalTime = targetOptimum[3];
            } else {
                optimalYaw = Units.degreesToRadians(targetYaw);
                optimalPitch = TurretConstants.Pitch.ANGLE_CONSTANT;
                optimalVelocity = targetVelocity;
                optimalTime = 0;
            }

            // calculate voltages and send to motors
            /** targetVelocity Clamped in Rot/s */
            double setVelocity = MathUtil.clamp(
                targetVelocity,
                TurretConstants.Launcher.MINIMUM_VELOCITY,
                TurretConstants.Launcher.MAXIMUM_VELOCITY
            );
            m_LauncherMotor.setControl(
                new MotionMagicVelocityTorqueCurrentFOC(setVelocity)
                    .withSlot(
                        isLaunching.getAsBoolean()
                            ? 0
                            : 1 // use more sensitive profile while launching
                    )
            );

            double setYaw = MathUtil.clamp(
                optimalYaw,
                Units.degreesToRadians(
                    TurretConstants.Yaw.ANGLE_MIN
                ),
                Units.degreesToRadians(
                    TurretConstants.Yaw.ANGLE_MAX
                )
            );
            // TODO: dunno if this allows for motion magic
            // WARNING: not too confident in this math
            m_YawMotor.setControl(
                    new PositionTorqueCurrentFOC(
                        setYaw * TurretConstants.Yaw.GEAR_RATIO
                    )
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

            TargetPositionPublisher.set(
                swerveSubsystem.get3dPose()
                    .plus(
                        new Transform3d(
                            toTarget,
                            new Rotation3d()
                        )
                    )
            );
            // launcher
            SmartDashboard.putNumber(
                "Turret/Launcher/Target_velocity", 
                optimalVelocity
            );
            SmartDashboard.putNumber(
                "Turret/Launcher/output", 
                m_LauncherMotor.getMotorOutputStatus().getValueAsDouble()
            );
            SmartDashboard.putBoolean(
                "Turret/Launcher/at_velocity",
                atVelocity
            );
            // yaw
            SmartDashboard.putNumber(
                "Turret/Yaw/Target_yaw",
                optimalYaw
            );
            // pitch
            SmartDashboard.putNumber(
                "Turret/Pitch/Target_pitch",
                optimalPitch
            );

            SmartDashboard.putNumber("test_shooter_targetV", setVelocity);
        }
        // targeting
        SmartDashboard.putString(
            "Turret/Targeting/current_target_name",
            target.toString()
        );
        SmartDashboard.putNumber(
            "Turret/Targeting/time_cost",
            Timer.getTimestamp() - startTime
        );
        SmartDashboard.putString(
            "Turret/Targeting/targeting_mode",
            targetingMode.toString()
        );
        // launcher
        SmartDashboard.putNumber(
            "Turret/Launcher/Target_velocity",
            targetVelocity
        );
        SmartDashboard.putNumber(
            "Turret/Launcher/Current_velocity",
            getLauncherVelocity()
        );
        SmartDashboard.putNumber(
                "Turret/Launcher/voltage",
                m_LauncherMotor.getMotorVoltage().getValueAsDouble()
        );
        // yaw
        SmartDashboard.putNumber(
            "Turret/Yaw/Current_yaw",
            getYaw() 
        );
        SmartDashboard.putNumber(
            "Turret/Yaw/raw_yaw",
            m_YawMotor.getPosition().getValueAsDouble()
        );
        SmartDashboard.putNumber(
            "Turret/Yaw/Current_velocity",
            getYawVelocity()
        );
        SmartDashboard.putBoolean(
            "Turret/Yaw/is_zeroed",
            yawIsZeroed
        );
        SmartDashboard.putNumber(
            "Turret/Yaw/output_current", 
            getYawCurrent()
        );
        // pitch
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
     * RADIANS
     */
    public double getYaw() {
        if (yawIsZeroed) {
            return Units.rotationsToRadians(
                (m_YawMotor.getPosition().getValueAsDouble())
                / TurretConstants.Yaw.GEAR_RATIO
                + Units.degreesToRotations(TurretConstants.Yaw.ANGLE_MIN)
            );
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

    // TODO: this
    private double calculateExitToLauncherVelocity(double exitVelocity) {
        return (TurretConstants.Launcher.VelocityRegression.A * exitVelocity)
            + TurretConstants.Launcher.VelocityRegression.B; 
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

    public Command resetCommand() {
        return new  ParallelCommandGroup(
            zeroYawCommand(),
            new InstantCommand(
                () -> {
                    setTargetVelocity(0);
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
                    swerveSubsystem.getRotation()
                ),
                new Rotation3d()
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
        this.targetYaw = yaw;
    }
    
    public boolean isAtVelocity() {
        return atVelocity;
    }
}
