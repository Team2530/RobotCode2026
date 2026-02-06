package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.function.Consumer;

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
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.TurretConstants;
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
        RELATIVE
    }

    private final SwerveSubsystem swerveSubsystem; 
    // Motor types may need to change, for now they are set to Spark Maxes for Neo 1/2/550.
    private final TalonFX m_XLauncherMotor; //x60
    private final TalonFX m_XYawMotor;      //x40
    private final SparkMax m_LauncherMotor;
    private final SparkMax m_YawMotor;
    private final SparkMax m_PitchMotor;   //Neo 550


    private final RelativeEncoder e_LauncherEncoder;
    private final RelativeEncoder e_YawEncoder;
    // lets just pray its a cancoder
    private final CANcoder e_PitchEncoder;

    private TurretTargets target;
    private TargetingMode targetingMode;
    private Pose3d targetPosition;
    private Translation3d toTarget; 
    private Translation3d targetRobotRelative;

    private final PIDController launcherPID;
    private final PIDController yawPID;
    private final PIDController pitchPID;

    private final SimpleMotorFeedforward yawFeedforward;

    private final BOBYQAOptimizer targetingOptimizer;
    private final MultivariateFunction optimizerFunction;

    // yaw logic
    private boolean yawIsZeroed;
    private double yawOffset;


    public TurretSubsystem(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        // Initialize Motors and Encoders
        m_XLauncherMotor = new TalonFX(
            TurretConstants.CanIDs.LAUNCHER_MOTOR 
            );
        m_XYawMotor = new TalonFX(
            TurretConstants.CanIDs.YAW_MOTOR 
            );
        m_LauncherMotor = new SparkMax(
            TurretConstants.CanIDs.LAUNCHER_MOTOR,
            MotorType.kBrushless
        );
        m_YawMotor = new SparkMax(
            TurretConstants.CanIDs.YAW_MOTOR,
            MotorType.kBrushless
        );
        m_PitchMotor = new SparkMax(
            TurretConstants.CanIDs.PITCH_MOTOR,
            MotorType.kBrushless
        );

        e_LauncherEncoder = m_LauncherMotor.getEncoder();
        e_YawEncoder = m_YawMotor.getEncoder();
        e_PitchEncoder = new CANcoder(TurretConstants.CanIDs.PITCH_ENCODER);

        launcherPID = new PIDController(
            TurretConstants.Launcher.PID.P,
            TurretConstants.Launcher.PID.I,
            TurretConstants.Launcher.PID.D
        );
        yawPID = new PIDController(
            TurretConstants.Yaw.PID.P,
            TurretConstants.Yaw.PID.I,
            TurretConstants.Yaw.PID.D
        );
        pitchPID = new PIDController(
            TurretConstants.Pitch.PID.P,
            TurretConstants.Pitch.PID.I,
            TurretConstants.Pitch.PID.D
        );

        yawFeedforward = new SimpleMotorFeedforward(
            TurretConstants.Yaw.Feedforward.kS,
            TurretConstants.Yaw.Feedforward.kV,
            TurretConstants.Yaw.Feedforward.kA
        );

        targetingOptimizer = new BOBYQAOptimizer(
            TurretConstants.TargetingOptimizer.INTERPOLATION_POINTS
        );
        optimizerFunction = new MultivariateFunction() {
            @Override
            public double value(double[] point) {
                return calculateError(
                        point[0],
                        point[1],
                        point[2],
                        point[3]
                );
            }
        };

        setTarget(TurretTargets.HUB);

        startYawZeroing();
    }
    
    private double calculateError(
        double yaw, 
        double pitch,
        double speed,
        double time
    ) {
        double g = 9.81; // m/s^2
        double dbx = speed * Math.cos(pitch) * Math.cos(yaw) * time;
        // X component of the ball's position relative to the robot. 
        double dby = speed * Math.cos(pitch) * Math.sin(yaw) * time;
        // Y component of the ball's position relative to the robot. 
        double dbz = (-.5 * g * time * time) + (speed * Math.sin(pitch) * time);
        // Z (vertical) component of the ball's position relative to the robot. 


        double squaredError = (dbx - targetRobotRelative.getX()) * (dbx - targetRobotRelative.getX())
                            + (dby - targetRobotRelative.getY()) * (dby - targetRobotRelative.getY())
                            + (dbz - targetRobotRelative.getZ()) * (dbz - targetRobotRelative.getZ());
        // targetRobotRelative is 

        return (squaredError); // Return error squared to avoid sqrt for optimization. 
                            // For this, minimizing squared error should be equivalent to minimizing error. 
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
        if (yawIsZeroed) {
            // get difference to target
            switch (targetingMode) {
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
            targetRobotRelative = toTarget;

            // use difference to set guess and optimize from there
            // TODO: set guesses with `toTarget`
            double[] guess = {
                // yaw
                // pitch
                // velocity
                // time
            };

            double[] targetOptimum = targetingOptimizer.optimize(
                new MaxEval(TurretConstants.TargetingOptimizer.MAX_EVALUATIONS),
                new ObjectiveFunction(optimizerFunction),
                GoalType.MINIMIZE,
                new InitialGuess(guess),
                new SimpleBounds(
                    TurretConstants.TargetingOptimizer.LOWER_BOUNDS, 
                    TurretConstants.TargetingOptimizer.UPPER_BOUNDS
                )
            ).getPoint();

            double optimalYaw = targetOptimum[0];
            double optimalPitch = targetOptimum[1];
            double optimalVelocity = targetOptimum[2];
            // i don't think we need time?

            // calculate voltages and send to motors
            m_LauncherMotor.setVoltage(
                launcherPID.calculate(
                    getLauncherVelocity(), 
                    calculateExitToLauncherVelocity(optimalVelocity)
                )
            );

            m_YawMotor.setVoltage(
                yawPID.calculate(
                    getYaw(),
                    optimalYaw
                ) 
                // WARNING: dogshit code
                //
                // compensate for drivebase movement
                // essentially, find the change in yaw because of translation,
                // then offset by the actual yaw velocity
                + yawFeedforward.calculateWithVelocities(
                    // translation-based yaw change
                    relativeAngularVelocityFromLinear(
                        toTarget.toTranslation2d(),
                        new Translation2d(
                            swerveSubsystem.getXVelocity(),
                            swerveSubsystem.getYVelocity()
                        )
                    ),
                    // turret + swerve rotation based yaw change
                    e_YawEncoder.getVelocity()
                        + relativeAngularVelocityFromLinear(
                            toTarget.toTranslation2d(),
                            // TODO: check the polarity of this difference
                            getLauncherPosition().relativeTo(
                                getLauncherPosition().rotateBy(
                                    new Rotation3d(
                                        0,
                                        0,
                                        swerveSubsystem.getAngularVelocity()
                                    )
                                )
                            ).toPose2d().getTranslation()
                        )
                )
            );

            m_PitchMotor.setVoltage(
                pitchPID.calculate(
                    getPitch(),
                    optimalPitch
                )
            );
        }

        
        // use difference to set guess and optimize from there
        // TODO: set guesses with `toTarget`
        double targetDistance = Math.sqrt(
            ((targetRobotRelative.getX())*(targetRobotRelative.getX()))
            +((targetRobotRelative.getY())*(targetRobotRelative.getY())) 
        );
        double[] guess = {
            // Start optimizer at current position to reduce optimization time.
            getYaw(),               
            getPitch(),
            getLauncherVelocity(),
            0
        };

        double[] targetOptimum = targetingOptimizer.optimize(
            new MaxEval(TurretConstants.TargetingOptimizer.MAX_EVALUATIONS),
            new ObjectiveFunction(optimizerFunction),
            GoalType.MINIMIZE,
            new InitialGuess(guess),
            new SimpleBounds(
                TurretConstants.TargetingOptimizer.LOWER_BOUNDS, 
                TurretConstants.TargetingOptimizer.UPPER_BOUNDS
            )
        ).getPoint();

        double optimalYaw = targetOptimum[0];
        double optimalPitch = targetOptimum[1];
        double optimalVelocity = targetOptimum[2];
        // i don't think we need time?

        // calculate voltages and send to motors
        m_LauncherMotor.setVoltage(
            launcherPID.calculate(
                getLauncherVelocity(), 
                calculateExitToLauncherVelocity(optimalVelocity)
            )
        );
        m_YawMotor.setVoltage(
            yawPID.calculate(
                getYaw(),
                optimalYaw
            )
        );
        m_PitchMotor.setVoltage(
            pitchPID.calculate(
                getPitch(),
                optimalPitch
            )
        );
    }
    

    public double getYaw() {
        return MathUtil.angleModulus(
            Units.rotationsToRadians(
                (e_YawEncoder.getPosition() 
                    + Units.degreesToRotations(
                        TurretConstants.Yaw.ANGLE_MIN
                    ) - yawOffset
                ) / TurretConstants.Yaw.GEAR_RATIO
            )
        );
    }

    public double getPitch() {
        return MathUtil.angleModulus(
            Units.rotationsToRadians(
                e_PitchEncoder.getPosition().getValueAsDouble() 
                    / TurretConstants.Pitch.GEAR_RATIO
            )
        );
    }


    public double getLauncherVelocity() {
        return m_LauncherMotor.getEncoder().getVelocity();
    }

    // TODO: this
    private double calculateExitToLauncherVelocity(double exitVelocity) {
        return 1;
    }
    
    public void startYawZeroing() {
        yawIsZeroed = false;

        class zeroingPassCommand extends Command{
            private Debouncer hardLimitDebouncer;
            private double passVoltage;
            
            public zeroingPassCommand(double voltage) {
                hardLimitDebouncer = new Debouncer(
                    TurretConstants.Yaw.Zeroing.DEBOUNCE_TIME,
                    DebounceType.kRising
            );

                passVoltage = voltage;
            }

            @Override
            public void initialize() {
                m_YawMotor.setVoltage(passVoltage);
            };

            @Override
            public boolean isFinished() {
                return hardLimitDebouncer.calculate(
                    m_YawMotor.getOutputCurrent() 
                        > TurretConstants.Yaw.Zeroing.CURRENT_LIMIT
                );
            };

            @Override
            public void end(boolean interrupted) {
                m_YawMotor.setVoltage(0);
            }
        };

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
            // move to zero, rough pass
            new zeroingPassCommand(5),
            // back it up a little
            new InstantCommand(() -> {
                m_YawMotor.setVoltage(-5);
            }),
            new WaitCommand(0.5),
            // move to zero, fine pass
            new zeroingPassCommand(1),
            new InstantCommand(() -> {
                yawIsZeroed = true;
                yawOffset = e_YawEncoder.getPosition();
            })
        ));
        
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
}
