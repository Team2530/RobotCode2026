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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.TurretConstants;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LoaderConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.swerve.Requester;
import frc.robot.util.swerve.RotationSocket;
import frc.robot.util.swerve.RotationSocket.RotationRequest;

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
    private final RotationSocket rotationSocket;

    private boolean isRequestingActive;
    private RotationRequest rotationRequest;

    private final TalonFX m_LauncherMotor;
    private final TalonFX m_YawMotor;
    // TODO: this
    // private final SparkMax m_PitchMotor;   //Neo 550

    // lets just pray its a cancoder
    // TODO: this
    // private final CANcoder e_PitchEncoder;

    // motor control
    private final PIDController launcherPID;
    private final PIDController yawPID;

    private final SimpleMotorFeedforward yawFeedforward;

    // targets
    private TurretTargets target;
    private TargetingMode targetingMode;
    private Pose3d targetPosition;


    // yaw logic
    private boolean yawIsZeroed;
    private double yawOffset;

    // optimizing
    private final BOBYQAOptimizer targetingOptimizer;
    private double solutionError;
    private double realError;


    // logging
    private final StructPublisher<Pose3d> TargetPositionPublisher;

    public TurretSubsystem(
        SwerveSubsystem swerveSubsystem,
        RotationSocket rotationSocket
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.rotationSocket = rotationSocket;
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
                new TalonFXConfiguration().withMotorOutput(
                    new MotorOutputConfigs().withInverted(
                        TurretConstants.Launcher.REVERSE
                            ? InvertedValue.Clockwise_Positive    
                            : InvertedValue.CounterClockwise_Positive        
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
                )
            );

        // TODO: this
        // e_PitchEncoder = new CANcoder(TurretConstants.CanIDs.PITCH_ENCODER);

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


        yawFeedforward = new SimpleMotorFeedforward(
            TurretConstants.Yaw.Feedforward.kS,
            TurretConstants.Yaw.Feedforward.kV,
            TurretConstants.Yaw.Feedforward.kA
        );

        targetingOptimizer = new BOBYQAOptimizer(
            TurretConstants.TargetingOptimizer.INTERPOLATION_POINTS
        );

        setTarget(TurretTargets.HUB);


        TargetPositionPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Turret/Targeting/Target_position", Pose3d.struct)
            .publish();
    }

    private void possessRotationSocket() {
        rotationSocket.possess(
            this,
            new Requester<RotationRequest>() {
                @Override
                public boolean isRequestingActive() {
                    return isRequestingActive;
                }

                public RotationRequest getRequest() {
                    return rotationRequest;
                };
            } 
        );
    }

    private void depossessRotationSocket() {
        rotationSocket.depossess();
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

    // TODO: this
    private double absoluteRotationFromRelative(
        Translation2d toTarget,
        double relative
    ) {
        return 0;
    }

    @Override
    public void periodic() {
        if (yawIsZeroed) {
            // get difference to target
            Translation3d toTarget;
            // TODO: i'm not sure about these poses and rotations
            switch (targetingMode) {
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

            MultivariateFunction optimizerFunction = new MultivariateFunction() 
            {
                @Override
                // WARNING: constant pitch 
                // if we just set the guess pitch and limits to the constant we
                // should be able to minimize the amount we have to change 
                // later
                public double value(double[] point) {
                    double yaw = point[0];
                    double pitch = point[1];
                    double speed = point[2];
                    double time = point[3];

                    // X component of the ball's position relative to the robot. 
                    double dbx = speed 
                        * Math.cos(pitch) 
                        * Math.cos(yaw);
                    dbx += Math.cos(
                        swerveSubsystem.getRotation().getX()
                    ) * swerveSubsystem.getXVelocity();
                    dbx += -swerveSubsystem.getAngularVelocity()
                        * TurretConstants.ShooterRotatinalPosition.SHOOTER_BACK; // Offset from center of rotation to center of launcher.
                    dbx *= time;

                    // Y component of the ball's position relative to the robot. 
                    double dby = speed 
                        * Math.cos(pitch) 
                        * Math.sin(yaw);
                    dby += Math.sin(
                            swerveSubsystem.getRotation().getX()
                    ) * swerveSubsystem.getXVelocity();
                    dby += swerveSubsystem.getAngularVelocity()
                        * TurretConstants.ShooterRotatinalPosition.SHOOTER_LEFT; // Offset from center of rotation to center of launcher.
                    dby *= time;

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
                getYaw(),               
                getPitch(),
                getLauncherVelocity(),
                0
            };

            // WARNING: the minimum pitch of the turret is dependent on the yaw, 
            // which presents a problem: we can't update the bounds once they're 
            // sent to the optimizer.
            //
            // my solution to this is to just get the theoretical minimum pitch 
            // at a straightline to the target; my assumption is that the 
            // optimizer will be rerun at a frequency that the discrepancies 
            // wont matter
            //
            // WARNING: as of writing, we have a 360 turret, so the optimizer is 
            // unbounded on yaw
            double [] lowerBounds = {
                Double.MIN_VALUE,
                TurretConstants.Pitch.ANGLE_CONSTANT,
                // WARNING: overridden for constant pitch
                /*TurretConstants.Pitch.ANGLE_MIN(
                    Units.radiansToDegrees(
                        Math.atan2(
                            toTarget.getY(),
                            toTarget.getX()
                        )
                    )
                ),*/
                0,
                0
            };
            double [] upperBounds = {
                Double.MAX_VALUE,
                TurretConstants.Pitch.ANGLE_CONSTANT,
                /*TurretConstants.Pitch.ANGLE_MAX,*/
                TurretConstants.TargetingOptimizer.MAXIMUM_VELOCITY,
                TurretConstants.TargetingOptimizer.MAXIMUM_TIME
            };

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

            // store the error values for later use
            solutionError = optimizerFunction.value(targetOptimum);
            // calculate the error of current position
            realError = optimizerFunction.value(new double[] {
                getYaw(),
                getPitch(),
                getLauncherVelocity(),
                // just steal the optimum time
                targetOptimum[3]
            });


            // modulo by 360 because of the infinite yaw limits
            double optimalYaw = targetOptimum[0] % 360;
            double optimalPitch = targetOptimum[1];
            double optimalVelocity = targetOptimum[2];
            double optimalTime = targetOptimum[3];
            // i don't think we need time?

            // calculate voltages and send to motors
            m_LauncherMotor.setVoltage(
                launcherPID.calculate(
                    getLauncherVelocity(),
                    calculateExitToLauncherVelocity(optimalVelocity)
                )
            );
        
            // WARNING: dogshit code
            //
            // compensate for drivebase movement
            // essentially, find the change in yaw because of translation,
            // then offset by the actual yaw velocity
            double yawFeedforwardOutput =
                yawFeedforward.calculateWithVelocities(
                    // translation-based yaw change
                    relativeAngularVelocityFromLinear(
                        toTarget.toTranslation2d(),
                        new Translation2d(
                            swerveSubsystem.getXVelocity(),
                            swerveSubsystem.getYVelocity()
                        )
                    ),
                    // turret + swerve rotation based yaw change
                    m_YawMotor.getVelocity().getValueAsDouble()
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
                );
            double outputYaw = optimalYaw;
            // WARNING: doggier shittier code
            //
            // if within the drivebase assist margin
            String yawControlStatus = "";
            if (
                optimalYaw - TurretConstants.Yaw.ANGLE_MIN
                    < TurretConstants.Yaw.ASSIST_MARGIN
                || TurretConstants.Yaw.ANGLE_MAX - optimalYaw
                    < TurretConstants.Yaw.ASSIST_MARGIN
            ) {
                // start requesting assistance
                if (!isRequestingActive) {
                    isRequestingActive = true;
                    possessRotationSocket();
                }

                rotationRequest = rotationSocket.new RelativeRotationRequest(
                    absoluteRotationFromRelative(
                        toTarget.toTranslation2d(),
                        (optimalYaw > 180
                            ? (TurretConstants.Yaw.ANGLE_MAX - optimalYaw)
                                - TurretConstants.Yaw.ASSIST_MARGIN 
                            : TurretConstants.Yaw.ASSIST_MARGIN 
                                - (optimalYaw -TurretConstants.Yaw.ANGLE_MIN)
                        ) 
                    )
                );

                if (rotationSocket.isActive()) {
                    yawControlStatus += "|RECIEVING ASSISTANCE";
                } else {
                    yawControlStatus += "|ASSISTANCE DENIED";
                }
                
                if (
                    optimalYaw > TurretConstants.Yaw.ANGLE_MAX
                    || optimalYaw < TurretConstants.Yaw.ANGLE_MIN
                ) {
                    outputYaw = MathUtil.clamp(
                        optimalYaw,
                        TurretConstants.Yaw.ANGLE_MIN,
                        TurretConstants.Yaw.ANGLE_MAX
                    );
                    yawFeedforwardOutput = 0;
                }

                // fullspin if the optimal and actual yaw are on different sides
                // of the dead middlepoint and if the optimal angle is beyond
                // the dead middlepoint by the deadband amount
                if (
                    (
                        (optimalYaw < 180)
                        != (getYaw() < 180)
                    ) && (
                        Math.abs(optimalYaw - 180)
                        < (180 - TurretConstants.Yaw.FULLSPIN_DEADBAND)
                    )
                ) {
                    yawControlStatus = "FULLSPIN";
                    if (optimalYaw > 180) {
                        outputYaw = TurretConstants.Yaw.ANGLE_MAX;
                    } else {
                        outputYaw = TurretConstants.Yaw.ANGLE_MIN;
                    }
                }
            } else if (isRequestingActive) {
                isRequestingActive = false;
                depossessRotationSocket();
            }

            m_YawMotor.setVoltage(
                yawPID.calculate(
                    getYaw(),
                    outputYaw
                ) + yawFeedforwardOutput
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

            // optimizer
            //
            // this publishes to "Turret/Targeting/Target_position"
            TargetPositionPublisher.set(
                swerveSubsystem.get3dPose()
                    .plus(
                        new Transform3d(
                            toTarget,
                            new Rotation3d()
                        )
                    )
            );
            SmartDashboard.putBoolean(
                "Turret/Targeting/has_valid_solution",
                hasValidSolution()
            );
            SmartDashboard.putBoolean(
                "Turret/Targeting/at_solution",
                isAtSolution()
            );
            SmartDashboard.putString(
                "Turret/Targeting/target_name",
                getTargetName().toString()
            );
            SmartDashboard.putString(
                "Turret/Targeting/targeting_type",
                getTargetingMode().toString()
            );
            SmartDashboard.putNumber(
                "Turret/Targeting/time_to_kill",
                optimalTime
            );
            // launcher
            SmartDashboard.putNumber(
                "Turret/Launcher/Target_velocity", 
                optimalVelocity
            );
            // yaw
            SmartDashboard.putNumber(
                "Turret/Yaw/Target_yaw",
                optimalYaw
            );
            SmartDashboard.putNumber(
                "Turret/Yaw/Current_yaw",
                getYaw() 
            );
            SmartDashboard.putNumber(
                "Turret/Yaw/Current_velocity",
                getYawVelocity()
            );
            SmartDashboard.putString(
                "Turret/Yaw/control_status",
                yawControlStatus + "|"
            );
            // pitch
            SmartDashboard.putNumber(
                "Turret/Pitch/Target_pitch",
                optimalPitch
            );
        }
        // launcher
        SmartDashboard.putNumber(
            "Turret/Launcher/Current_velocity",
            getLauncherVelocity()
        );
        // yaw
        SmartDashboard.putNumber(
            "Turret/Yaw/Current_yaw",
            getYaw() 
        );
        SmartDashboard.putNumber(
            "Turret/Yaw/Current_velocity",
            getYawVelocity()
        );
        SmartDashboard.putBoolean(
            "Turret/Yaw/is_zeroed",
            yawIsZeroed
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
    

    public double getYaw() {
        return MathUtil.angleModulus(
            Units.rotationsToRadians(
                (m_YawMotor.getPosition().getValueAsDouble() 
                    + Units.degreesToRotations(
                        TurretConstants.Yaw.ANGLE_MIN
                    ) - yawOffset
                ) / TurretConstants.Yaw.GEAR_RATIO
            )
        );
    }

    public double getYawVelocity() {
        return Units.rotationsToRadians(
            m_YawMotor.getVelocity().getValueAsDouble() 
                / TurretConstants.Yaw.GEAR_RATIO
        );
    }

    // TODO: this
    public double getPitch() {
        return TurretConstants.Pitch.ANGLE_CONSTANT;
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
        return 1;
    }
    
    public Command zeroYawCommand() {
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
                    m_YawMotor.getSupplyCurrent().getValueAsDouble() 
                        > TurretConstants.Yaw.Zeroing.CURRENT_LIMIT
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
            new zeroingPassCommand(5),
            // back it up a little
            new InstantCommand(() -> {
                m_YawMotor.setVoltage(-5);
            }),
            new WaitCommand(0.5),
            // move to zero, fine pass
            new zeroingPassCommand(1),
            new InstantCommand(
                () -> {
                    yawIsZeroed = true;
                    yawOffset = m_YawMotor.getPosition()
                        .getValueAsDouble();
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

    /*
     * @return boolean - true if the turret subsystem was able to generate a
     * **theoretical** valid solution to the given target
     */
    public boolean hasValidSolution() {
        return solutionError
            > TurretConstants.TargetingOptimizer.MAX_ERROR;
    }
    
    /*
     * @return boolean - true if the turret subsystem **is at** a valid solution 
     * to the given target
     */
    public boolean isAtSolution() {
        return hasValidSolution()
            && (
                realError
                < TurretConstants.TargetingOptimizer.MAX_REAL_ERROR
            );
    }
}
