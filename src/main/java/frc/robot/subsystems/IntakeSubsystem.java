package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    public enum IntakePreset {
        STOWED(true, RotationsPerSecond.of(0)),
        OUT(false, RotationsPerSecond.of(0)),
        INTAKING(false, RotationsPerSecond.of(44)),
        AGITATING(false, RotationsPerSecond.of(40)),
        SPITTING(false, IntakeConstants.Feeder.MAXIMUM_VELOCITY.times(-1)),
        CUSTOM(false, RotationsPerSecond.of(Double.MAX_VALUE));

        
        public final boolean pivotRaised;
        public final AngularVelocity feederVelocity;

        private IntakePreset(
                boolean pivotRaised,
                AngularVelocity feederVelocity
        ) {
            this.pivotRaised = pivotRaised;
            this.feederVelocity = feederVelocity;
        }
    }

    private final TalonFX m_FeederMotor;
    private final SparkMax m_PivotMotor;

    private SparkClosedLoopController m_PivotPID;

    private IntakePreset intakePreset;
    private boolean targetPivotRaised;
    private AngularVelocity targetFeederVelocity;

    private Debouncer pivotDebouncer;
    private boolean isHolding;

    private Time waveStart;

    public IntakeSubsystem() {
        m_FeederMotor = new TalonFX(
            IntakeConstants.CANIDs.FEEDER
        );
        m_FeederMotor.getConfigurator()
            .apply(
                new TalonFXConfiguration().withMotorOutput(
                    new MotorOutputConfigs().withInverted(
                        IntakeConstants.Feeder.REVERSE
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive
                    )
                ).withSlot0(
                    new Slot0Configs()
                        .withKP(IntakeConstants.Feeder.PID.P)
                        .withKI(IntakeConstants.Feeder.PID.I)
                        .withKD(IntakeConstants.Feeder.PID.D)
                        .withKS(IntakeConstants.Feeder.Feedforward.kS)
                        .withKV(IntakeConstants.Feeder.Feedforward.kV)
                )
            );

        m_PivotMotor = new SparkMax(
            IntakeConstants.CANIDs.PIVOT,
            MotorType.kBrushless
        );
        m_PivotPID = m_PivotMotor.getClosedLoopController();

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig
            .secondaryCurrentLimit(
                    IntakeConstants.Pivot.ABSOLUTE_CURRENT_LIMIT.in(Amps)
            )
            .idleMode(IdleMode.kBrake)
            .closedLoop
                // while holding
                .p(IntakeConstants.Pivot.Zeroing.PID.P, ClosedLoopSlot.kSlot0)
                .i(IntakeConstants.Pivot.Zeroing.PID.I, ClosedLoopSlot.kSlot0)
                .d(IntakeConstants.Pivot.Zeroing.PID.D, ClosedLoopSlot.kSlot0)
                // while agitating
                .p(IntakeConstants.Pivot.Waving.PID.P, ClosedLoopSlot.kSlot1)
                .i(IntakeConstants.Pivot.Waving.PID.I, ClosedLoopSlot.kSlot1)
                .d(IntakeConstants.Pivot.Waving.PID.D, ClosedLoopSlot.kSlot1);
                
        m_PivotMotor.configure(
            pivotConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        m_PivotPID.setSetpoint(0, ControlType.kPosition);

        resetPivotTracking();
        setPreset(IntakePreset.OUT);
    }

    @Override
    public void periodic() {
        m_FeederMotor.setControl(
            new VelocityTorqueCurrentFOC(
                targetFeederVelocity
            ).withUpdateFreqHz(1000)
        );

        if (!isHolding) {
            // if moving to deploy / stow
            m_PivotMotor.set(
                IntakeConstants.Pivot.Zeroing.DEPLOY_OUTPUT.in(Value)
                * (targetPivotRaised
                    ? -1
                    : 1
                )
            );
            
            if (
                pivotDebouncer.calculate(
                    m_PivotMotor.getOutputCurrent() 
                    > IntakeConstants.Pivot.Zeroing.CURRENT_LIMIT.in(Amps)
                )
            ) {
                isHolding = true;
                m_PivotMotor.getEncoder()
                    .setPosition(IntakeConstants.Pivot.Waving.HEIGHT);
                m_PivotMotor.getClosedLoopController().setSetpoint(
                    IntakeConstants.Pivot.Waving.HEIGHT,
                    ControlType.kPosition,
                    ClosedLoopSlot.kSlot0
                );
                waveStart = Seconds.of(Timer.getFPGATimestamp());
            }
        }


        if (
            isHolding
            && intakePreset == IntakePreset.AGITATING
        ) {
            m_PivotMotor.getClosedLoopController()
                .setSetpoint(
                    Math.cos(
                        (
                            (Timer.getFPGATimestamp() - waveStart.in(Seconds))
                            * (2 * Math.PI)
                        )
                        / (
                            IntakeConstants.Pivot.Waving.PERIOD.in(Seconds)
                        )
                    ) * IntakeConstants.Pivot.Waving.HEIGHT,
                    ControlType.kPosition,
                    ClosedLoopSlot.kSlot1
                );
        }


        SmartDashboard.putNumber(
            "Intake/Feeder/target_velocity",
            targetFeederVelocity.in(RotationsPerSecond)
        );
        // TODO: i don't know what units this is in
        SmartDashboard.putNumber(
            "Intake/Feeder/velocity",
            m_FeederMotor.getVelocity().getValueAsDouble()
        );
        SmartDashboard.putNumber(
            "Intake/Feeder/voltage",
            m_FeederMotor.getMotorVoltage().getValueAsDouble()
        );

        SmartDashboard.putBoolean(
            "Intake/Pivot/target_is_raised",
            targetPivotRaised
        );
        SmartDashboard.putBoolean(
            "Intake/Pivot/is_holding",
            isHolding 
        );
        SmartDashboard.putNumber(
            "Intake/Pivot/output",
            m_PivotMotor.getAppliedOutput()
        );
        SmartDashboard.putNumber(
            "Intake/Pivot/amps",
             m_PivotMotor.getOutputCurrent()
        );
        SmartDashboard.putNumber(
            "Intake/Pivot/target_position",
            m_PivotMotor.getClosedLoopController().getSetpoint()
        );
        SmartDashboard.putNumber(
            "Intake/Pivot/position",
            m_PivotMotor.getEncoder().getPosition()
        );
        SmartDashboard.putString(
            "Intake/preset",
            intakePreset.toString()
        );
    }
    
   /*
    * set the velocity of the feeder motor
    * @ param velocity - the velocity of the feeder motor
    */
    public void setFeederVelocity(AngularVelocity velocity) {
        this.intakePreset = IntakePreset.CUSTOM;

        targetFeederVelocity = RotationsPerSecond.of(
                MathUtil.clamp(
                    velocity.in(RotationsPerSecond),
                    IntakeConstants.Feeder.MAXIMUM_VELOCITY
                        .times(-1)
                        .in(RotationsPerSecond),
                    IntakeConstants.Feeder.MAXIMUM_VELOCITY.in(RotationsPerSecond)
                )
            );
    }

    /*
     * get velocity of the intake feeder motor
     * @return speed - the velocity of the feeder motor; this can be a value 
     * from -MAXIMUM_VELOCITY to MAXIMUM_VELOCITY
     */
    public AngularVelocity getFeederVelocity() {
        return RotationsPerSecond.of(
                m_FeederMotor.getVelocity()
                .getValueAsDouble()
            );
    }

    /*
     * set speed of the intake feeder motor
     * @param speed - the speed of the feeder motor; this can be a value from
     * 1.0 to -1.0
     */
    public void setFeederSpeed(Dimensionless speed) {
        setFeederVelocity(
            IntakeConstants.Feeder.MAXIMUM_VELOCITY.times(speed)
        );
    }

    /*
     * get speed of the intake feeder motor
     * @return speed - the speed of the feeder motor; this can be a value from
     * 1.0 to -1.0
     */
    public Dimensionless getTargetFeedSpeed() {
        return getFeederVelocity()
            .div(IntakeConstants.Feeder.MAXIMUM_VELOCITY);
            
    }

    public void setPivotIsRaised(boolean raised) {
        this.intakePreset = IntakePreset.CUSTOM;

        targetPivotRaised = raised;
        resetPivotTracking();
    }

    public void setPreset(IntakePreset preset) {
        this.intakePreset = preset;

        targetFeederVelocity = preset.feederVelocity;

        targetPivotRaised = preset.pivotRaised;
        resetPivotTracking();
    }

    private void resetPivotTracking() {
        pivotDebouncer = new Debouncer(
            IntakeConstants.Pivot.Zeroing.DEBOUNCE_TIME.in(Seconds),
            DebounceType.kRising
        );
        isHolding = false;
    }

    public IntakePreset getPreset() {
        return this.intakePreset;
    }
}
