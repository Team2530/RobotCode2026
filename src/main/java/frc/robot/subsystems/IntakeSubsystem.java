package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    public enum IntakePreset {
        // for the pivot angle, 0 is horizontal, increasing as the pivot raises.
        // i.e., 90 would be vertical
        STOWED(true, 0),
        OUT(false, 0),
        INTAKING(false, 40), //intake speed, in rps
        AGITATING(true, 40),
        SPITTING(false, -IntakeConstants.Feeder.MAXIMUM_VELOCITY),
        CUSTOM(false, Double.MAX_VALUE);

        
        public final boolean pivotRaised;
        public final double feederVelocity;

        private IntakePreset(
                boolean pivotRaised,
                double feederVelocity
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
    private double targetFeederVelocity;

    private Debouncer pivotDebouncer;
    private boolean isHolding;

    public IntakeSubsystem() {
        m_FeederMotor = new TalonFX(
            IntakeConstants.Feeder.CAN_ID
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
            IntakeConstants.Pivot.CAN_ID, 
            MotorType.kBrushless
        );
        m_PivotPID = m_PivotMotor.getClosedLoopController();

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig
            .secondaryCurrentLimit(IntakeConstants.Pivot.ABSOLUTE_CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake)
            .closedLoop
                .p(IntakeConstants.Pivot.PID.P) 
                .i(IntakeConstants.Pivot.PID.I) 
                .d(IntakeConstants.Pivot.PID.D);
        m_PivotMotor.configure(
            pivotConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        m_PivotPID.setSetpoint(0, ControlType.kPosition);

        targetPivotRaised = false;
        setPreset(IntakePreset.OUT);

        isHolding = false;
    }

    @Override
    public void periodic() {
        m_FeederMotor.setControl(
            new VelocityTorqueCurrentFOC(
                targetFeederVelocity
            )
        );

        if (!isHolding) {
            // if moving to deploy / stow
            m_PivotMotor.set(
                IntakeConstants.Pivot.DEPLOY_OUTPUT
                * (targetPivotRaised
                    ? -1
                    : 1
                )
            );
            
            if (
                pivotDebouncer.calculate(
                    m_PivotMotor.getOutputCurrent() 
                    > IntakeConstants.Pivot.Zeroing.CURRENT_LIMIT
                )
            ) {
                isHolding = true;
                m_PivotMotor.getEncoder().setPosition(0);
                m_PivotMotor.getClosedLoopController().setSetpoint(0, ControlType.kPosition);
            }
        } 

        SmartDashboard.putNumber(
            "Intake/Feeder/target_velocity",
            targetFeederVelocity
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
    }
    
   /*
    * set the velocity of the feeder motor
    * @ param velocity - the velocity of the feeder motor; a value in rps
    */
    public void setFeederVelocity(double velocity) {
        this.intakePreset = IntakePreset.CUSTOM;

        targetFeederVelocity = MathUtil.clamp(
            velocity,
            -IntakeConstants.Feeder.MAXIMUM_VELOCITY,
            IntakeConstants.Feeder.MAXIMUM_VELOCITY
        );
    }

    /*
     * get velocity of the intake feeder motor
     * @return speed - the velocity of the feeder motor; this can be a value 
     * from -MAXIMUM_VELOCITY to MAXIMUM_VELOCITY
     */
    public double getFeederVelocity() {
        return m_FeederMotor.getVelocity()
            .getValueAsDouble();
    }

    /*
     * set speed of the intake feeder motor
     * @param speed - the speed of the feeder motor; this can be a value from 
     * 1.0 to -1.0
     */
    public void setFeederSpeed(double speed) {
        setFeederVelocity(
            speed * IntakeConstants.Feeder.MAXIMUM_VELOCITY
        );
    }

    /*
     * get speed of the intake feeder motor
     * @return speed - the speed of the feeder motor; this can be a value from 
     * 1.0 to -1.0
     */
    public double getTargetFeedSpeed() {
        return getFeederVelocity()
            / IntakeConstants.Feeder.MAXIMUM_VELOCITY;
    }

    public void setPivotIsRaised(boolean raised) {
        this.intakePreset = IntakePreset.CUSTOM;

        if (raised != targetPivotRaised) {
            targetPivotRaised = raised;
            resetPivotTracking();
        }
    }

    public void setPreset(IntakePreset preset) {
        this.intakePreset = preset;

        targetFeederVelocity = preset.feederVelocity;

        if (preset.pivotRaised != targetPivotRaised) {
            targetPivotRaised = preset.pivotRaised;
            resetPivotTracking();
        }
    }

    private void resetPivotTracking() {
        pivotDebouncer = new Debouncer(
            IntakeConstants.Pivot.Zeroing.DEBOUNCE_TIME,
            DebounceType.kRising
        );
        isHolding = false;
    }

    public IntakePreset getPreset() {
        return this.intakePreset;
    }
}
