package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {


    public enum IntakePreset {
        // for the pivot angle, 0 is horizontal, increasing as the pivot raises.
        // i.e., 90 would be vertical
        STOWED(true, 0),
        INTAKING(false, 1),
        AGITATING(true, 1),
        CUSTOM(false, Double.MAX_VALUE);

        
        public final boolean pivotRaised;
        public final double feederSpeed;

        private IntakePreset(
                boolean pivotRaised,
                double feederSpeed
        ) {
            this.pivotRaised = pivotRaised;
            this.feederSpeed = feederSpeed;
        }
    }

    private final SparkFlex m_FeederMotor;
    private final SparkFlex m_PivotMotor;

    private IntakePreset intakePreset;
    private boolean targetPivotRaised;
    private double targetFeederSpeed;

    private Debouncer pivotDebouncer;

    public IntakeSubsystem() {
        //TODO: Change to ACTUAL motor type
        m_FeederMotor = new SparkFlex(
            IntakeConstants.Feeder.CAN_ID,
            MotorType.kBrushless
        );
        m_PivotMotor = new SparkFlex(
            IntakeConstants.Pivot.CAN_ID, 
            MotorType.kBrushless
        );

        setPreset(IntakePreset.STOWED);
    }

    @Override
    public void periodic() {
        m_FeederMotor.set(targetFeederSpeed);

        // WARNING: i'm not sure what the motor behavior will be while holding
        boolean isHolding = pivotDebouncer.calculate(
            m_PivotMotor.getOutputCurrent() 
            > IntakeConstants.Pivot.Zeroing.CURRENT_LIMIT
        );

        if (!isHolding) {
            // if moving to deploy / stow
            m_PivotMotor.setVoltage(
                IntakeConstants.Pivot.DEPLOY_VOLTAGE
                * (targetPivotRaised
                    ? -1
                    : 1
                )
            );
        } else {
            // if holding position
            m_PivotMotor.setVoltage(
                IntakeConstants.Pivot.HOLD_VOLTAGE
                * (targetPivotRaised
                    ? -1
                    : 1
                )
            );
        }

        SmartDashboard.putNumber(
            "Intake/Feeder/target_speed",
            targetFeederSpeed
        );
        // TODO: i don't know what units this is in
        SmartDashboard.putNumber(
            "Intake/Feeder/velocity",
            m_FeederMotor.getEncoder().getVelocity()
        );
        SmartDashboard.putNumber(
            "Intake/Feeder/voltage",
            m_FeederMotor.getBusVoltage()
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
            "Intake/Pivot/voltage",
            m_PivotMotor.getBusVoltage()
        );
    }
    
    /*
     * set speed of the intake feeder motor
     * @param speed - the speed of the feeder motor; this can be a value from 
     * 1.0 to -1.0
     */
    public void setFeederSpeed(double speed) {
        this.intakePreset = IntakePreset.CUSTOM;

        targetFeederSpeed = speed;
    }

    /*
     * get speed of the intake feeder motor
     * @return speed - the speed of the feeder motor; this can be a value from 
     * 1.0 to -1.0
     */
    public double getTargetFeedSpeed() {
        return targetFeederSpeed;
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

        targetFeederSpeed = preset.feederSpeed;

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
    }

    public IntakePreset getPreset() {
        return this.intakePreset;
    }
}
