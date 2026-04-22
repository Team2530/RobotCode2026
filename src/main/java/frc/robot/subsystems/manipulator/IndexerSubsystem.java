package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import frc.robot.RobotContainer;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {

    private final SparkMax m_IndexerMotor;

    private Dimensionless targetSpeed;

    public enum IndexerFuelState {
        FUELED,
        EMPTY,
        UNKNOWN
    }
    private IndexerFuelState fuelState = IndexerFuelState.UNKNOWN;
    private Debouncer fuelDebouncer;


    public IndexerSubsystem() {
        m_IndexerMotor = new SparkMax(
            IndexerConstants.CAN_ID,
            MotorType.kBrushless
        );

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(
            IndexerConstants.REVERSE
        );
        
        m_IndexerMotor.configure(
            config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        stop();

        fuelState = IndexerFuelState.UNKNOWN;
        fuelDebouncer = new Debouncer(
                IndexerConstants.Fueling.DEBOUNCE_TIME.in(Seconds),
                DebounceType.kBoth
            );

    }

    @Override
    public void periodic() {
        Dimensionless setSpeed;

        if (
            RobotContainer.turretSubsystem.allowFiring()
            || targetSpeed.lt(Percent.of(0))
        ) {
            setSpeed = targetSpeed;

            fuelState = fuelDebouncer.calculate(
                    getCurrent().gt(IndexerConstants.Fueling.RUN_LIMIT)
                )
                ? IndexerFuelState.FUELED
                : IndexerFuelState.EMPTY;

        } else {
            setSpeed = Percent.of(0);

            fuelState = IndexerFuelState.UNKNOWN;
        }

        m_IndexerMotor.set(setSpeed.in(Value));

        SmartDashboard.putNumber(
            "Indexer/set_output",
            setSpeed.in(Value)
        );
        SmartDashboard.putNumber(
            "Indexer/velocity",
            m_IndexerMotor.getEncoder().getVelocity()
        );
        SmartDashboard.putNumber(
            "Indexer/current",
            getCurrent().in(Amps)
        );
        SmartDashboard.putNumber(
            "Indexer/output",
            m_IndexerMotor.get()
        );
        SmartDashboard.putString(
            "Indexer/fuel_state",
            fuelState.toString()
        );
    }

    public void run() {
        run(
            IndexerConstants.SPEED
        );
    }

    /*
     * set a custom speed for the indexer motor
     * @param speed - a value between -1 and 1 to set the motor to
     */
    public void run(Dimensionless speed) {
        this.targetSpeed = speed;
    }

    public void run(boolean reverse) {
        if (!reverse) {
            run();
        } else {
            runReverse();
        }
    }
    
    public void runReverse() {
        run(
            IndexerConstants.SPEED
            .times(-1)
        );
    }

    public void stop() {
        run(
            Percent.of(0)
        );
    }

    public Current getCurrent() {
        return Amps.of(
                m_IndexerMotor.getOutputCurrent()
            );
    }

    public IndexerFuelState getFuelState() {
        return fuelState;
    }
}

