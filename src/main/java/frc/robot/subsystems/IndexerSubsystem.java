package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

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

    private double speed;

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
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(
            "Indexer/velocity",
            m_IndexerMotor.getEncoder().getVelocity()
        );

        SmartDashboard.putNumber(
            "Indexer/current",
            m_IndexerMotor.getOutputCurrent()
        );

        SmartDashboard.putNumber(
            "Indexer/output",
            m_IndexerMotor.get()
        );

        if (
            RobotContainer.turretSubsystem.isAtVelocity()
        ) {
            m_IndexerMotor.set(speed);
        } else {
            m_IndexerMotor.set(0);
        }
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
        m_IndexerMotor.set(
            speed.in(Value)
        );

        SmartDashboard.putNumber(
            "Indexer/speed",
            speed.in(Value)
        );
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

}

