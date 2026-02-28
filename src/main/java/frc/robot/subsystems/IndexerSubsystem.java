package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.RobotContainer;

public class IndexerSubsystem extends SubsystemBase {

    private final SparkMax m_IndexerMotor;

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

        RobotContainer.debugTracer.addEpoch(getName());

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(
            "Indexer/velocity",
            m_IndexerMotor.getEncoder().getVelocity()
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
    public void run(double speed) {
        m_IndexerMotor.set(speed);

        SmartDashboard.putNumber(
            "Indexer/speed",
            speed
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
            -IndexerConstants.SPEED
        );
    }

    public void stop() {
        run(0.0);
    }

}

