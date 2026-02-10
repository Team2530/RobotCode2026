package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {

    private final SparkMax m_IndexerMotor;

    public IndexerSubsystem() {
        m_IndexerMotor = new SparkMax(
            IndexerConstants.CAN_ID,
            MotorType.kBrushless
        );
    }

    public void run() {
        m_IndexerMotor.set(
            IndexerConstants.SPEED
        );
    }

    /*
     * set a custom speed for the indexer motor
     * @param speed - a value between -1 and 1 to set the motor to
     */
    public void run(double speed) {
        m_IndexerMotor.set(speed);
    }

    public void run(boolean reverse) {
        if (!reverse) {
            run();
        } else {
            runReverse();
        }
    }
    
    public void runReverse() {
        m_IndexerMotor.set(
            -IndexerConstants.SPEED
        );
    }

    public void stop() {
        m_IndexerMotor.set(0.0);
    }

}

