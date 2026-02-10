package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.LoaderConstants;

public class LoaderSubsystem extends SubsystemBase {

    private final SparkMax m_LoaderMotor;

    public LoaderSubsystem() {
        m_LoaderMotor = new SparkMax(
                LoaderConstants.CAN_ID,
                MotorType.kBrushless
        );
    }

    public void run() {
        m_LoaderMotor.set(
            LoaderConstants.SPEED
        );
    }

    /*
     * set a custom speed for the loader motor
     * @param speed - a value between -1 and 1 to set the motor to
     */
    public void run(double speed) {
        m_LoaderMotor.set(speed);
    }

    public void run(boolean reverse) {
        if (!reverse) {
            run();
        } else {
            runReverse();
        }
    }
    
    public void runReverse() {
        m_LoaderMotor.set(
            -LoaderConstants.SPEED
        );
    }

    public void stop() {
        m_LoaderMotor.set(0.0);
    }
}
