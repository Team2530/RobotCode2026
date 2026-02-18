package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.LoaderConstants;

public class LoaderSubsystem extends SubsystemBase {

    private final SparkMax m_LoaderMotor;

    public LoaderSubsystem() {
        m_LoaderMotor = new SparkMax(
                LoaderConstants.CAN_ID,
                MotorType.kBrushless
        );
        run(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(
            "Loader/velocity",
            m_LoaderMotor.getEncoder().getVelocity()
        );
    }

    public void run() {
        run(
            LoaderConstants.SPEED
        );
    }

    /*
     * set a custom speed for the loader motor
     * @param speed - a value between -1 and 1 to set the motor to
     */
    public void run(double speed) {
        m_LoaderMotor.set(speed);
        SmartDashboard.putNumber(
            "Loader/speed",
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
            -LoaderConstants.SPEED
        );
    }

    public void stop() {
        run(0.0);
    }
}
