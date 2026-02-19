package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.LoaderConstants;

public class LoaderSubsystem extends SubsystemBase {

    private final TalonFX m_LoaderMotor;

    public LoaderSubsystem() {
        m_LoaderMotor = new TalonFX(
            LoaderConstants.CAN_ID
        );
        m_LoaderMotor.getConfigurator()
            .apply(
                new TalonFXConfiguration().withMotorOutput(
                    new MotorOutputConfigs().withInverted(
                        LoaderConstants.REVERSE
                            ? InvertedValue.Clockwise_Positive    
                            : InvertedValue.CounterClockwise_Positive        
                    )
                )
            );
        run(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(
            "Loader/velocity",
            m_LoaderMotor.getVelocity().getValueAsDouble()
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
