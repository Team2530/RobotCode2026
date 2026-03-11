package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
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
                ).withSlot0(
                    new Slot0Configs()
                        .withKP(LoaderConstants.PID.P)
                        .withKI(LoaderConstants.PID.I)
                        .withKD(LoaderConstants.PID.D)
                )
            );

        stop();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(
            "Loader/velocity",
            getVelocity()
        );
    }

    /*
     * set a custom velocity for the loader motor
     * @param velocity - a value between -MAXIMUM_VELOCITY and MAXIMUM_VELOCITY,
     * in rps
     */
    public void runVelocity(double velocity) {
        m_LoaderMotor.setControl(
            new VelocityTorqueCurrentFOC(
                MathUtil.clamp(
                    velocity,
                    -LoaderConstants.MAXIMUM_VELOCITY,
                    LoaderConstants.MAXIMUM_VELOCITY
                )
            )
        );
        SmartDashboard.putNumber(
            "Loader/target_velocity",
            velocity
        );
    }

    /*
     * set a custom speed for the loader motor
     * @param speed - a value between -1 and 1 to set the motor to
     */
    public void run(double speed) {
        runVelocity(
            speed * LoaderConstants.MAXIMUM_VELOCITY
        );
    }
    
    public void stop() {
        run(0.0);
    }

    public double getVelocity() {
        return m_LoaderMotor.getVelocity()
            .getValueAsDouble();
    }
}
