package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
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
            getVelocity().in(RotationsPerSecond)
        );
    }

    /*
     * set a custom velocity for the loader motor
     * @param velocity - a value between -MAXIMUM_VELOCITY and MAXIMUM_VELOCITY,
     * in rps
     */
    public void runVelocity(AngularVelocity velocity) {
        m_LoaderMotor.setControl(
            new VelocityTorqueCurrentFOC(
                MathUtil.clamp(
                    velocity.in(RotationsPerSecond),
                    LoaderConstants.MAXIMUM_VELOCITY
                        .times(-1)
                        .in(RotationsPerSecond),
                    LoaderConstants.MAXIMUM_VELOCITY.in(RotationsPerSecond)
                )
            )
        );
        SmartDashboard.putNumber(
            "Loader/target_velocity",
            velocity.in(RotationsPerSecond)
        );
    }

    /*
     * set a custom speed for the loader motor
     * @param speed - a value between -1 and 1 to set the motor to
     */
    public void run(Dimensionless speed) {
        runVelocity(
            LoaderConstants.MAXIMUM_VELOCITY.times(speed)
        );
    }
    
    public void stop() {
        run(
            Percent.zero()
        );
    }

    public AngularVelocity getVelocity() {
        return RotationsPerSecond.of(
                m_LoaderMotor.getVelocity()
                .getValueAsDouble()
            );
    }
}
