package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
    // Motor types may need to change, for now they are set to Spark Maxes for Neo 1/2/550.

    private final SparkMax m_LauncherMotor;
    private final SparkMax m_YawMotor;
    private final SparkMax m_PitchMotor;

    private final RelativeEncoder e_LauncherEncoder;
    private final RelativeEncoder e_YawEncoder;
    // lets just pray its a  cancoder
    private final CANcoder e_PitchEncoder;

    private double targetLauncherVelocity;
    private double targetYaw;
    private double targetPitch;

    private final PIDController launcherPID;
    private final PIDController yawPID;
    private final PIDController pitchPID;


    private boolean yawIsZeroed;
    private double yawOffset;
    private Debouncer hardLimitDebouncer;

    public TurretSubsystem() {
        // Initialize Motors and Encoders
        m_LauncherMotor = new SparkMax(
            TurretConstants.CanIDs.LAUNCHER_MOTOR, 
            MotorType.kBrushless
        );
        m_YawMotor = new SparkMax(
            TurretConstants.CanIDs.YAW_MOTOR, 
            MotorType.kBrushless
        );
        m_PitchMotor = new SparkMax(
            TurretConstants.CanIDs.PITCH_MOTOR,
            MotorType.kBrushless
        );

        e_LauncherEncoder = m_LauncherMotor.getEncoder();
        e_YawEncoder = m_YawMotor.getEncoder();
        e_PitchEncoder = new CANcoder(TurretConstants.CanIDs.PITCH_ENCODER);

        launcherPID = new PIDController(
            TurretConstants.Launcher.PID.P,
            TurretConstants.Launcher.PID.I,
            TurretConstants.Launcher.PID.D
        );
        yawPID = new PIDController(
            TurretConstants.Yaw.PID.P,
            TurretConstants.Yaw.PID.I,
            TurretConstants.Yaw.PID.D
        );
        pitchPID = new PIDController(
            TurretConstants.Pitch.PID.P,
            TurretConstants.Pitch.PID.I,
            TurretConstants.Pitch.PID.D
        );

        startYawZeroing();
    }

    @Override
    public void periodic() {
        if (yawIsZeroed) {
            m_LauncherMotor.setVoltage(
                launcherPID.calculate(
                    getLauncherVelocity(), 
                    getTargetLauncherVelocity()
                )
            );
            m_YawMotor.setVoltage(
                yawPID.calculate(
                    getYaw(),
                    getTargetYaw()
                )
            );
            m_PitchMotor.setVoltage(
                pitchPID.calculate(
                    getPitch(),
                    getTargetPitch()
                )
            );
        } else {
             if (hardLimitDebouncer.calculate(
                m_YawMotor.getOutputCurrent() 
                    > TurretConstants.Yaw.Zeroing.CURRENT_LIMIT
            )) {
                m_YawMotor.setVoltage(0);

                yawIsZeroed = true;
                yawOffset = e_YawEncoder.getPosition();
            }
        }
    }


    public void setYaw(double angle) {
        // TODO: limit handling
        targetYaw = angle;
    }

    public double getYaw() {
        return MathUtil.angleModulus(
            Units.rotationsToRadians(
                e_YawEncoder.getPosition() / TurretConstants.Yaw.GEAR_RATIO
            )
        );
    }

    public double getTargetYaw() {
        return targetYaw;
    }

    public void setPitch(double angle) {
        // TODO: limit handling
        targetPitch = angle;
    }

    public double getPitch() {
        return MathUtil.angleModulus(
            Units.rotationsToRadians(
                e_PitchEncoder.getPosition().getValueAsDouble() 
                    / TurretConstants.Pitch.GEAR_RATIO
            )
        );
    }

    public double getTargetPitch() {
        return targetPitch;
    }


    public double getLauncherVelocity() {
        return m_LauncherMotor.getEncoder().getVelocity();
    }

    public double getTargetLauncherVelocity() {
        return targetLauncherVelocity;
    }

    public void setLauncherVelocity(double velocity) {
        targetLauncherVelocity = velocity; 
    }

    public void startYawZeroing() {
        yawIsZeroed = false;
        yawOffset = 0;
        hardLimitDebouncer = new Debouncer(
            TurretConstants.Yaw.Zeroing.DEBOUNCE_TIME,
            DebounceType.kRising
        );
    }
}

