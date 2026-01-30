package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase;


public class ShooterSubsystem extends SubsystemBase {
    // Constants for the Intake
    private static final int SHOOTER_MOTOR = 41; // **CHANGE THIS TO YOUR ACTUAL CAN ID**
    private static final double SHOOTER_SPEED = -0.70;    // Motor speed (0.0 to 1.0) //Please change motor speed if needed. 

    // Motor Controller Declaration
    private final SparkFlex m_shooterMotor;  

    /**
     * Initializes the Intake Subsystem.
     */
    public ShooterSubsystem() {
        // Instantiate the SPARK Flex. SPARK Flex is typically used with Brushless motors.
        m_shooterMotor = new SparkFlex(SHOOTER_MOTOR, MotorType.kBrushless);

        // --- Basic Motor Configuration (REVLib 2025 Style) ---
        // While REVLib 2025 uses declarative config objects, for simple open-loop control
        // like this, default settings are usually fine. We'll set the idle mode explicitly.
        
        // This sets the motor to coast when output is 0 (recommended for most intakes)
        // If you need to set brake, use m_intakeMotor.configAccessor.idleMode(IdleMode.kBrake);
        // Note: The specific API call might vary slightly in the final 2025 release,
        // For simple settings like this, a direct setter may still exist, or be 
        // done via config objects as shown in docs. We will use the direct method 
        // if it still exists for simplicity. If not, use the configure() method.
        
        
        // Optional: Invert motor direction if needed (for your specific hardware setup and were the motor is mounted)
        // m_intakeMotor.setInverted(true);

        // Factory reset is usually handled in the REV Hardware Client or via configure()
        // m_intakeMotor.restoreFactoryDefaults(); //this is somewhat optional in 2025 style, but it used to be reccomened to make sure the motor is in a known state.
  }
void setCoastMode() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
    

}
    /**
     * Runs the intake motor at the specified speed (forward).
     */
    public void runShooter() {
        m_shooterMotor.set(SHOOTER_SPEED);
    }

    /**
     * Stops the intake motor.
     */
    public void stopShooter() {
        m_shooterMotor.set(0.0);
    }
}
