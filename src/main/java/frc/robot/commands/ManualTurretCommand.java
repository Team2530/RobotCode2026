package frc.robot.commands;

import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ManualTurretCommand extends Command {
    private final TurretSubsystem m_LauncherSubsystem;
    private final SwerveSubsystem m_SwerveSubsystem;
    private final XboxController operatorXbox;

    private double targetVelocity;
    private double targetYaw;

    private boolean analogClick;

    public ManualTurretCommand(
        TurretSubsystem LauncherSubsystem,
        SwerveSubsystem SwerveSubsystem,
        XboxController operatorXbox
    ) {
        m_LauncherSubsystem = LauncherSubsystem;
        m_SwerveSubsystem = SwerveSubsystem;
        this.operatorXbox = operatorXbox;
    
        this.targetVelocity = 40;
        this.targetYaw = 0;

        addRequirements(LauncherSubsystem);

        analogClick = false;
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (
            (
                Math.pow(
                    operatorXbox.getLeftY(),
                    2
                ) + Math.pow(
                    operatorXbox.getLeftX(),
                    2
                )


            ) > 0.2
        ) {
            targetYaw = Units.radiansToDegrees(
                (
                    (3 * Math.PI) 
                    + Math.atan2(
                        operatorXbox.getLeftX(),
                        operatorXbox.getLeftY()
                    ) 
                    - m_SwerveSubsystem.getRotation().getZ()
                ) % (2 * Math.PI)
            );
        }

        if (operatorXbox.getRightY() > 0.2 || operatorXbox.getRightY() < -0.2) {
            if (!analogClick) {
                analogClick = true;
                changeVelocity(-Math.signum(operatorXbox.getRightY()));
            }            
        } else {
            analogClick = false;
        }

        m_LauncherSubsystem.setManualControl(
            targetYaw,
            targetVelocity 
        );

         
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return (
            operatorXbox.getXButton()
            || operatorXbox.getAButton()
            || operatorXbox.getBButton()
        );
    }

    public void increaseVelocity() {
        changeVelocity(1);        
    }

    public void decreaseVelocity() { //Dpad velocity decrease step
        changeVelocity(-1);        
    }

    public void changeVelocity(double amount) {
        targetVelocity = MathUtil.clamp(
            targetVelocity + amount,
            TurretConstants.Launcher.MINIMUM_VELOCITY,
            TurretConstants.Launcher.MAXIMUM_VELOCITY
        );
    }
}

