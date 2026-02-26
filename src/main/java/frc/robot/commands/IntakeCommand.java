
package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem.IntakePresets;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.EmergencyManagementContainer.EmergencyManagementSystem.EMSoutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.EmergencyManagementContainer.EmergencyManagementSystem;

public class IntakeCommand extends Command {
    
    private final IntakeSubsystem subsystem;
    private EmergencyManagementSystem.EMSoutput EMSIntake;

    

    public IntakeCommand(IntakeSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }
    
     @Override
    public void end(boolean interrupted) {
        subsystem.setIntakePreset(IntakePresets.IDLE);
        SmartDashboard.putString("Intake Ended", "Ended");
    }

     @Override
    public void initialize() {
        if (down == 1) {
            subsystem.setIntakePreset(IntakePresets.OUT);
            SmartDashboard.putString("Intake Command", "Started");
            if (start) {
                subsystem.setIntakePreset(IntakePresets.INTAKE);
                    if (Robot.isSimulation()) {
                    SmartDashboard.putBoolean("[SIM] Collecting", true);
                
                }
            }
            else {
                setDown(0);
                subsystem.setIntakePreset(IntakePresets.IDLE);
                if (Robot.isSimulation()) {
                    SmartDashboard.putBoolean("[SIM] Collecting", false);
                }
            }
        
        }
        if (EMSIntake == EMSoutput.INTAKETRIGGER) {
                    subsystem.setIntakePreset(IntakePresets.INTAKECLEAR);
        }
        else {
            subsystem.setIntakePreset(IntakePresets.IDLE);
        }
        
    }
            private static boolean start;
    
    public static void setStart(Boolean start) {
        IntakeCommand.start = start;
    } 
    public boolean getstart() {
        return start;
    }
/*  
    // DOWN IS BEING DEPRECATED WHEN IT IS NECESSARY \\
    \\  REMOVE AND CHANGE DOWN ( REPLACED BY START ) //
*/
    private static double down;
    
    public static void setDown(double down) {
        IntakeCommand.down = down;
    } 
    public double getdown() {
        return down;
    }

}
