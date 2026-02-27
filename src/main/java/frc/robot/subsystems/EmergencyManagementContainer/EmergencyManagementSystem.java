/*
 * This File contains the Emergecy responce tht will react
 * when a specific button is press. (EMS's in act:1)
 */

package frc.robot.subsystems.EmergencyManagementContainer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.ClimberCommand.ClimbPresets;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePresets;
import frc.robot.subsystems.LauncherSubsystem.LauncherPresets;

public class EmergencyManagementSystem extends SubsystemBase {
    //use this space to defind other thing before linker
    
    public EMSoutput EMSoutput;
    private EmergencyManagementSystem emergencyManagementSystem;
    private IntakeSubsystem subsystem;
    private LauncherSubsystem subsystem2;
    private ClimberSubsystem subsystem3;
    
    public EmergencyManagementSystem() { //Linker

    }

    @SuppressWarnings("static-access")
    public void runEMS() { //Executer
        if (EMSIntake) {
            emergencyManagementSystem.setEMSoutput(EMSoutput.INTAKETRIGGER);
        }
        if (EMSLauncher) {
            emergencyManagementSystem.setEMSoutput(EMSoutput.LAUNCHERTRIGGER);

        }
        if (EMSClimber) {
            emergencyManagementSystem.setEMSoutput(EMSoutput.CLIMBERTRIGGER);
            new WaitCommand(5);
            emergencyManagementSystem.setEMSoutput(EMSoutput.INACTIVE);
        }
        else {
            emergencyManagementSystem.setEMSoutput(EMSoutput.INACTIVE);
        }

    }
    public EmergencyManagementSystem(IntakeSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    @SuppressWarnings("static-access")
    public void setEMSoutput(EMSoutput active){
        this.EMSoutput = active;

        if (EMSoutput.INTAKETRIGGER == active) {
            SmartDashboard.putString("Emergency Management System", "Intake has triggered 'Emergency Mode'");
            subsystem.setIntakePreset(IntakePresets.INTAKECLEAR);
        }
        if (EMSoutput.LAUNCHERTRIGGER == active) {
            SmartDashboard.putString("Emergency Management System","Launcher has triggered 'Emergency Mode");
            subsystem2.setLauncherPresets(LauncherPresets.EMS);
        }
        if (EMSoutput.CLIMBERTRIGGER == active) {
            SmartDashboard.putString("Emergency Management System","Climber has triggered 'Emergency Mode'");
            subsystem3.setClimbPreset(ClimbPresets.CLIMBINGEMS);
        }
        else {
            SmartDashboard.putString("Emergency Management System", "System Inactive, no emergencies being delt with");
        }
    }

    public enum EMSoutput{ //Control panel. Developed Triggers: Launcher, Intake, Climber. Developed Buttons: Intake, Launcher, Climber.
        INACTIVE,
        INTAKETRIGGER,
        LAUNCHERTRIGGER,
        CLIMBERTRIGGER,
    }

    public static Boolean EMSIntake;
    
    public static void setEMSIntake(Boolean emsintake) {
        EmergencyManagementSystem.EMSIntake = emsintake;
    } 
    public boolean getEMSIntake() {
        return EMSIntake;
    }
    public static Boolean EMSLauncher;
    
    public static void setEMSLauncher(Boolean emslauncher) {
        EmergencyManagementSystem.EMSLauncher = emslauncher;
    } 
    public boolean getEMSLauncher() {
        return EMSLauncher;
    }
    public static Boolean EMSClimber;
    
    public static void setEMSClimber(Boolean emsclimber) {
        EmergencyManagementSystem.EMSClimber = emsclimber;
    } 
    public boolean getEMSClimber() {
        return EMSClimber;
    }
}   
