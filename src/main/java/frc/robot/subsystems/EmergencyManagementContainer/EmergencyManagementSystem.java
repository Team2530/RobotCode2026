/*
 * This File contains the Emergecy responce tht will react
 * when a specific button is press. (EMS's in act:1)
 */

package frc.robot.subsystems.EmergencyManagementContainer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EmergencyManagementSystem extends SubsystemBase {
    //use this space to defind other thing before linker
    
    public EMSoutput EMSoutput;
    private EmergencyManagementSystem emergencyManagementSystem;
    
    public EmergencyManagementSystem() { //Linker

    }

    @SuppressWarnings("static-access")
    public void runEMS() { //Executer
        if (EMSIntake) {
            emergencyManagementSystem.setEMSoutput(EMSoutput.INTAKETRIGGER);
        }
        else {
            emergencyManagementSystem.setEMSoutput(EMSoutput.INACTIVE);
        }
    }
    
    @SuppressWarnings("static-access")
    public void setEMSoutput(EMSoutput active){
        this.EMSoutput = active;

        if (EMSoutput.INTAKETRIGGER == active) {
            SmartDashboard.putString("Emergency Management System", "Intake has triggered 'Emergency Mode'");
        }
        else {
            SmartDashboard.putString("Emergency Management System", "System Inactive, no emergencies being delt with");
        }
    }

    public enum EMSoutput{ //Control panel, only intake has this system installed
        INACTIVE,
        INTAKETRIGGER,
    }

    public static Boolean EMSIntake;
    
    public static void setEMSIntake(Boolean emsintake) {
        EmergencyManagementSystem.EMSIntake = emsintake;
    } 
    public boolean getEMSIntake() {
        return EMSIntake;
    }
}   
