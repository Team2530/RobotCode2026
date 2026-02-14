// Copyright (c) CNrailfaner4039 and other contributors.
// Copyright (c) FIRST, 2530 contributors, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.package frc.robot.subsystems;
package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem.IntakePresets;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeCommand extends Command {
    
    private final IntakeSubsystem subsystem;

    

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
            if (start == 1) {
                subsystem.setIntakePreset(IntakePresets.INTAKE);
                    if (Robot.isSimulation()) {
                    SmartDashboard.putBoolean("[SIM] Collecting", true);
                
                }
            }
            if (start == 0) {
                setDown(0);
                subsystem.setIntakePreset(IntakePresets.IDLE);
                if (Robot.isSimulation()) {
                    SmartDashboard.putBoolean("[SIM] Collecting", false);
                }
            }
        
        }
        if (EMS == 1) {
            subsystem.setIntakePreset(IntakePresets.INTAKECLEAR);
        }
        else {
            subsystem.setIntakePreset(IntakePresets.IDLE);
        }
        
    }
    private static double start;
    
    public static void setStart(double start) {
        IntakeCommand.start = start;
    } 
    public double getstart() {
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
    //EMS (Emergency Management system)
    private static double EMS;
    
    public static void setEMS(double ems) {
        IntakeCommand.EMS = ems;
    } 
    public double getEMG() {
        return EMS;
    }
}
