package frc.robot.commands.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class MatchtimeStatusCommand extends Command {

    //LUKE ADDED THIS MAKE SURE IT IS OK, It posts the match time to smart dashboard
    @Override 
    public void execute() {
        SmartDashboard.putNumber(
            "Meta/Matchtime", 
            Math.floor(DriverStation.getMatchTime())
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
