package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperTowerSubSystem;

public class HomeArm extends CommandBase {

    private final GripperTowerSubSystem gripperTowerSubSystem;

    public HomeArm(GripperTowerSubSystem _gripperTowerSubSystem){
        gripperTowerSubSystem = _gripperTowerSubSystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(gripperTowerSubSystem.isGripperHomed()){
            gripperTowerSubSystem.moveArm(-0.5f);
        }else{
            cancel();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        gripperTowerSubSystem.stop();
        gripperTowerSubSystem.resetArmEncoder();

    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return gripperTowerSubSystem.isArmHomed();
    }
}
