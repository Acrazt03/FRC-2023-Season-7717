package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperTowerSubSystem;

public class MoveGripper extends CommandBase{
    private final GripperTowerSubSystem gripperTowerSubSystem;

    private final double m_tolerance = 2;

    public Double gripperExtension;

    public MoveGripper(GripperTowerSubSystem subsystem, Double _gripperExtension){
        gripperTowerSubSystem = subsystem;
        gripperExtension = _gripperExtension;

        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        Double gripperCurrentPosition = gripperTowerSubSystem.getGripperPosition();

        if(gripperCurrentPosition < gripperExtension){
            //Move Forward
            gripperTowerSubSystem.moveGripper(0.75f);
        }else if (gripperCurrentPosition > gripperExtension){
            //Move Backward
            gripperTowerSubSystem.moveGripper(-0.75f);
        }
    }

    private Boolean isWithinTolerance(Double currentPosition, Double desiredPosition){
        return Math.abs(currentPosition - desiredPosition) < m_tolerance;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        gripperTowerSubSystem.stop();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        Double gripperCurrentPosition = gripperTowerSubSystem.getGripperPosition();
        Boolean isGipperOnTolerance = isWithinTolerance(gripperExtension, gripperCurrentPosition);

        return isGipperOnTolerance;
    }
}
