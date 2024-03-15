package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperTowerSubSystem;

public class MoveTower extends CommandBase {
    private final GripperTowerSubSystem gripperTowerSubSystem; 

    private final double m_tolerance = 5;

    public Double armAngle;
    public Double gripperExtension;
    public Boolean gripperState;

    public MoveTower(GripperTowerSubSystem subsystem, Double _armAngle, Double _gripperExtension, Boolean _gripperState){
        gripperTowerSubSystem = subsystem;
        armAngle = _armAngle;
        gripperExtension = _gripperExtension;
        gripperState = _gripperState;

        addRequirements(subsystem);
    }

    private Double convertPositionToAngle(double position){
        return position; //TODO
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        Double armCurrentAngle = convertPositionToAngle(gripperTowerSubSystem.getArmPosition());

        if(armCurrentAngle < armAngle){
            //Move up
            gripperTowerSubSystem.moveArm(0.75f);
        }else if (armCurrentAngle > armAngle){
            //Move down
            gripperTowerSubSystem.moveArm(-0.75f);
        }

        Double gripperCurrentPosition = gripperTowerSubSystem.getGripperPosition();

        if(gripperCurrentPosition < gripperExtension){
            //Move Forward
            //gripperTowerSubSystem.moveGripper(0.75f);
        }else if (gripperCurrentPosition > gripperExtension){
            //Move Backward
            //gripperTowerSubSystem.moveGripper(-0.75f);
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

        Double armCurrentAngle = convertPositionToAngle(gripperTowerSubSystem.getArmPosition());
        Double gripperCurrentPosition = gripperTowerSubSystem.getGripperPosition();

        Boolean isArmOnTolerance = isWithinTolerance(armAngle, armCurrentAngle);
        Boolean isGipperOnTolerance = true;//isWithinTolerance(gripperExtension, gripperCurrentPosition);

        return isArmOnTolerance && isGipperOnTolerance;
    }

}
