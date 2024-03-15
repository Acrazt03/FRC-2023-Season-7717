package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperTowerSubSystem;

public class MoveArm extends CommandBase {
    private final GripperTowerSubSystem gripperTowerSubSystem;

    private final double m_tolerance = 5;

    public Double armAngle;

    public MoveArm(GripperTowerSubSystem subsystem, Double _armAngle){
        gripperTowerSubSystem = subsystem;
        armAngle = _armAngle;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        Double armCurrentAngle = gripperTowerSubSystem.getArmPosition();

        if(armCurrentAngle < armAngle){
            //Move up
            gripperTowerSubSystem.moveArm(0.75f);
        }else if (armCurrentAngle > armAngle){
            //Move down
            gripperTowerSubSystem.moveArm(-0.75f);
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

        Double armCurrentAngle = gripperTowerSubSystem.getArmPosition();
        Boolean isArmOnTolerance = isWithinTolerance(armAngle, armCurrentAngle);

        return isArmOnTolerance;
    }

}
