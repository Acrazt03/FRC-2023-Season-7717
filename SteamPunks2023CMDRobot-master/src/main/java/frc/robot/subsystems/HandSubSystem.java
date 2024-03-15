package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class HandSubSystem extends SubsystemBase {
    private  DoubleSolenoid gripperSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    
    public HandSubSystem(){
        closeGripper();
    }

    public void toogleGripper(){
        gripperSolenoid.toggle();
    }

    public void closeGripper(){
        gripperSolenoid.set(kReverse);
    }

    public void openGripper(){
        gripperSolenoid.set(kForward);
    }
}
