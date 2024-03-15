package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Values;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.Compressor;

public class GearSubSystem extends SubsystemBase {
    
    //private Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

    private DoubleSolenoid transmissioSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

    public GearSubSystem(){
        //pcmCompressor.enableDigital();
        transmissioSolenoid.set(kReverse);
    }
    @Override
    public void periodic(){
        SmartDashboard.putBoolean("isInMecanumMode", Values.Variables.isInMecanumMode);

    }
    public void ToggleGearMode(){
        transmissioSolenoid.toggle();
    }

    public void ChangeToMecanum(){
        transmissioSolenoid.set(kForward);
    }

    public void ChangeToTranctionWheels(){
        transmissioSolenoid.set(kReverse);
    }

    public void changeWheels(){
        if(Values.Variables.isInMecanumMode){
            ChangeToTranctionWheels();
        }else{
            ChangeToMecanum();
        }
    }

    public void changeBoolean(){
        Values.Variables.isInMecanumMode = !Values.Variables.isInMecanumMode;
    }
}
