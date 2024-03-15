package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Inputs {
    public static CommandXboxController commandController = new CommandXboxController(0); // Creates a CommandXboxController on port 1.
    public static class bindings{
        public static Boolean isInTurboMode = Inputs.commandController.getHID().getRightBumper();
        
    }

    public void updateInputs(){
        bindings.isInTurboMode = Inputs.commandController.getHID().getRightBumper();
        SmartDashboard.putBoolean("Is Turbo: ", bindings.isInTurboMode);
        //SmartDashboard.putBoolean("Is Mecanum: ", Values.Variables.isInMecanumMode);
    }
}
