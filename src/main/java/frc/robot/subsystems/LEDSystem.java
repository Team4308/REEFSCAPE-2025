package frc.robot.subsystems;

import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.util.sendable.Sendable;

public class LEDSystem extends LogSubsystem {

    public final Spark blinkin;

    public LEDSystem(){
        blinkin = new Spark(0);
    }

    public void CoralInCorrectPlace() {
        if (true) { // Add Logic Later When done for not just default true 
            blinkin.set(0.75); 
        } else {
            blinkin.set(-0.11); // Blink Red
        }
    }


    public void PickUp () {
        blinkin.set(0.71); // Coral
    }

    public Sendable log() {
        return this;
    }
}