package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;


public class LEDs extends SubsystemBase { //
    int port;  //possibly 9?
    public final EventLoop loop = new EventLoop();


    private BooleanSupplier noNote;
    AddressableLED led = new AddressableLED(port);
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(60);
    private boolean shooting;
    private boolean wrong;
    private BooleanEvent hasNote=
            new BooleanEvent(loop,noNote).debounce(0.1);


    public void robotInit(){

        led.setLength(60);
        for(int i = 0; i < 60; i++)
            ledBuffer.setHSV(i, 0,100,100);
        led.setData(ledBuffer); //setting the length and data, apparently this is a very consuming command, so i will just initialize iy here

    }
    public void ledMain(){

    }
    public void setColour(String colour){
        switch(colour.toLowerCase()){
            case "red":
                for(int i = 0; i < 60; i++)
                    ledBuffer.setHSV(i,0, 100, 75);
                led.setData(ledBuffer);
                break;
            case "orange":
                for(int i = 0; i < 60; i++)
                    ledBuffer.setHSV(i,30,100,75);
                led.setData(ledBuffer);
                break;
            case "white":
                for(int i = 0; i < 60; i++)
                    ledBuffer.setHSV(i, 0,0,100);
                led.setData(ledBuffer);
            case "green":
                for(int i = 0; i<60; i++)
                    ledBuffer.setHSV(i, 120,100,75);
                led.setData(ledBuffer);
                break;
        }
    }
    public void flashColour(LEDs led, int h, int s, int v){}




}
