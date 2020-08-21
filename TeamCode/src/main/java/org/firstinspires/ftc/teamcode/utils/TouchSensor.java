package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TouchSensor {
    DigitalChannel digitalTouch;
    public TouchSensor(HardwareMap hmap, String name){
        this.digitalTouch = hmap.get(DigitalChannel.class, name);
        this.digitalTouch.setMode(DigitalChannel.Mode.INPUT);
    }
    public boolean isPressed(){
        return (!this.digitalTouch.getState());
    }
}
