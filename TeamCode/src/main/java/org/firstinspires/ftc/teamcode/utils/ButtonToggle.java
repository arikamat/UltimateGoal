package org.firstinspires.ftc.teamcode.utils;

public class ButtonToggle {
    boolean on = false;
    boolean mode = false;
    public ButtonToggle(){}
    public boolean ifPress(boolean gamepadVal){
        if (!on && gamepadVal) {
            mode = !mode;
            return true;
        }

        return false;
    }
    public boolean getMode(){
        return mode;
    }
}
