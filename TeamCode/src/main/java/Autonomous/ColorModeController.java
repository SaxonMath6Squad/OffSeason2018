package Autonomous;

import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by robotics on 9/22/17.
 */

/*
    A class to help us tell what color the color sensor sees
 */
public class ColorModeController {
    public enum type{
        JEWEL_SNATCH_O_MATIC, GLYPH_STACK_O_TRON
    }

    public enum color{
        BLUE, RED, GREY, BROWN, UNKNOWN
    }

    type colorMode;
    ColorSensor colorSensor;
    float hsvValues[] = {0F, 0F, 0F};
    final double SCALE_FACTOR = 255;

    public ColorModeController(type m, ColorSensor sensor){
        setColorMode(m);
        colorSensor = sensor;
    }

    public color getColor(){
        color color = ColorModeController.color.UNKNOWN;
        android.graphics.Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                (int) (colorSensor.green() * SCALE_FACTOR),
                (int) (colorSensor.blue() * SCALE_FACTOR),
                hsvValues);
        switch (colorMode){
            case JEWEL_SNATCH_O_MATIC:
                if(red() && !blue()) color = ColorModeController.color.RED;
                else if(blue() && !red()) color = ColorModeController.color.BLUE;
                else color = ColorModeController.color.UNKNOWN;
                break;
            case GLYPH_STACK_O_TRON:
                if(brown() && !grey()) color = ColorModeController.color.BROWN;
                else if(grey() && !brown()) color = ColorModeController.color.GREY;
                else color = ColorModeController.color.UNKNOWN;
                break;
        }
        return color;
    }

    public void setColorMode(type m) {colorMode = m;}

    public boolean red(){
        boolean red = false;
        if(hsvValues[0] < 70 || hsvValues[0] > 300) red = true;
        return red;
    }
    public boolean blue(){
        boolean blue = false;
        if(hsvValues[0] < 270 && hsvValues[0] > 80) blue = true;
        return blue;
    }
    public boolean grey(){
        boolean grey = false;
        if(hsvValues[1] < 0.23) grey = true;
        return grey;
    }
    public boolean brown(){
        boolean brown = false;
        if(hsvValues[1] > 0.27) brown = true;
        return brown;
    }
}
