package org.firstinspires.ftc.teamcode.mech;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensor {
    NormalizedColorSensor colorSensor;

    public enum detectedColor{
        RED,

        PURPLE,

        BLUE,

        LGREEN,

        UNKNOWN,
    }

    public ColorSensor(HardwareMap hwMap) {
        colorSensor = hwMap.get(NormalizedColorSensor.class, "colorsens");
        colorSensor.setGain(5);
    }

    public int colorIndex(){
        if(getColor().equals("green")){
            return 1;
        } else if (getColor().equals("purple")){
            return 2;
        } else if (getColor().equals("unknown")){
            return 0; // that means its empty
        }
        return 0;
    }
    //this is telemetry
    public String getColor(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();//return 4 values
        float normRed, normGreen, normBlue;
        normRed = colors.red/colors.alpha;
        normGreen = colors.green/colors.alpha;
        normBlue = colors.blue/ colors.alpha;
        //identifies which color is being viewed
        if(normRed > .5 && normGreen < .3 && normBlue < .3){
            return  "red";
        } else if (normRed < .3 && normGreen > .45 && normBlue < .3) {
            return "green";
        } else if (normRed < .5 && normGreen < .14 && normBlue < .6) {
            return "purple";
        }else if (normRed < .3 && normGreen < .4 && normBlue > .4){
            return "blue";
        } else{
            return "unknown";
        }


    }
    public detectedColor getDetectedColor(Telemetry telemetry){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();//return 4 values

        float normRed, normGreen, normBlue;
        normRed = colors.red/colors.alpha;
        normGreen = colors.green/colors.alpha;
        normBlue = colors.blue/ colors.alpha;

        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);

        //need to add if statements for specific colors added
        /*
        * (R,G,B)
        * red = (1, .29, .61)
        * green = (.57, 1, .75)
        * blue = (.27, .49, .75)
        * purple = (.67, .14, 1)
        * */
        //identifies which color is being viewed
        if(normRed > .5 && normGreen < .3 && normBlue < .3){
            return  detectedColor.RED;
        } else if (normRed < .3 && normGreen > .45 && normBlue < .3) {
            return detectedColor.LGREEN;
       } else if (normRed < .5 && normGreen < .14 && normBlue < .6) {
            return detectedColor.PURPLE;
        }else if (normRed < .3 && normGreen < .4 && normBlue > .4){
            return detectedColor.BLUE;
        } else{
            return detectedColor.UNKNOWN;
        }
    }
}
