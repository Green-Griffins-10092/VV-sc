package org.firstinspires.ftc.griffins.Testing;

/*
Modern Robotics Color Sensor Active & Passive Example
Created 7/25/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 1.6
Reuse permitted with credit where credit is due

Configuration:
Device Interface Module named "Device Interface Module 1"
Color sensor at default I2C address of 0x3C named "color"
Touch sensor named "t" and connected to the port specified by the user in their config file

This program can be run without a battery and Power Destitution Module

For more information, visit modernroboticsedu.com.
Support is available by emailing support@modernroboticsinc.com.
*/

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "color sensor test", group = "test")
@Disabled
public class MRI_ColorSensor extends LinearOpMode {

    ColorSensor colorSensor;       //Instance of ColorSensor - for reading color
    TouchSensor touch;             //Instance of TouchSensor - for changing color sensor mode
    DeviceInterfaceModule CDI;     //Instance of DeviceInterfaceModule - for showing a red or blue LED

    @Override
    public void runOpMode() throws InterruptedException {
        //the below three lines set up the configuration file
        colorSensor = hardwareMap.colorSensor.get("color");
        touch = hardwareMap.touchSensor.get("t");
        //We named the CDI using the default name given by the FTC configuration file
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");


        boolean touchState = false;  //Tracks the last known state of the touch sensor
        boolean LEDState = true;     //Tracks the mode of the color sensor; Active = true, Passive = false

        waitForStart();  //Wait for the play button to be pressed

        colorSensor.enableLed(LEDState);  //Set the mode of the LED; Active = true, Passive = false
        //Active - For measuring reflected light. Cancels out ambient light
        //Passive - For measuring ambient light, eg. the FTC Color Beacon

        float hsvValues[] = {0, 0, 0};  //used to get Hue

        while (opModeIsActive()) {  //Main loop of program

            //The below two if() statements ensure that the mode of the color sensor is changed only once each time the touch sensor is pressed.
            //The mode of the color sensor is saved to the sensor's long term memory. Just like flash drives, the long term memory has a life time in the 10s or 100s of thousands of cycles.
            //This seems like a lot but if your program wrote to the long term memory every time though the main loop, it would shorten the life of your sensor.

            if (!touchState && touch.isPressed()) {  //If the touch sensor is just now being pressed (was not pressed last time through the loop but now is)
                touchState = true;                   //Change touch state to true because the touch sensor is now pressed
                LEDState = !LEDState;                //Change the LEDState to the opposite of what it was
                colorSensor.enableLed(LEDState);     //Set the mode of the color sensor using LEDState
            }
            if (!touch.isPressed()) {                //If the touch sensor is now pressed
                touchState = false;                  //Set the touchState to false to indicate that the touch sensor was released
            }

            //calculate hue
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

            //display values
            telemetry.addData("2 Clear", colorSensor.alpha());
            telemetry.addData("3 Red  ", colorSensor.red());
            telemetry.addData("4 Green", colorSensor.green());
            telemetry.addData("5 Blue ", colorSensor.blue());
            telemetry.addData("6 Hue", hsvValues[0]);
            telemetry.addData("7 Touch", touch.isPressed());

            //illuminate the RED/BLUE LED on the Core Device Interface if the RED/BLUE value is greatest
            if (colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green()) {
                CDI.setLED(1, true);           //Red ON
                CDI.setLED(0, false);          //Blue OFF
            } else if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()) {
                CDI.setLED(1, false);          //Red OFF
                CDI.setLED(0, true);           //Blue ON
            } else {
                CDI.setLED(1, false);           //Red OFF
                CDI.setLED(0, false);           //Blue OFF
            }

            telemetry.update();
        } //End main loop of program
    }
}