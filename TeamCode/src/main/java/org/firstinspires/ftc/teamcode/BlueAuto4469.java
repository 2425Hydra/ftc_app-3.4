package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;
import org.firstinspires.ftc.robotcontroller.external.samples.PushbotAutoDriveByTime_Linear;

import java.util.concurrent.SynchronousQueue;


@Autonomous(name = "Blue001")
//@Disabled
//Created by HYDRA ROBOTICS 10/24/17

public class BlueAuto4469 extends LinearOpMode {
    DcMotor left_Drive = null;
    DcMotor right_Drive = null;
    DcMotor winch = null;
    Servo bravo = null;
    Servo charlie = null;
    ColorSensor sensor = null;




    @Override
    public void runOpMode() throws InterruptedException {
        {


            left_Drive = hardwareMap.dcMotor.get("left_Drive");
            right_Drive = hardwareMap.dcMotor.get("right_Drive");
            winch = hardwareMap.dcMotor.get("winch");
            bravo = hardwareMap.servo.get("bravo");
            charlie = hardwareMap.servo.get("charlie");

            sensor = hardwareMap.colorSensor.get("color");
            sensor.setI2cAddress(I2cAddr.create8bit(0x6c));




            left_Drive.setDirection(DcMotorSimple.Direction.REVERSE);
            right_Drive.setDirection(DcMotorSimple.Direction.REVERSE);

            waitForStart();

            bravo.setPosition(0);
            charlie.setPosition(1);

            winch.setPower(.25);
            Thread.sleep(25);
            winch.setPower(0);

            if(sensor.red() > sensor.blue()) {
                left_Drive.setPower(-.25);
                right_Drive.setPower(.25);
                Thread.sleep(300);

                left_Drive.setPower(0);
                right_Drive.setPower(0);

                left_Drive.setPower(.25);
                right_Drive.setPower(-.25);
                Thread.sleep(300);
            } else {
                left_Drive.setPower(.25);
                right_Drive.setPower(-.25);
                Thread.sleep(300);

                left_Drive.setPower(0);
                right_Drive.setPower(0);

                left_Drive.setPower(-.25);
                right_Drive.setPower(.25);
                Thread.sleep(300);
            }

            left_Drive.setPower(1);
            right_Drive.setPower(1);
            Thread.sleep(875);

            left_Drive.setPower(-1);
            right_Drive.setPower(1);
            Thread.sleep(900);

            left_Drive.setPower(.5);
            right_Drive.setPower(.5);
            Thread.sleep(475);

            left_Drive.setPower(0);
            right_Drive.setPower(0);
            Thread.sleep(800);

            bravo.setPosition(1);
            charlie.setPosition(0);


            left_Drive.setPower(-.5);
            right_Drive.setPower(-.5);
            Thread.sleep(200);

            left_Drive.setPower(.5);
            right_Drive.setPower(.5);
            Thread.sleep(500);

            left_Drive.setPower(-.5);
            right_Drive.setPower(-.5);
            Thread.sleep(200);
        }
    }
}
