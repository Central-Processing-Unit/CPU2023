package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class TestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo1 = hardwareMap.get(Servo.class, "servo0");
        Servo servo2 = hardwareMap.get(Servo.class, "servo1");

        waitForStart();

        while (!isStopRequested())
        {
            servo1.setPosition(0.2);
            servo2.setPosition(0.2);
        }
    }
}
