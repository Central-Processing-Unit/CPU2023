package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class TestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx m0 = hardwareMap.get(DcMotorEx.class, "m0");
        DcMotorEx m1= hardwareMap.get(DcMotorEx.class, "m1");
        DcMotorEx m2 = hardwareMap.get(DcMotorEx.class, "m2");
        DcMotorEx m3 = hardwareMap.get(DcMotorEx.class, "m3");

        long time = System.currentTimeMillis() + 2000;

        m0.setPower(1);

        while (System.currentTimeMillis() < time)
        {
            //nothing
        }

        time += 2000;

        m1.setPower(1);

        while (System.currentTimeMillis() < time)
        {
            //nothing
        }

        time += 2000;

        m2.setPower(1);

        while (System.currentTimeMillis() < time)
        {
            //nothing
        }

        time += 2000;

        m3.setPower(1);

        while (System.currentTimeMillis() < time)
        {
            //nothing
        }

    }
}
