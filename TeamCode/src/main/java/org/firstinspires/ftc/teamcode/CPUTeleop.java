package org.firstinspires.ftc.teamcode;

import com.chsrobotics.ftccore.actions.integratedactions.SampleIMUAction;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.chsrobotics.ftccore.hardware.config.accessory.AccessoryType;
import com.chsrobotics.ftccore.teleop.Drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class CPUTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Config config = new Config.Builder()
                .setDriveMotors("m0", "m1", "m2", "m3")
                .addAccessory(new Accessory(AccessoryType.MOTOR, "l0"))
                //.addAccessory(new Accessory(AccessoryType.MOTOR, "c0"))
                .setIMU("imu")
                .setTeleopValues(1, .6)
                .setMotorDirection(DcMotorSimple.Direction.FORWARD)
                .addAccessory(new Accessory(AccessoryType.SERVO, "s0"))
                .addAccessory(new Accessory(AccessoryType.SERVO, "s1"))
//                .addAccessory(new Accessory(AccessoryType.WEBCAM, "webcam"))
                .setOpMode(this)
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        Drive drive = new Drive.Builder(manager)
                .useScale(Drive.Builder.ScaleMode.SIN_SCALE)
                .addUserLoop(new CPULoop(manager, this))
                .build();

        waitForStart();

        drive.runDriveLoop();
    }
}
