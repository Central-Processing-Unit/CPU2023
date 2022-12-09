package org.firstinspires.ftc.teamcode;

import com.chsrobotics.ftccore.actions.integratedactions.SampleIMUAction;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.chsrobotics.ftccore.hardware.config.accessory.AccessoryType;
import com.chsrobotics.ftccore.teleop.Drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class CPUTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Config config = new Config.Builder()
                .setDriveMotors("m0", "m1", "m2", "m3")
                .addAccessory(new Accessory(AccessoryType.MOTOR, "l0"))
                .addAccessory(new Accessory(AccessoryType.MOTOR, "c0"))
                .setIMU("imu")
                .setTeleopValues(.73, .5)
//                .addAccessory(new Accessory(AccessoryType.WEBCAM, "webcam"))
                .setOpMode(this)
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        manager.accessoryMotors[1].setDirection(DcMotorSimple.Direction.REVERSE);

        Drive drive = new Drive.Builder(manager)
                .addUserLoop(new CPULoop(manager, this))
                .build();

        manager.accessoryMotors[1].setPower(0.3);

        waitForStart();
        drive.runDriveLoop();
    }
}
