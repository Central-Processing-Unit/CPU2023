package org.firstinspires.ftc.teamcode.opmodes;

import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.chsrobotics.ftccore.hardware.config.accessory.AccessoryType;
import com.chsrobotics.ftccore.management.constants.MiscConstants;
import com.chsrobotics.ftccore.pipeline.Pipeline;
import com.chsrobotics.ftccore.vision.CVUtility;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.actions.ContinuousLiftAction;
import org.firstinspires.ftc.teamcode.actions.SetClawAction;
import org.firstinspires.ftc.teamcode.actions.UpdateLiftAction;
import org.firstinspires.ftc.teamcode.actions.WaitAction;
import org.firstinspires.ftc.teamcode.actions.WaitLiftAction;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

@Autonomous
@com.acmerobotics.dashboard.config.Config
public class CPUNorthBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        int parkingPos = 0;

        Config config = new Config.Builder()
                .setDriveMotors("m0", "m1", "m2", "m3")
                .setOdometryWheelProperties(537.7f, 96, 0, 0)
//                .addAccessory(new Accessory(AccessoryType.MOTOR, "l0"))
//                .addAccessory(new Accessory(AccessoryType.MOTOR, "c0"))
                .addAccessory(new Accessory(AccessoryType.SERVO, "s0"))
                .addAccessory(new Accessory(AccessoryType.SERVO, "s1"))
                .setNavigationTolerances(10, 0.1)
                .useDegrees(true)
//                .addAccessory(new Accessory(AccessoryType.WEBCAM, "webcam"))
                .setMotorDirection(DcMotorSimple.Direction.FORWARD)
                .setIMU("imu")
                .setOpMode(this)
                .setPIDCoefficients(RobotConstants.linear, RobotConstants.rotation)
                //.setPIDCoefficients(new PIDCoefficients(3.2, 0.001,0 ), new PIDCoefficients(550, 0.04, 0))
                //.useCV()
                //.setDebugMode(true)
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        //manager.accessoryMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);

//        manager.accessoryServos[0].setPosition(0.55);
//        manager.accessoryServos[1].setPosition(0.55);
//
//        manager.accessoryServos[0].setPosition(0.45);
//        manager.accessoryServos[1].setPosition(0.45);

//        CVUtility cv = null;
//        try {
//            cv = new CVUtility(manager, telemetry);
//        } catch (Exception e) {
//            telemetry.addLine("CVUtility failed to initialized");
//            telemetry.update();
//        }

        Pipeline pipeline = new Pipeline.Builder(manager)
                .addLinearPath(
                        new Position(0, 1120, 315),
                        new Position(-580, 1000, 90),
                        new Position(0, 1120, 315),
                        new Position(-580, 1000, 90),
                        new Position(0, 1120, 315)
                        )

                .build();

        waitForStart();

//        int dots = 1;
//        if (cv != null && cv.initialized && cv.grabFrame() != null) {
//            dots = org.firstinspires.ftc.teamcode.auto.util.SignalSleeveDetector.detectOrientation(cv.grabFrame());
//
//            telemetry.addData("Dots: ", dots);
//            cv.stopStreaming();
//        } else {
//            telemetry.addLine("Signal sleeve detection failed");
//        }
//
//        parkingPos = dots == 1 ? -600 :
//                (dots == 2 ? 0 : 700);

        pipeline.execute();

    }
}
