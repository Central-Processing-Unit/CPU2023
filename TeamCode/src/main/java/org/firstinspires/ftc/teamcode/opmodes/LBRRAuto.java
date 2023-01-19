package org.firstinspires.ftc.teamcode.opmodes;

import android.content.res.AssetManager;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.chsrobotics.ftccore.engine.navigation.path.PrecisionMode;
import com.chsrobotics.ftccore.engine.navigation.path.TrapezoidalMotionProfile;
import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.chsrobotics.ftccore.hardware.config.accessory.AccessoryType;
import com.chsrobotics.ftccore.pipeline.Pipeline;
import com.chsrobotics.ftccore.vision.CVUtility;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.actions.ContinuousDebugAction;
import org.firstinspires.ftc.teamcode.actions.ContinuousLiftAction;
import org.firstinspires.ftc.teamcode.actions.SetClawAction;
import org.firstinspires.ftc.teamcode.actions.UpdateLiftAction;
import org.firstinspires.ftc.teamcode.actions.WaitAction;
import org.firstinspires.ftc.teamcode.actions.WaitLiftAction;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.SignalSleeveDetector;

import java.io.FileDescriptor;
import java.io.IOException;

@Autonomous
public class LBRRAuto extends LinearOpMode {
    public static Pipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        Config config = new Config.Builder()
                .setDriveMotors("m0", "m1", "m2", "m3")
                .setOdometryWheelProperties(8192, 70, 6.35, 50.8)
                .addAccessory(new Accessory(AccessoryType.MOTOR, "l0"))
                .addAccessory(new Accessory(AccessoryType.SERVO, "s0"))
                .addAccessory(new Accessory(AccessoryType.SERVO, "s1"))
                .addAccessory(new Accessory(AccessoryType.WEBCAM, "webcam"))
                .addAccessory(new Accessory(AccessoryType.ODOMETRY_POD, "o0"))
                .addAccessory(new Accessory(AccessoryType.ODOMETRY_POD, "o1"))
                .setNavigationTolerances(RobotConstants.mediumPrecision)
                .setHighPrecisionTolerances(RobotConstants.highPrecision)
                .setLowPrecisionTolerances(RobotConstants.lowPrecision)
                .useCV()
                .useDegrees(true)
                .setMotorDirection(DcMotorSimple.Direction.FORWARD)
                .setIMU("imu")
                .setOpMode(this)
                .setPIDCoefficients(RobotConstants.linear, RobotConstants.rotation)
                .setDebugMode(true)
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        manager.accessoryOdometryPods[0].setDirection(DcMotorSimple.Direction.REVERSE);
        manager.accessoryOdometryPods[1].setDirection(DcMotorSimple.Direction.REVERSE);

        manager.accessoryMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);

        manager.accessoryServos[0].setPosition(0.54);
        manager.accessoryServos[1].setPosition(0.46);

        SignalSleeveDetector.initAprilTags(manager);

        SignalSleeveDetector.Zone zone = SignalSleeveDetector.Zone.ZONE_TWO;

        while (!isStarted() && !isStopRequested())
            SignalSleeveDetector.detect(manager);

        waitForStart();

        zone = SignalSleeveDetector.zone;

        SignalSleeveDetector.camera.stopStreaming();

        double parkingPose = 0;

        if (zone == SignalSleeveDetector.Zone.ZONE_ONE)
            parkingPose = 500;
        else if (zone == SignalSleeveDetector.Zone.ZONE_THREE)
            parkingPose = -500;

        pipeline = new Pipeline.Builder(manager)
                .addContinuousAction(new ContinuousLiftAction(manager))
                .addAction(new SetClawAction(manager, true))
                .addAction(new UpdateLiftAction(manager, 4700, 200))
                .addLinearPath(PrecisionMode.LOW, new TrapezoidalMotionProfile(770, 1000),
                        new Position(105, 1300, 315, 1.4))
                .addAction(new SetClawAction(manager, false))
                .addAction(new WaitAction(manager, 300))
                .addAction(new UpdateLiftAction(manager, 1150))

                .addLinearPath(PrecisionMode.MEDIUM, new TrapezoidalMotionProfile(700, 1000), //Drive to B3
                        new Position(-640, 1160, 90, 0.5))
                .addAction(new SetClawAction(manager, true))
                .addAction(new WaitAction(manager, 300))
                .addAction(new UpdateLiftAction(manager, 1500))
                .addAction(new WaitLiftAction(manager))
                .addAction(new UpdateLiftAction(manager, 4900))

                .addLinearPath(PrecisionMode.MEDIUM, new TrapezoidalMotionProfile(700, 1000), //Drive to B3
                        new Position(200, 1200, 0))
                .addAction(new SetClawAction(manager, false))

//
//                .addLinearPath(PrecisionMode.MEDIUM, new TrapezoidalMotionProfile(730, 1000), //Drive to A3
//                        new Position(-520, 950, 90, 0.5))
//                .addAction(new SetClawAction(manager, true)) //Close claw
//                .addAction(new WaitAction(manager, 300)) //Wait for 300ms so claw can close
//                .addAction(new UpdateLiftAction(manager, 1500)) //Raise lift to prevent from dragging stack down
//                .addAction(new WaitLiftAction(manager)) //Wait for lift to climb to proper height
//                .addAction(new UpdateLiftAction(manager, 4400)) //Raise lift to max height
//
//                .addLinearPath(PrecisionMode.MEDIUM, new TrapezoidalMotionProfile(700, 1000), //Drive to B3
//                        new Position(305, 970, 0, 0.5))
//                .addAction(new WaitLiftAction(manager)) //Wait for lift to climb to proper height
//                .addAction(new SetClawAction(manager, false)) //Release claw
//                .addAction(new WaitAction(manager, 300)) //Wait for 300ms so claw can release
//                .addAction(new UpdateLiftAction(manager, 750)) //Lower lift to top cone of stack
//
//                .addLinearPath(PrecisionMode.MEDIUM, new TrapezoidalMotionProfile(730, 1000), //Drive to A3
//                        new Position(-520, 950, 90, 0.5))
//                .addAction(new SetClawAction(manager, true)) //Close claw
//                .addAction(new WaitAction(manager, 300)) //Wait for 300ms so claw can close
//                .addAction(new UpdateLiftAction(manager, 1500)) //Raise lift to prevent from dragging stack down
//                .addAction(new WaitLiftAction(manager)) //Wait for lift to climb to proper height
//                .addAction(new UpdateLiftAction(manager, 4400)) //Raise lift to max height
//
//                .addLinearPath(PrecisionMode.MEDIUM, new TrapezoidalMotionProfile(700, 1000), //Drive to B3
//                        new Position(305, 970, 0, 0.5))
//                .addAction(new WaitLiftAction(manager)) //Wait for lift to climb to proper height
//                .addAction(new SetClawAction(manager, false)) //Release claw
//                .addAction(new WaitAction(manager, 300)) //Wait for 300ms so claw can release
//                .addAction(new UpdateLiftAction(manager, 200)) //Lower lift to top cone of stack
//                .addLinearPath(PrecisionMode.MEDIUM, new TrapezoidalMotionProfile(730, 1000), //Drive to B3, Parking Position
//                        new Position(305, 950, 0),
//                        new Position(parkingPose, 950, 0))
                .build();

        ContinuousDebugAction.pipeline = pipeline;
        pipeline.execute();
    }
}
