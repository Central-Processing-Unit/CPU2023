package org.firstinspires.ftc.teamcode.opmodes;

import com.chsrobotics.ftccore.engine.navigation.path.PrecisionMode;
import com.chsrobotics.ftccore.engine.navigation.path.TrapezoidalMotionProfile;
import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.chsrobotics.ftccore.hardware.config.accessory.AccessoryType;
import com.chsrobotics.ftccore.pipeline.Pipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.actions.ContinuousLiftAction;
import org.firstinspires.ftc.teamcode.actions.SetClawAction;
import org.firstinspires.ftc.teamcode.actions.UpdateLiftAction;
import org.firstinspires.ftc.teamcode.actions.WaitAction;
import org.firstinspires.ftc.teamcode.actions.WaitLiftAction;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.SignalSleeveDetector;

public class RBLRAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Config config = new Config.Builder()
                .setDriveMotors("m0", "m1", "m2", "m3")
                .addAccessory(new Accessory(AccessoryType.ODOMETRY_POD, "p0"))
                .addAccessory(new Accessory(AccessoryType.ODOMETRY_POD, "p1"))
                .setOdometryWheelProperties(537.7f, 96, 0, 0)

                .addAccessory(new Accessory(AccessoryType.MOTOR, "l0"))
                .addAccessory(new Accessory(AccessoryType.SERVO, "s0"))
                .addAccessory(new Accessory(AccessoryType.SERVO, "s1"))
                .addAccessory(new Accessory(AccessoryType.WEBCAM, "webcam"))

                .useCV()

                .setNavigationTolerances(RobotConstants.mediumPrecision)
                .setHighPrecisionTolerances(RobotConstants.highPrecision)
                .setLowPrecisionTolerances(RobotConstants.lowPrecision)
                .setPIDCoefficients(RobotConstants.linear, RobotConstants.rotation)


                .setMotorDirection(DcMotorSimple.Direction.FORWARD)
                .setIMU("imu")

                .setDebugMode(true)
                .setOpMode(this)
                .useDegrees(true)
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        SignalSleeveDetector.initializeTensorFlow(manager, telemetry);

        waitForStart();

        double parkingPose = 0;

        SignalSleeveDetector.Zone zone = SignalSleeveDetector.detectZone();

        if (zone == SignalSleeveDetector.Zone.ZONE_ONE)
            parkingPose = 500;
        else if (zone == SignalSleeveDetector.Zone.ZONE_THREE)
            parkingPose = -500;

        Pipeline pipeline = new Pipeline.Builder(manager)
                .addAction(new ContinuousLiftAction(manager))
                .addAction(new SetClawAction(manager, true)) //Close claw
                .addAction(new UpdateLiftAction(manager, 300)) //Set claw up a little bit to prevent dragging
                .addLinearPath(PrecisionMode.LOW, new TrapezoidalMotionProfile(600, 1000), //Drive to C1
                        new Position(-610, 100, 0 ))

                .addAction(new UpdateLiftAction(manager, 3000)) //Raise lift to 3000 ticks to prevent tipping
                .addLinearPath(PrecisionMode.LOW, new TrapezoidalMotionProfile(600, 1300), //Drive to C3
                        new Position(-610, 1100, 0))

                .addAction(new UpdateLiftAction(manager, 4400)) //Raise lift to max height
                .addLinearPath(PrecisionMode.MEDIUM, new TrapezoidalMotionProfile(600, 1000), //Drive to B3
                        new Position(-305, 1100, 0))
                .addAction(new WaitLiftAction(manager)) //Wait for lift to climb to proper height
                .addAction(new SetClawAction(manager, false)) //Release claw
                .addAction(new WaitAction(manager, 300)) //Wait for 300ms so claw can release
                .addAction(new UpdateLiftAction(manager, 850)) //Lower lift to top cone of stack

                .addLinearPath(PrecisionMode.MEDIUM, new TrapezoidalMotionProfile(600, 1000), //Drive to A3
                        new Position(520, 950, 270, 1))
                .addAction(new SetClawAction(manager, true)) //Close claw
                .addAction(new WaitAction(manager, 300)) //Wait for 300ms so claw can close
                .addAction(new UpdateLiftAction(manager, 1500)) //Raise lift to prevent from dragging stack down
                .addAction(new WaitLiftAction(manager)) //Wait for lift to climb to proper height
                .addAction(new UpdateLiftAction(manager, 4400)) //Raise lift to max height

                .addLinearPath(PrecisionMode.MEDIUM, new TrapezoidalMotionProfile(600, 1000), //Drive to B3
                        new Position(-305, 1100, 0, 1))
                .addAction(new WaitLiftAction(manager)) //Wait for lift to climb to proper height
                .addAction(new SetClawAction(manager, false)) //Release claw
                .addAction(new WaitAction(manager, 300)) //Wait for 300ms so claw can release
                .addAction(new UpdateLiftAction(manager, 750)) //Lower lift to top cone of stack

                .addLinearPath(PrecisionMode.MEDIUM, new TrapezoidalMotionProfile(600, 1000), //Drive to A3
                        new Position(520, 950, 270, 1))
                .addAction(new SetClawAction(manager, true)) //Close claw
                .addAction(new WaitAction(manager, 300)) //Wait for 300ms so claw can close
                .addAction(new UpdateLiftAction(manager, 1500)) //Raise lift to prevent from dragging stack down
                .addAction(new WaitLiftAction(manager)) //Wait for lift to climb to proper height
                .addAction(new UpdateLiftAction(manager, 4400)) //Raise lift to max height

                .addLinearPath(PrecisionMode.MEDIUM, new TrapezoidalMotionProfile(600, 1000), //Drive to B3
                        new Position(-305, 1100, 0, 1))
                .addAction(new WaitLiftAction(manager)) //Wait for lift to climb to proper height
                .addAction(new SetClawAction(manager, false)) //Release claw
                .addAction(new WaitAction(manager, 300)) //Wait for 300ms so claw can release
                .addAction(new UpdateLiftAction(manager, 100)) //Lower lift to bottom

                .addLinearPath(PrecisionMode.MEDIUM, new TrapezoidalMotionProfile(600, 1000), //Drive to B3, Parking Position
                        new Position(-305, 950, 0),
                        new Position(-parkingPose, 950, 0))
                .build();

        pipeline.execute();
    }
}
