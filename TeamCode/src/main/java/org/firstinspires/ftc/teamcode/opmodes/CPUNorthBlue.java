package org.firstinspires.ftc.teamcode.opmodes;

import com.chsrobotics.ftccore.engine.navigation.path.PrecisionMode;
import com.chsrobotics.ftccore.engine.navigation.path.Tolerances;
import com.chsrobotics.ftccore.engine.navigation.path.TrapezoidalMotionProfile;
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
                .addAccessory(new Accessory(AccessoryType.MOTOR, "l0"))
                .addAccessory(new Accessory(AccessoryType.SERVO, "s0"))
                .addAccessory(new Accessory(AccessoryType.SERVO, "s1"))
                .setNavigationTolerances(RobotConstants.mediumPrecision)
                .setHighPrecisionTolerances(RobotConstants.highPrecision)
                .setLowPrecisionTolerances(RobotConstants.lowPrecision)
                .useDegrees(true)
                .addAccessory(new Accessory(AccessoryType.WEBCAM, "webcam"))
                .setMotorDirection(DcMotorSimple.Direction.FORWARD)
                .setIMU("imu")
                .setOpMode(this)
                .setPIDCoefficients(RobotConstants.linear, RobotConstants.rotation)
                .setMotionProfilePIDCoeffs(new PIDCoefficients(4.5, 0.0015, 0.21))
                .useCV()
                .setDebugMode(true)
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        manager.accessoryMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);

        manager.accessoryServos[0].setPosition(0.45);
        manager.accessoryServos[1].setPosition(0.45);

        CVUtility cv = null;
        try {
            cv = new CVUtility(manager, telemetry);
        } catch (Exception e) {
            telemetry.addLine("CVUtility failed to initialized");
            telemetry.update();
        }

        waitForStart();

        int dots = 1;
        if (cv != null && cv.initialized && cv.grabFrame() != null) {
            dots = org.firstinspires.ftc.teamcode.util.SignalSleeveDetector.detectOrientation(cv.grabFrame());

            telemetry.addData("Dots: ", dots);
            cv.stopStreaming();
        } else {
            telemetry.addLine("Signal sleeve detection failed");
        }

        parkingPos = dots == 1 ? -600 :
                (dots == 2 ? 0 : 700);

        Pipeline pipeline = new Pipeline.Builder(manager)
                .addContinuousAction(new ContinuousLiftAction(manager))
                .addAction(new SetClawAction(manager, true))
                .addAction(new UpdateLiftAction(manager, 4200))
                .addLinearPath(PrecisionMode.LOW, new TrapezoidalMotionProfile(600, 1000), false,
                        new Position(-75, 1100, 315))
                .addLinearPath(false, new Position(35, 1050, 315))
                .addAction(new WaitLiftAction(manager))
                .addAction(new SetClawAction(manager, false))
                .addAction(new WaitAction(manager, 300))
                .addAction(new UpdateLiftAction(manager, 900))
                .addLinearPath(false,
                        new Position(-580, 1000, 90))
                .addLinearPath()
                .build();

        pipeline.execute();

    }
}
