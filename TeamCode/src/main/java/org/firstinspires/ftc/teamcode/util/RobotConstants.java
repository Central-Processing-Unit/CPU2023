package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.chsrobotics.ftccore.geometry.Position;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class RobotConstants {
    public static PIDCoefficients rotation = new PIDCoefficients(550, 0.04, 0.1);
    public static PIDCoefficients linear = new PIDCoefficients(3, 0.0008, 0.1);
}
