package org.firstinspires.ftc.teamcode.cv;

import com.chsrobotics.ftccore.utilities.ComputerVision;

import org.opencv.core.Mat;

public class CVUtil {

    public static Mat grabbedFrame;
    public static void getMat()
    {
        grabbedFrame = ComputerVision.grabFrame();
    }
}
