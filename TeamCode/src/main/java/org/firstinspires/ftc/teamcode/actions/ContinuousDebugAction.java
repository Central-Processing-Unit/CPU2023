package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.chsrobotics.ftccore.actions.ContinuousAction;
import com.chsrobotics.ftccore.engine.localization.LocalizationEngine;
import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.pipeline.Pipeline;
import com.chsrobotics.ftccore.pipeline.PipelineStep;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.LBRRAuto;

import java.util.ArrayList;

public class ContinuousDebugAction extends ContinuousAction {
    private final FtcDashboard dashboard;
    public static Pipeline pipeline;

    public ContinuousDebugAction(HardwareManager hardware, FtcDashboard dashboard) {
        super(hardware);
        this.dashboard = dashboard;
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();

        packet.fieldOverlay()
                .setStrokeWidth(1)
                .setFill("goldenrod")
                .setStroke("black")
                .fillCircle(LocalizationEngine.position.x, LocalizationEngine.position.y, 1);

        ArrayList<Position> positions = new ArrayList<>();

        positions.add(new Position(0, 0, 0));

        for (PipelineStep step : pipeline.steps)
        {
            if (step.type == PipelineStep.StepType.NAVIGATION)
            {
                assert step.path != null;

                positions.addAll(step.path.positions);
            }
        }

        double[] xPoints = new double[positions.size()];
        double[] yPoints = new double[positions.size()];

        for (int i = 0; i < positions.size(); i++)
        {
            xPoints[i] = positions.get(i).x;
            yPoints[i] = positions.get(i).y;
        }

        packet.fieldOverlay().strokePolyline(xPoints, yPoints);

        dashboard.sendTelemetryPacket(packet);




    }

    @Override
    public void initialize() {

    }
}
