package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp(name="SorterTesting", group="A")
public class SorterTesting extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private final double SERVO_INCREMENT = .09;
    private int[] balls = {0,0,0};
    private int[] pattern = {0,0,0};
    private Servo sorter;
    private Limelight3A limelight;

    public int[] readObelisk() {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
        }
        
        return new int[] {0,0,0};
    }

    @Override
    public void init() {
        sorter = hardwareMap.get(Servo.class, "sorter");
        sorter.setPosition(0.5);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        if (gamepad1.xWasPressed()) {
            moveSorter(-SERVO_INCREMENT);
        } else if (gamepad1.bWasPressed()) {
            moveSorter(SERVO_INCREMENT);
        } else if (gamepad1.yWasPressed()) {
            sorter.setPosition(0.5);
        }
    }

    public void moveSorter(double increment) {
        double currentPos = sorter.getPosition();
        double finalPos = currentPos + increment;
        double extra = increment/10;
        sorter.setPosition(currentPos + extra);
        sorter.setPosition(finalPos);
    }

    public void homeSorter() {
        double currentPos = sorter.getPosition();
        if (0.5 - currentPos < 0) {
            sorter.setPosition(0.48);
        }
        sorter.setPosition(0.5);
    }
}