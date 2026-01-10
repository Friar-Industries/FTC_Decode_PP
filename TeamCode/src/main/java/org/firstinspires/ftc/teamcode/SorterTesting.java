package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="SorterTesting", group="A")
public class SorterTesting extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private final double SERVO_INCREMENT = .09;
    private Servo sorter;

    @Override
    public void init() {
        sorter = hardwareMap.get(Servo.class, "sorter");
        sorter.setPosition(0.5);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        if (gamepad1.xWasPressed()) {
            sorter.setPosition(sorter.getPosition() - SERVO_INCREMENT);
        } else if (gamepad1.bWasPressed()) {
            sorter.setPosition(sorter.getPosition() + SERVO_INCREMENT);
        } else if (gamepad1.yWasPressed()) {
            sorter.setPosition(0.5);
        }
    }
}