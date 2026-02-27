package org.firstinspires.ftc.teamcode.mechanisms;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Sorter {
    private Servo sorter;
    private Servo flick;
    private final double FINGER_HOME = 1;
    private final double FINGER_ACTIVE = 0.5;
    private final double SORTER_LEFT = 0.06;
    private final double SORTER_HOME = 0.5;
    private final double SORTER_RIGHT = 0.94;

    public void init(HardwareMap hwm) {
        sorter = hwm.get(Servo.class,"sorter");
        flick = hwm.get(Servo.class,"flick");
        sorter.setPosition(SORTER_HOME);
        flick.setDirection(Servo.Direction.FORWARD);
        flick.setPosition(FINGER_HOME);
    }

    public void move(double pos) {
        sorter.setPosition(pos);
    }

    public void home() {
        sorter.setPosition(SORTER_HOME);
    }

    public void left() {
        sorter.setPosition(SORTER_LEFT);
    }

    public void right() {
        sorter.setPosition(SORTER_RIGHT);
    }

    public void flickBall(int timeout) throws InterruptedException {
        sleep(timeout+500);
        flick.setPosition(FINGER_ACTIVE);
        sleep(1000);
        flick.setPosition(FINGER_HOME);
        sleep(timeout-250);
    }

    public void setFinger(double pos) {
        flick.setPosition(pos);
    }

    public void shootBallsInPattern(String pattern) throws InterruptedException {
        int timeout = 1000;
        switch (pattern) {
            case "GPP":
                flickBall(timeout);
                move(SORTER_LEFT);
                flickBall(timeout);
                move(SORTER_RIGHT);
                flickBall(timeout);
                break;
            case "PGP":
                move(SORTER_LEFT);
                flickBall(timeout);
                move(SORTER_HOME);
                flickBall(timeout);
                move(SORTER_RIGHT);
                flickBall(timeout);
                break;
            case "PPG":
                move(SORTER_LEFT);
                flickBall(timeout);
                move(SORTER_RIGHT);
                flickBall(timeout);
                move(SORTER_HOME);
                flickBall(timeout);
                break;
        }
    }
}
