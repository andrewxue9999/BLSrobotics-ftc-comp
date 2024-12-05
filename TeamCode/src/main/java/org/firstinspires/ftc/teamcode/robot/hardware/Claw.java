// "ryan ramu's code"

package org.firstinspires.ftc.teamcode.robot.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Claw {

    private Servo clawServo;
    private boolean isOpen = false;
    private double lastTime = 0;
    private final double TIME_DELAY = 0.15;

    public void init(@NonNull HardwareMap hardwareMap, String name) {
        clawServo = hardwareMap.get(Servo.class, name);

    }

    public void toggle(ElapsedTime time) {
        if (time.time() > lastTime + TIME_DELAY) {
            if (isOpen) {
                clawServo.setPosition(0.0);
                isOpen = false;
            }
            else {
                clawServo.setPosition(1.0);
                isOpen = true;
            }
        }

        lastTime = time.time();
    }

    public boolean getState() {
        return isOpen;
    }

    public String getTelemetry() {
        return String.format("Open %B", isOpen);
    }

}

