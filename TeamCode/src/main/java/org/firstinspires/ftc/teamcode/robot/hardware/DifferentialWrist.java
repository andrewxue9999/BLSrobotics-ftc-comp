// two servos that turn; takes difference of two servos to make change the height of claw
// opposite spin: spins in place
// same spin: moves up/down

package org.firstinspires.ftc.teamcode.robot.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DifferentialWrist {
    private Servo wrist;

    public void init(@NonNull HardwareMap hardwareMap, String name) {
        wrist = hardwareMap.get(Servo.class, name);
    }
    public String getTelemetry(String name) {
        return String.format("Open %S", name);
    }
}
