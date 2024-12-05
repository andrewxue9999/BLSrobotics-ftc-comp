// two servos that turn; takes difference of two servos to make change the height of claw
// opposite spin: spins in place
// same spin: moves up/down

package org.firstinspires.ftc.teamcode.robot.hardware;

public class DifferentialWrist {
    public void init(@NonNull HardwareMap hardwareMap, String name) {
        wrist = hardwareMap.get(Servo.class, name);
    }
    public String getTelemetry(String name) {
        return String.format("Open %S", name);
    }
}
