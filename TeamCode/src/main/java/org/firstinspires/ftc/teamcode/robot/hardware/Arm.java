// set points for arm extension

package org.firstinspires.ftc.teamcode.robot.hardware;

public class Arm {
    public void init(@NonNull HardwareMap hardwareMap, String name) {
        arm = hardwareMap.get(Servo.class, name);
    }
    public String getTelemetry(String name) {
        return String.format("Open %S", name);
    }
}
