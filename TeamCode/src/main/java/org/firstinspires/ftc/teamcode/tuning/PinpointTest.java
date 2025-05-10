package org.firstinspires.ftc.teamcode.tuning;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.hardware.GoBildaPinpointDriver;
import org.opencv.video.DISOpticalFlow;

@Config
@TeleOp(name = "Pinpoint Testing")
public class PinpointTest extends OpMode {

    GoBildaPinpointDriver odo;

    double oldTime = 0;

    public static GoBildaPinpointDriver.EncoderDirection xDirect = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection yDirect = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.setOffsets(0.0, 38.6, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(xDirect, yDirect);

        odo.recalibrateIMU();
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.update();
    }

    @Override
    public void start() {
        resetRuntime();
    }

    @Override
    public void loop() {
        odo.update();

        if (gamepad1.a){
            odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
        }

        if (gamepad1.b){
            odo.recalibrateIMU(); //recalibrates the IMU without resetting position
        }

        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;


    }

}

