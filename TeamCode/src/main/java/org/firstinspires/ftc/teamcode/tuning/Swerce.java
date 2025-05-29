package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.Pose;

import org.firstinspires.ftc.teamcode.robot.hardware.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.Vector2D;

import java.util.List;

@Config
@TeleOp(name = "TeleOp Swerve", group = "Drive")
public class Swerce extends OpMode {

    public SwerveDrivetrain drivetrain;

    @Override
    public void init() {
        drivetrain = new SwerveDrivetrain(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y; // Y is inverted
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        Vector2d drive = new Vector2d(x, y); //DIS DA VECTOR2D FROM TEAMCODE.UTIL

        drivetrain.read();
        drivetrain.setDrivePower(drive, turn);
        drivetrain.update();

        if (gamepad1.a) {
            if (drivetrain.maintainHeading) {
                drivetrain.setMaintainHeading(false);
            } else {
                drivetrain.setMaintainHeading(true);
            }
        } else { drivetrain.setMaintainHeading(drivetrain.maintainHeading);}

        drivetrain.getTelemetry(telemetry);

        telemetry.update();
    }
}
