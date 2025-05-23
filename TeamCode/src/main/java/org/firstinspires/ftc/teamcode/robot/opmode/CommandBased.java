package org.firstinspires.ftc.teamcode.robot.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.hardware.Pivot;
import org.firstinspires.ftc.teamcode.robot.hardware.PivotGamepadCommand;
import org.firstinspires.ftc.teamcode.robot.hardware.PivotPIDCommand;
import org.firstinspires.ftc.teamcode.robot.hardware.PivotSubsystem;

@Disabled
@TeleOp(name="diddle you")
public class CommandBased extends CommandOpMode {

    GamepadEx driver;
    Button test;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        PivotSubsystem pivot = new PivotSubsystem(hardwareMap, "pivot", "poop", telemetry);

        driver = new GamepadEx(gamepad1);

        test = new GamepadButton(driver, GamepadKeys.Button.A);
        test.whenHeld(new PivotGamepadCommand(pivot, 0.2)); //A button
        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new PivotGamepadCommand(pivot, -0.2));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new PivotPIDCommand(pivot, 0.8));

        

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
    //TODO: figure out how to actually use the command scheduler
}
