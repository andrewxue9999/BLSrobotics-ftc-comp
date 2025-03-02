package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.hardware.PivotCommandBased;

public class PivotCommandHigh extends CommandBase {
    private PivotCommandBased pivotSubSystem;

    public PivotCommandHigh(PivotCommandBased pivot)
    {
        pivotSubSystem = pivot;

        addRequirements(pivotSubSystem);
    }

    @Override
    public void initialize()
    {
        pivotSubSystem.goToHigh();
    }

    public void excecute()
    {

    }
    public boolean isFinished() {
        return pivotSubSystem.reached();
    }
}
