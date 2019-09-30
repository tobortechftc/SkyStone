package org.firstinspires.ftc.teamcode.opmodes.sigmaBot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Robot;
import org.firstinspires.ftc.teamcode.hardware.Sigma.ToboTest;
import org.firstinspires.ftc.teamcode.support.diagnostics.DiagnosticsTeleOp;

/**
 * Diagnostic TeleOp for Ruckus
 * @see DiagnosticsTeleOp
 */
@TeleOp(name="Test::Diagnostics", group="Test")
public class TestDiagnostics extends DiagnosticsTeleOp {

    // override log level, if desired
    // static { LOG_LEVEL = Log.VERBOSE; }

    @Override
    public Robot createRobot() {
        return new ToboTest().configureLogging("ToboTest", LOG_LEVEL);
    }
}
