package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode.AUTO
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.helpers.PowerVector
import org.firstinspires.ftc.teamcode.helpers.bulkCachingMode
import org.firstinspires.ftc.teamcode.intothedeep.Dave

@TeleOp(group = "Into The Deep", name = "Dave")
class DaveTeleOP : OpMode() {
    val dave by lazy { Dave(hardwareMap, telemetry) }

    override fun init() {
        hardwareMap.bulkCachingMode = AUTO
        var dashboard = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)

        dave.initialize()
    }

    override fun loop() {
        driveControl(gamepad1)

        armControl(gamepad2)

        dave.update()
    }

    fun driveControl(gamepad: Gamepad) {
        dave.velocity = gamepad.povVelocity()
    }

    fun armControl(gamepad: Gamepad) {
        if (gamepad.right_stick_y < -0.1) dave.armUp()
        if (gamepad.right_stick_y > 0.1) dave.armDown()

        if (gamepad.left_stick_y < -0.1) dave.extend()
        if (gamepad.left_stick_y > 0.1) dave.retract()

        if (gamepad.right_trigger > 0) dave.closePincer()
        if (gamepad.left_trigger > 0) dave.openPincer()

        if (gamepad.left_stick_x < -0.1) dave.wristDown()
        if (gamepad.left_stick_x > 0.1) dave.wristUp()
    }
}

fun Gamepad.povVelocity() = PowerVector(
    -this.left_stick_y.toDouble(),
    this.left_stick_x.toDouble(),
    this.right_stick_x.toDouble()
)