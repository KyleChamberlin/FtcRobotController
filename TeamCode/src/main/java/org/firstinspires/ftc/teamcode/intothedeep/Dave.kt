package org.firstinspires.ftc.teamcode.intothedeep

import androidx.core.math.MathUtils.clamp
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION
import com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS
import org.firstinspires.ftc.teamcode.helpers.PowerVector
import org.firstinspires.ftc.teamcode.helpers.asPoseVelocity
import org.firstinspires.ftc.teamcode.helpers.getMotor
import org.firstinspires.ftc.teamcode.helpers.getServo
import org.firstinspires.ftc.teamcode.helpers.imu
import org.firstinspires.ftc.teamcode.helpers.mecanumDrive
import org.firstinspires.ftc.teamcode.intothedeep.PincerPosition.CLOSED
import org.firstinspires.ftc.teamcode.intothedeep.PincerPosition.OPEN
import org.firstinspires.ftc.teamcode.robot.Robot
import java.lang.Thread.sleep

enum class PincerPosition {
    OPEN,
    CLOSED
}

@Config
class Dave(
    hardwareMap: HardwareMap,
    val telemetry: Telemetry,
    startingPose: Pose2d = Pose2d(Vector2d(0.0, 0.0), 0.0),
) : Robot {
    override val drivetrain by lazy { hardwareMap.mecanumDrive() }
    override var position = startingPose
    override var velocity = PowerVector(0.0, 0.0, 0.0)
        set(value) {
            field = value
            drivetrain.powerVector = value
        }
    override val positionHistory = mutableListOf(position.copy())
    override val velocityHistory = mutableListOf(velocity.asPoseVelocity())
    val arm by lazy { hardwareMap.getMotor("arm_motor") }
    val extension by lazy { hardwareMap.getMotor("extension_motor") }
    val wrist by lazy { hardwareMap.getServo("wrist") }
    val leftPincer by lazy { hardwareMap.getServo("pincer_left") }
    val rightPincer by lazy { hardwareMap.getServo("pincer_right") }
    var armPosition = 0
        set(value) {
            var target = clamp(value, minimumArmPosition, maximumArmPosition)
            arm.targetPosition = target
            arm.velocity = armVelocity
            arm.mode = RUN_TO_POSITION
            field = target
        }
    var armExtension = 0
        set(value) {
            var target = clamp(
                value,
                minimumArmExtension,
                clamp(
                    maximumArmExtension - ((armPosition / minimumArmPosition) * 3000).toInt(),
                    minimumArmExtension,
                    maximumArmExtension
                )
            )
            extension.targetPosition = target
            extension.velocity = extensionVelocity
            extension.mode = RUN_TO_POSITION
            field = target
        }
    var wristPosition = 0.0
        set(value) {
            var target = clamp(value, 0.0, 1.0)
            wrist.position = target
            field = target
        }

    var pincerPosition: PincerPosition = CLOSED
        set(value) {
            when (value) {
                OPEN -> {
                    rightPincer.position = rightPincerOpenPosition
                    leftPincer.position = leftPincerOpenPosition
                }

                CLOSED -> {
                    rightPincer.position = rightPincerClosePosition
                    leftPincer.position = leftPincerClosePosition
                }
            }
            field = value
        }

    companion object {
        @JvmField
        var upperChamberExtensionBegin = 6000

        @JvmField
        var upperChamberExtensionEnd = 5000

        @JvmField
        var upperChamberArmPosition = 150

        @JvmField
        var upperBasketArmPosition = -50

        @JvmField
        var upperBasketArmExtension = 6500

        @JvmField
        var upperBasketWristPosition = 0.2

        @JvmField
        var upperChamberWristBegin = 0.7

        @JvmField
        var armGearRatio = 5

        @JvmField
        var armPositionModifier = 3

        @JvmField
        var maximumArmPosition = 0

        @JvmField
        var minimumArmPosition = -2500

        @JvmField
        var maximumArmExtension = 9000

        @JvmField
        var minimumArmExtension = 0

        @JvmField
        var extensionRate = 100

        @JvmField
        var armVelocity = 500.0

        @JvmField
        var extensionVelocity = 2000.0

        @JvmField
        var wristRate = 0.01

        @JvmField
        var rightPincerClosePosition = 1.0

        @JvmField
        var leftPincerClosePosition = 1.0

        @JvmField
        var rightPincerOpenPosition = 0.0

        @JvmField
        var leftPincerOpenPosition = 0.0

        @JvmField
        var autoLinearSpeed = 0.4

        @JvmField
        var autoRotationalSpeed = 0.4
    }

    fun autoStrafeFor(millis: Int) {
        drivetrain.powerVector = PowerVector(0.0, autoLinearSpeed, 0.0)
        sleep(millis.toLong())
        drivetrain.powerVector = PowerVector(0.0, 0.0, 0.0)
    }

    fun autoForwardFor(millis: Int) {
        drivetrain.powerVector = PowerVector(autoLinearSpeed, 0.0, 0.0)
        sleep(millis.toLong())
        drivetrain.powerVector = PowerVector(0.0, 0.0, 0.0)
    }
    fun autoBackwardFor(millis: Int) {
        drivetrain.powerVector = PowerVector(-autoLinearSpeed, 0.0, 0.0)
        sleep(millis.toLong())
        drivetrain.powerVector = PowerVector(0.0, 0.0, 0.0)
    }

    fun armTo(position: Int) {
        armPosition = position
        while (arm.currentPosition > armPosition + 5) {
            telemetry.motorPosition("arm", arm)
            sleep(10)
        }
    }

    fun extensionTo(position: Int) {
        armExtension = position
        while (extension.currentPosition < armExtension - 5) {
            telemetry.motorPosition("extension", extension)
            sleep(10)
        }
    }

    fun rotateToHeading(heading: Double) {
        drivetrain.powerVector = PowerVector(0.0, 0.0, 0.0)
    }

    override fun update() {
        telemetry.motorPosition("Arm", arm)
        telemetry.motorPosition("Extension", extension)
        telemetry.addData("velocity", velocity)
        positionHistory.add(position)
        velocityHistory.add(velocity.asPoseVelocity())

    }

    fun specimenToUpperChamber() {
        armPosition = upperChamberArmPosition
        armExtension = upperChamberExtensionBegin
        wristPosition = upperChamberWristBegin
        pincerPosition = CLOSED
    }

    fun hangSpecimenUpperChamber() {
        armExtension = upperChamberExtensionEnd
        // pincerPosition = OPEN
    }

    fun sampleToUpperBasket() {
        armPosition = upperBasketArmPosition
        armExtension = upperBasketArmExtension
        wristPosition = upperBasketWristPosition
        pincerPosition = CLOSED
    }

    fun dropSample() {
        pincerPosition = OPEN
    }

    fun armUp() {
        armPosition += armGearRatio * armPositionModifier
    }

    fun armDown() {
        armPosition -= armGearRatio * armPositionModifier
    }

    fun extend() {
        armExtension += extensionRate
    }

    fun retract() {
        armExtension -= extensionRate
    }

    fun wristUp() {
        wristPosition += wristRate
    }

    fun wristDown() {
        wristPosition -= wristRate
    }

    fun closePincer() {
        pincerPosition = CLOSED
    }

    fun openPincer() {
        pincerPosition = OPEN
    }

    override fun initialize() {
        arm.zeroPowerBehavior = BRAKE
        arm.mode = STOP_AND_RESET_ENCODER
        arm.targetPosition = 0
        arm.direction = REVERSE
        arm.velocity = armVelocity

        extension.zeroPowerBehavior = BRAKE
        extension.mode = STOP_AND_RESET_ENCODER
        extension.targetPosition = 0
        extension.direction = FORWARD
        extension.velocity = extensionVelocity

        arm.mode = RUN_TO_POSITION
        extension.mode = RUN_TO_POSITION

        wrist.direction = Servo.Direction.FORWARD
        rightPincer.direction = Servo.Direction.FORWARD
        leftPincer.direction = Servo.Direction.REVERSE
    }
}

fun Telemetry.motorPosition(name: String, motor: DcMotorEx) {
    this.addData("$name target", motor.targetPosition)
    this.addData("$name Position", motor.currentPosition)
    this.addData("$name velocity", motor.velocity)
    this.addData("$name power", motor.power)
}