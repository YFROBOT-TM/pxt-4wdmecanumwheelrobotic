input.onButtonPressed(Button.A, function () {
    basic.showNumber(Robotic.Ultrasonic(Robotic.UltrasonicAddress.Addr13))
})
input.onButtonPressed(Button.B, function () {
    Robotic.setFourMotor(40, 40, 40, 40)
    basic.pause(1000)
    Robotic.setFourMotor(-40, -40, -40, -40)
    basic.pause(1000)
    Robotic.setFourMotor(0, 0, 0, 0)
})
Robotic.initMotorDirectionReverse(Robotic.SetMotorDir.CW, Robotic.SetMotorDir.CW, Robotic.SetMotorDir.CW, Robotic.SetMotorDir.CW)
music.playMelody("C5 B A G F E D C ", 500)
