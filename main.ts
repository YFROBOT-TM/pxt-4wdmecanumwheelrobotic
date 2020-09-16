input.onButtonPressed(Button.B, function () {
    Robotic.setMotor(Robotic.Motors.MAll, Robotic.Dir.CW, 206)
    basic.pause(1000)
    Robotic.setMotor(Robotic.Motors.MAll, Robotic.Dir.CCW, 156)
    basic.pause(1000)
    Robotic.stopMotor(Robotic.Motors.MAll)
})
Robotic.connectIrReceiver(DigitalPin.P1, Robotic.IrProtocol.NEC)
