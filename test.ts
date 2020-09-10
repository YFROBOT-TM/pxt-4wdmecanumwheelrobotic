input.onButtonPressed(Button.B, function () {
    Robotic.setMotor(Robotic.Motors.MAll, Robotic.Dir.CW, 206)
    basic.pause(1000)
    Robotic.setMotor(Robotic.Motors.MAll, Robotic.Dir.CCW, 156)
    basic.pause(1000)
    Robotic.stopMotor(Robotic.Motors.MAll)
})
Robotic.connectIrReceiver(DigitalPin.P1, Robotic.IrProtocol.NEC)
basic.forever(function () {
    if (Robotic.irButton() == Robotic.irButtonCode(Robotic.IrButton.Number_2)) {
        Robotic.setMotor(Robotic.Motors.MAll, Robotic.Dir.CW, 200)
    } else if (Robotic.irButton() == Robotic.irButtonCode(Robotic.IrButton.Number_4)) {
        Robotic.setFourMotor(-200, 200, 200, -200)
    } else if (Robotic.irButton() == Robotic.irButtonCode(Robotic.IrButton.Number_6)) {
        Robotic.setFourMotor(200, -200, -200, 200)
    } else if (Robotic.irButton() == Robotic.irButtonCode(Robotic.IrButton.Number_8)) {
        Robotic.setMotor(Robotic.Motors.MAll, Robotic.Dir.CCW, 200)
    } else if (Robotic.irButton() == Robotic.irButtonCode(Robotic.IrButton.Number_5)) {
        Robotic.stopMotor(Robotic.Motors.MAll)
    } else {
    	
    }
})
