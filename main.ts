input.onButtonPressed(Button.A, function () {
    strip.showRainbow(1, 360)
    basic.pause(1000)
    strip.clear()
    strip.show()
})
input.onButtonPressed(Button.B, function () {
    Robotic.setMotor(Robotic.Motors.MAll, Robotic.Dir.CW, 206)
    basic.pause(1000)
    Robotic.setMotor(Robotic.Motors.MAll, Robotic.Dir.CCW, 156)
    basic.pause(1000)
    Robotic.stopMotor(Robotic.Motors.MAll)
})
function R () {
	
}
let strip: neopixel.Strip = null
Robotic.connectIrReceiver(DigitalPin.P1, Robotic.IrProtocol.NEC)
strip = neopixel.create(DigitalPin.P8, 6, NeoPixelMode.RGB)
music.playMelody("C5 B A G F E D C ", 500)
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
