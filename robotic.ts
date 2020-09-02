/** 
 * @file pxt-robotic/robotic.ts
 * @brief YFROBOT's robotic with four wheel driver makecode library.
 * @n This is a MakeCode graphical programming education robot.
 * 
 * @copyright    YFROBOT[www.yfrobot.com],2020
 * @copyright    MIT Lesser General Public License
 * 
 * @author [email](yfrobot@qq.com)
 * @date  2020-07-22
*/


//% color="#7BD239" weight=10 icon="\uf1b0" block="Robotic4WD"
namespace Robotic {
    const PCA9685_ADDRESS = 0x40
    const MODE1 = 0x00
    const MODE2 = 0x01
    const SUBADR1 = 0x02
    const SUBADR2 = 0x03
    const SUBADR3 = 0x04
    const PRESCALE = 0xFE
    const LED0_ON_L = 0x06
    const LED0_ON_H = 0x07
    const LED0_OFF_L = 0x08
    const LED0_OFF_H = 0x09
    const ALL_LED_ON_L = 0xFA
    const ALL_LED_ON_H = 0xFB
    const ALL_LED_OFF_L = 0xFC
    const ALL_LED_OFF_H = 0xFD

    export enum Motors {
        M1 = 0x1,
        M2 = 0x2,
        M3 = 0x3,
        M4 = 0x4,
        MAll = 0x5
    }

    export enum Dir {
        //% blockId="CW" block="Forward"
        CW = 0x0,
        //% blockId="CCW" block="Backward"
        CCW = 0x1
    }

    let initialized = false  // Initialization flag

    function i2cwrite(addr: number, reg: number, value: number) {
        let buf = pins.createBuffer(2)
        buf[0] = reg
        buf[1] = value
        pins.i2cWriteBuffer(addr, buf)
    }

    function i2ccmd(addr: number, value: number) {
        let buf = pins.createBuffer(1)
        buf[0] = value
        pins.i2cWriteBuffer(addr, buf)
    }

    function i2cread(addr: number, reg: number) {
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE);
        let val = pins.i2cReadNumber(addr, NumberFormat.UInt8BE);
        return val;
    }

    function initPCA9685(): void {
        i2cwrite(PCA9685_ADDRESS, MODE1, 0x00)
        setFreq(50);
        for (let idx = 0; idx < 16; idx++) {
            setPwm(idx, 0, 0);
        }
        initialized = true
    }

    function setFreq(freq: number): void {
        // Constrain the frequency
        let prescaleval = 25000000;
        prescaleval /= 4096;
        prescaleval /= freq;
        prescaleval -= 1;
        let prescale = prescaleval; //Math.Floor(prescaleval + 0.5);
        let oldmode = i2cread(PCA9685_ADDRESS, MODE1);
        let newmode = (oldmode & 0x7F) | 0x10; // sleep
        i2cwrite(PCA9685_ADDRESS, MODE1, newmode); // go to sleep
        i2cwrite(PCA9685_ADDRESS, PRESCALE, prescale); // set the prescaler
        i2cwrite(PCA9685_ADDRESS, MODE1, oldmode);
        control.waitMicros(5000);
        i2cwrite(PCA9685_ADDRESS, MODE1, oldmode | 0xa1);
    }

    function setPwm(channel: number, on: number, off: number): void {
        if (channel < 0 || channel > 15)
            return;
        //serial.writeValue("ch", channel)
        //serial.writeValue("on", on)
        //serial.writeValue("off", off)

        let buf = pins.createBuffer(5);
        buf[0] = LED0_ON_L + 4 * channel;
        buf[1] = on & 0xff;
        buf[2] = (on >> 8) & 0xff;
        buf[3] = off & 0xff;
        buf[4] = (off >> 8) & 0xff;
        pins.i2cWriteBuffer(PCA9685_ADDRESS, buf);
    }

    

    /**
     * Set the direction and speed of Robot motor.
     * @param index motors index: M1/M2/M3/M4/MAll
     * @param direction direction to turn
     * @param speed speed of motors (0 to 255). eg: 120
     */
    //% blockId=robotic_set_motor block="Motor|%index|%direction|speed %speed"
    //% weight=85
    //% speed.min=0 speed.max=255
    //% index.fieldEditor="gridpicker" index.fieldOptions.columns=2
    //% direction.fieldEditor="gridpicker" direction.fieldOptions.columns=2
    export function setMotor(index: Motors, direction: Dir, speed: number): void {
        if (!initialized) {
            initPCA9685()
        }
        speed = speed * 16; // map 255 to 4096
        if (speed >= 4096) {
            speed = 4095
        }
        if (speed <= 0) {
            speed = 0
        }
        if (index > 4 || index <= 0)
            return
        if (index != Motors.MAll) {
            let mDirPin = (index - 1) * 2
            let mPwmPin = (index - 1) * 2 + 1
            if (direction == 0) {
                setPwm(mDirPin, 0, 4096)
                setPwm(mPwmPin, 0, speed)
            } else {
                setPwm(mDirPin, 4096, 0)
                setPwm(mPwmPin, 0, speed)
            }
        } else {
            if (direction == 0) {
                setPwm(0, 0, 4096)
                setPwm(1, 0, speed)
                setPwm(2, 0, 4096)
                setPwm(3, 0, speed)
                setPwm(4, 0, 4096)
                setPwm(5, 0, speed)
                setPwm(6, 0, 4096)
                setPwm(7, 0, speed)
            } else {
                setPwm(0, 4096, 0)
                setPwm(1, 0, speed)
                setPwm(2, 4096, 0)
                setPwm(3, 0, speed)
                setPwm(4, 4096, 0)
                setPwm(5, 0, speed)
                setPwm(6, 4096, 0)
                setPwm(7, 0, speed)
            }
        }
    }

    // Set the speed of Robot motor.
    // If the speed is greater than 0, the motor will rotate forward, and if the speed is less than 0, the motor will reverse.
    function setSingleMotor(dirPin: number, pwmPin: number, speed: number) {
        speed = speed * 16; // map 255 to 4096

        if (speed >= 4096) speed = 4095
        if (speed <= -4096) speed = -4095

        if (speed >= 0) {
            setPwm(dirPin, 0, 4096)
            setPwm(pwmPin, 0, speed)
        } else {
            setPwm(dirPin, 4096, 0)
            setPwm(pwmPin, 0, -speed)
        }
    }

    /**
     * Set the speed of four Robot motors at the same time. 
     * @param speedMotor1 speed of motor 1 (-255 to 255). eg: 120
     * @param speedMotor2 speed of motor 2 (-255 to 255). eg: 120
     * @param speedMotor3 speed of motor 3 (-255 to 255). eg: 120
     * @param speedMotor4 speed of motor 4 (-255 to 255). eg: 120
     */
    //% blockId=robotic_set_four_motor block="Motor|%speedMotor1|M2|%speedMotor2|M3|%speedMotor3|M4 %speedMotor4"
    //% weight=83
    //% speedMotor1.min=-255 speedMotor1.max=255
    //% speedMotor2.min=-255 speedMotor2.max=255
    //% speedMotor3.min=-255 speedMotor3.max=255
    //% speedMotor4.min=-255 speedMotor4.max=255
    //% inlineInputMode=inline
    export function setFourMotor(speedMotor1: number, speedMotor2: number, speedMotor3: number, speedMotor4: number): void {
        if (!initialized) {
            initPCA9685()
        }

        setSingleMotor(0, 1, speedMotor1);
        setSingleMotor(2, 3, speedMotor2);
        setSingleMotor(4, 5, speedMotor3);
        setSingleMotor(6, 7, speedMotor4);
    }



}