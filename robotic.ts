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

    const Motor1DirectionChannel = 0;
    const Motor1PWMChannel = 1;
    const Motor2DirectionChannel = 2;
    const Motor2PWMChannel = 3;
    const Motor3DirectionChannel = 4;
    const Motor3PWMChannel = 5;
    const Motor4DirectionChannel = 6;
    const Motor4PWMChannel = 7;

    export enum Motors {
        M1 = 0x1,
        M2 = 0x2,
        M3 = 0x3,
        M4 = 0x4,
        MAll = 0x5
    }

    export enum Dir {
        //% blockId="CW" block="Forward"
        CW = 1,
        //% blockId="CCW" block="Backward"
        CCW = -1
    }

    export enum SetMotorDir {
        //% block="Positive"
        CW = 1,
        //% block="Reverse"
        CCW = -1
    }


    let initialized = false  // Initialization flag
    let Motor1Dir = 1;      // motor 1 direction
    let Motor2Dir = 1;      // motor 2 direction
    let Motor3Dir = 1;      // motor 3 direction
    let Motor4Dir = 1;      // motor 4 direction


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
        if (index > 5 || index <= 0)
            return

        if (index == Motors.MAll) {
            if (direction * Motor1Dir == 1)
                setPwm(Motor1DirectionChannel, 0, 4096)
            else
                setPwm(Motor1DirectionChannel, 4096, 0)
            if (direction * Motor2Dir == 1)
                setPwm(Motor2DirectionChannel, 4096, 0)
            else
                setPwm(Motor2DirectionChannel, 0, 4096)
            if (direction * Motor3Dir == 1)
                setPwm(Motor3DirectionChannel, 0, 4096)
            else
                setPwm(Motor3DirectionChannel, 4096, 0)
            if (direction * Motor4Dir == 1)
                setPwm(Motor4DirectionChannel, 4096, 0)
            else
                setPwm(Motor4DirectionChannel, 0, 4096)

            setPwm(Motor1PWMChannel, 0, speed)
            setPwm(Motor2PWMChannel, 0, speed)
            setPwm(Motor3PWMChannel, 0, speed)
            setPwm(Motor4PWMChannel, 0, speed)
        } else {
            let dir
            let mDirPin = (index - 1) * 2
            let mPwmPin = (index - 1) * 2 + 1
            if (mDirPin == 0) {
                dir = Motor1Dir
            } else if (mDirPin == 2) {
                dir = Motor2Dir*-1
            } else if (mDirPin == 4) {
                dir = Motor3Dir
            } else if (mDirPin == 6) {
                dir = Motor4Dir*-1
            }
            if (direction * dir == 1) {
                setPwm(mDirPin, 0, 4096)
            } else {
                setPwm(mDirPin, 4096, 0)
            }
            setPwm(mPwmPin, 0, speed)
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
    //% blockId=robotic_set_four_motor block="Motor M1|%speedMotor1|M2|%speedMotor2|M3|%speedMotor3|M4 %speedMotor4"
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
        // motor direction
        speedMotor1 = speedMotor1 * Motor1Dir
        speedMotor2 = speedMotor2 * Motor2Dir
        speedMotor3 = speedMotor3 * Motor3Dir
        speedMotor4 = speedMotor4 * Motor4Dir

        setSingleMotor(Motor1DirectionChannel, Motor1PWMChannel, speedMotor1);
        setSingleMotor(Motor2DirectionChannel, Motor2PWMChannel, -speedMotor2);
        setSingleMotor(Motor3DirectionChannel, Motor3PWMChannel, speedMotor3);
        setSingleMotor(Motor4DirectionChannel, Motor4PWMChannel, -speedMotor4);
    }

    /**
     * Set the direction of four Robot motors at the same time. 
     * @param m1Dir direction of motor1.
     * @param m2Dir direction of motor2.
     * @param m3Dir direction of motor3.
     * @param m4Dir direction of motor4.
     */
    //% blockId=robotic_init_four_motor_direction block="Motor M1|%speedMotor1|M2|%speedMotor2|M3|%speedMotor3|M4 %speedMotor4"
    //% weight=100
    //% inlineInputMode=inline
    //% advanced=true
    export function initMotorDirectionReverse(m1Dir: SetMotorDir, m2Dir: SetMotorDir, m3Dir: SetMotorDir, m4Dir: SetMotorDir) {
        Motor1Dir = m1Dir;      // motor 1 direction
        Motor2Dir = m2Dir;      // motor 2 direction
        Motor3Dir = m3Dir;      // motor 3 direction
        Motor4Dir = m4Dir;      // motor 4 direction
    }

    /**
     * Stop Robot motors. 
     * @param index motors index: M1/M2/M3/M4/MAll
     */
    //% blockId=robotic_stop_motor block="Stop Motor |%index"
    //% weight=81
    //% inlineInputMode=inline
    export function stopMotor(index: Motors): void {
        if (!initialized) {
            initPCA9685()
        }
        if (index > 5 || index <= 0)
            return
        if (index == Motors.MAll) {
            setPwm(Motor1PWMChannel, 0, 0)
            setPwm(Motor2PWMChannel, 0, 0)
            setPwm(Motor3PWMChannel, 0, 0)
            setPwm(Motor4PWMChannel, 0, 0)
        } else {
            let mPwmPin = (index - 1) * 2 + 1
            setPwm(mPwmPin, 0, 0)
        }
    }



}