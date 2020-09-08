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

    // motor pca9685 channel 
    const Motor1DirectionChannel = 0
    const Motor1PWMChannel = 1
    const Motor2DirectionChannel = 2
    const Motor2PWMChannel = 3
    const Motor3DirectionChannel = 4
    const Motor3PWMChannel = 5
    const Motor4DirectionChannel = 6
    const Motor4PWMChannel = 7

    // iic Ultrasonic SR09 
    // http://www.yfrobot.com/wiki/index.php?title=Ultrasonic
    const CMDREG = 0x02
    const CMDB4 = 0xB4  // 1~500cm , mm, Temperature compensation, max interval 87ms

    // IR 
    const MICROBIT_MAKERBIT_IR_NEC = 777
    const MICROBIT_MAKERBIT_IR_BUTTON_PRESSED_ID = 789
    const MICROBIT_MAKERBIT_IR_BUTTON_RELEASED_ID = 790
    const IR_REPEAT = 256
    const IR_INCOMPLETE = 257

    let initialized = false // Initialization flag
    let Motor1Dir = 1       // motor 1 direction
    let Motor2Dir = -1       // motor 2 direction
    let Motor3Dir = 1       // motor 3 direction
    let Motor4Dir = -1       // motor 4 direction
    let SR09Address = 0xE8  // sr09 iic address

    // IR
    let irState: IrState
    interface IrState {
        protocol: IrProtocol;
        command: number;
        hasNewCommand: boolean;
        bitsReceived: uint8;
        commandBits: uint8;
    }

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

    export enum UltrasonicAddress {
        XD0 = 0xD0, XD2 = 0xD2,
        XD4 = 0xD4, XD6 = 0xD6,
        XD8 = 0xD8, XDA = 0xDA,
        XDC = 0xDC, XDE = 0xDE,
        XE0 = 0xE0, XE2 = 0xE2,
        XE4 = 0xE4, XE6 = 0xE6,
        XE8 = 0xE8, XEA = 0xEA,
        XEC = 0xEC, XEE = 0xEE,
        XF8 = 0xF8, XFA = 0xFA,
        XFC = 0xFC, XFE = 0xFE
    }

    export enum IrProtocol {
        //% block="Keyestudio"
        Keyestudio = 0,
        //% block="NEC"
        NEC = 1,
    }

    export enum IrButtonAction {
        //% block="pressed"
        Pressed = 0,
        //% block="released"
        Released = 1,
    }

    export enum IrButton {
        //IR HANDLE
        //% block="up"
        UP = 0x11,
        //% block="down"
        DOWN = 0x91,
        //% block="left"
        LEFT = 0x81,
        //% block="right"
        RIGHT = 0xa1,
        //% block="m1"
        M1 = 0xe9,
        //% block="m2"
        M2 = 0x69,
        //% block="a"
        A = 0x21,
        //% block="b"
        B = 0x01,
        //% block="any"
        Any = -1,
        // MINI IR 
        //% block="power"
        Power = 0xa2,
        //% block="menu"
        MENU = 0xe2,
        //% block="test"
        TEST = 0x22,
        //% block="+"
        PLUS = 0x02,
        //% block="back"
        Back = 0xc2,
        //% block="<<"
        Back2 = 0xe0,
        //% block="play"
        Play = 0xa8,
        //% block=">>"
        F = 0x90,
        //% block="0"
        Number_0 = 0x68,
        //% block="-"
        Less = 0x98,
        //% block="c"
        C = 0xb0,
        //% block="1"
        Number_1 = 0x30,
        //% block="2"
        Number_2 = 0x18,
        //% block="3"
        Number_3 = 0x7a,
        //% block="4"
        Number_4 = 0x10,
        //% block="5"
        Number_5 = 0x38,
        //% block="6"
        Number_6 = 0x5a,
        //% block="7"
        Number_7 = 0x42,
        //% block="8"
        Number_8 = 0x4a,
        //% block="9"
        Number_9 = 0x52,
    }



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
                setPwm(Motor1DirectionChannel, 4096, 0)
            else
                setPwm(Motor1DirectionChannel, 0, 4096)

            if (direction * Motor2Dir == 1)
                setPwm(Motor2DirectionChannel, 4096, 0)
            else
                setPwm(Motor2DirectionChannel, 0, 4096)

            if (direction * Motor3Dir == 1)
                setPwm(Motor3DirectionChannel, 4096, 0)
            else
                setPwm(Motor3DirectionChannel, 0, 4096)

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
            if (mDirPin == Motor1DirectionChannel) {
                dir = Motor1Dir
            } else if (mDirPin == Motor2DirectionChannel) {
                dir = Motor2Dir
            } else if (mDirPin == Motor3DirectionChannel) {
                dir = Motor3Dir
            } else if (mDirPin == Motor4DirectionChannel) {
                dir = Motor4Dir
            }
            if (direction * dir == 1) {
                setPwm(mDirPin, 4096, 0)
            } else {
                setPwm(mDirPin, 0, 4096)
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
            setPwm(dirPin, 4096, 0)
            setPwm(pwmPin, 0, speed)
        } else {
            setPwm(dirPin, 0, 4096)
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
        setSingleMotor(Motor2DirectionChannel, Motor2PWMChannel, speedMotor2);
        setSingleMotor(Motor3DirectionChannel, Motor3PWMChannel, speedMotor3);
        setSingleMotor(Motor4DirectionChannel, Motor4PWMChannel, speedMotor4);
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
        Motor1Dir = m1Dir;          // motor 1 direction
        Motor2Dir = m2Dir * -1;     // motor 2 direction
        Motor3Dir = m3Dir;          // motor 3 direction
        Motor4Dir = m4Dir * -1;     // motor 4 direction
    }

    /**
     * Stop Robot motors. 
     * @param index motors index: M1/M2/M3/M4/MAll
     */
    //% blockId=robotic_stop_motor block="Stop Motor |%index"
    //% weight=81
    //% index.fieldEditor="gridpicker" index.fieldOptions.columns=2
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

    /**
     * IIC Ultrasonic SR09.
     * @param address Ultrasonic iic address; eg: Robotic.UltrasonicAddress.XE8
     */
    //% blockId=robotic_ultrasonic block="Ultrasonic |%address"
    //% weight=79
    //% address.fieldEditor="gridpicker" address.fieldOptions.columns=4
    //% inlineInputMode=inline
    export function Ultrasonic(address: UltrasonicAddress): number {
        address = address >> 1  // 7 bit address
        i2cwrite(address, CMDREG, CMDB4)
        pause(87)   // wait the data,max 87ms
        i2ccmd(address, CMDREG)
        let data_mm = 0
        data_mm = i2cread(address, CMDREG) << 8
        data_mm |= i2cread(address, CMDREG + 1)
        return data_mm
    }


    /***************** IR *******************/

    function pushBit(bit: number): number {
        irState.bitsReceived += 1;
        if (irState.bitsReceived <= 8) {
            // ignore all address bits
            if (irState.protocol === IrProtocol.Keyestudio && bit === 1) {
                // recover from missing message bits at the beginning
                // Keyestudio address is 0 and thus missing bits can be easily detected
                // by checking for the first inverse address bit (which is a 1)
                irState.bitsReceived = 9;
            }
            return IR_INCOMPLETE;
        }
        if (irState.bitsReceived <= 16) {
            // ignore all inverse address bits
            return IR_INCOMPLETE;
        } else if (irState.bitsReceived < 24) {
            irState.commandBits = (irState.commandBits << 1) + bit;
            return IR_INCOMPLETE;
        } else if (irState.bitsReceived === 24) {
            irState.commandBits = (irState.commandBits << 1) + bit;
            return irState.commandBits & 0xff;
        } else {
            // ignore all inverse command bits
            return IR_INCOMPLETE;
        }
    }

    function detectCommand(markAndSpace: number): number {
        if (markAndSpace < 1600) {
            // low bit
            return pushBit(0);
        } else if (markAndSpace < 2700) {
            // high bit
            return pushBit(1);
        }

        irState.bitsReceived = 0;

        if (markAndSpace < 12500) {
            // Repeat detected
            return IR_REPEAT;
        } else if (markAndSpace < 14500) {
            // Start detected
            return IR_INCOMPLETE;
        } else {
            return IR_INCOMPLETE;
        }
    }

    function enableIrMarkSpaceDetection(pin: DigitalPin) {
        pins.setPull(pin, PinPullMode.PullNone);

        let mark = 0;
        let space = 0;

        pins.onPulsed(pin, PulseValue.Low, () => {
            // HIGH, see https://github.com/microsoft/pxt-microbit/issues/1416
            mark = pins.pulseDuration();
        });

        pins.onPulsed(pin, PulseValue.High, () => {
            // LOW
            space = pins.pulseDuration();
            const command = detectCommand(mark + space);
            if (command !== IR_INCOMPLETE) {
                control.raiseEvent(MICROBIT_MAKERBIT_IR_NEC, command);
            }
        });
    }

    /**
     * Connects to the IR receiver module at the specified pin and configures the IR protocol.
     * @param pin IR receiver pin. eg: DigitalPin.P1
     * @param protocol IR protocol. eg: Robotic.IrProtocol.NEC
     */
    //% subcategory="IR Receiver"
    //% blockId="makerbit_infrared_connect_receiver"
    //% block="connect IR receiver at pin %pin and decode %protocol"
    //% pin.fieldEditor="gridpicker"
    //% pin.fieldOptions.columns=4
    //% pin.fieldOptions.tooltips="false"
    //% weight=15
    export function connectIrReceiver(pin: DigitalPin, protocol: IrProtocol): void {
        if (irState) {
            return;
        }

        irState = {
            protocol: protocol,
            bitsReceived: 0,
            commandBits: 0,
            command: IrButton.Any,
            hasNewCommand: false,
        };

        enableIrMarkSpaceDetection(pin);

        let activeCommand = IR_INCOMPLETE;
        let repeatTimeout = 0;
        const REPEAT_TIMEOUT_MS = 120;

        control.onEvent(
            MICROBIT_MAKERBIT_IR_NEC,
            EventBusValue.MICROBIT_EVT_ANY,
            () => {
                const necValue = control.eventValue();

                // Refresh repeat timer
                if (necValue <= 255 || necValue === IR_REPEAT) {
                    repeatTimeout = input.runningTime() + REPEAT_TIMEOUT_MS;
                }

                // Process a new command
                if (necValue <= 255 && necValue !== activeCommand) {
                    if (activeCommand >= 0) {
                        control.raiseEvent(
                            MICROBIT_MAKERBIT_IR_BUTTON_RELEASED_ID,
                            activeCommand
                        );
                    }

                    irState.hasNewCommand = true;
                    irState.command = necValue;
                    activeCommand = necValue;
                    control.raiseEvent(MICROBIT_MAKERBIT_IR_BUTTON_PRESSED_ID, necValue);
                }
            }
        );

        control.inBackground(() => {
            while (true) {
                if (activeCommand === IR_INCOMPLETE) {
                    // sleep to save CPU cylces
                    basic.pause(2 * REPEAT_TIMEOUT_MS);
                } else {
                    const now = input.runningTime();
                    if (now > repeatTimeout) {
                        // repeat timed out
                        control.raiseEvent(
                            MICROBIT_MAKERBIT_IR_BUTTON_RELEASED_ID,
                            activeCommand
                        );
                        activeCommand = IR_INCOMPLETE;
                    } else {
                        basic.pause(REPEAT_TIMEOUT_MS);
                    }
                }
            }
        });
    }

    /**
     * Do something when a specific button is pressed or released on the remote control.
     * @param button the button to be checked
     * @param action the trigger action
     * @param handler body code to run when event is raised
     */
    //% subcategory="IR Receiver"
    //% blockId=makerbit_infrared_on_ir_button
    //% block="on IR button | %button | %action"
    //% button.fieldEditor="gridpicker"
    //% button.fieldOptions.columns=3
    //% button.fieldOptions.tooltips="false"
    //% weight=13
    export function onIrButton(button: IrButton, action: IrButtonAction, handler: () => void) {
        control.onEvent(
            action === IrButtonAction.Pressed
                ? MICROBIT_MAKERBIT_IR_BUTTON_PRESSED_ID
                : MICROBIT_MAKERBIT_IR_BUTTON_RELEASED_ID,
            button === IrButton.Any ? EventBusValue.MICROBIT_EVT_ANY : button,
            () => {
                irState.command = control.eventValue();
                handler();
            }
        );
    }

    /**
     * Returns the code of the IR button that was pressed last. Returns -1 (IrButton.Any) if no button has been pressed yet.
     */
    //% subcategory="IR Receiver"
    //% blockId=makerbit_infrared_ir_button_pressed
    //% block="IR button"
    //% weight=10
    export function irButton(): number {
        if (!irState) {
            return IrButton.Any;
        }
        return irState.command;
    }

    /**
     * Returns true if any button was pressed since the last call of this function. False otherwise.
     */
    //% subcategory="IR Receiver"
    //% blockId=makerbit_infrared_was_any_button_pressed
    //% block="any IR button was pressed"
    //% weight=7
    export function wasAnyIrButtonPressed(): boolean {
        if (!irState) {
            return false;
        }
        if (irState.hasNewCommand) {
            irState.hasNewCommand = false;
            return true;
        } else {
            return false;
        }
    }

    /**
     * Returns the command code of a specific IR button.
     * @param button the button
     */
    //% subcategory="IR Receiver"
    //% blockId=makerbit_infrared_button_code
    //% button.fieldEditor="gridpicker"
    //% button.fieldOptions.columns=3
    //% button.fieldOptions.tooltips="false"
    //% block="IR button code %button"
    //% weight=5
    export function irButtonCode(button: IrButton): number {
        return button as number;
    }


    /***************** PS2 *******************/


}