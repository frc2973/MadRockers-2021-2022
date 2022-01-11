namespace Ports {
    // Robot DIO ports
    enum DIO
    {
        DIO_0 = 0,
        DIO_1 = 1,
        DIO_2 = 2,
        DIO_3 = 3,
        DIO_4 = 4,
        DIO_5 = 5,
        DIO_6 = 6,
        DIO_7 = 7,
        DIO_8 = 8,
        DIO_9 = 9,
    };

    // Robot PMW ports
    enum PWM
    {
        PWM_0 = 0,
        PWM_1 = 1,
        PWM_2 = 2,
        PWM_3 = 3,
        PWM_4 = 4,
        PWM_5 = 5,
        PWM_6 = 6,
        PWM_7 = 7,
        PWM_8 = 8,
        PWM_9 = 9,
    };

    // Robot analog ports
    enum ANALOG
    {
        ANALOG_0 = 0,
        ANALOG_1 = 1,
        ANALOG_2 = 2,
        ANALOG_3 = 3
    };

    // XBox controllers
    enum USB
    {
        USB_0 = 0,
        USB_1 = 1,
        USB_2 = 2,
        USB_3 = 3
    };

    // CAN

    enum CAN
    {
        CAN_0 = 0,
        CAN_1 = 1,
        CAN_2 = 2,
        CAN_3 = 3,
        CAN_4 = 4,
        CAN_5 = 5,
        CAN_6 = 6,
        CAN_7 = 7,
        CAN_8 = 8,
        CAN_9 = 9
    };

    const USB XBOX_DRIVER = USB::USB_0;

    const CAN LEFT_FRONT = CAN::CAN_1;
    const CAN LEFT_BACK = CAN::CAN_2;
    const CAN RIGHT_FRONT = CAN::CAN_3;
    const CAN RIGHT_BACK = CAN::CAN_4;
    const CAN SHOOTER = CAN::CAN_9;
    
    const PWM ELEVATOR = PWM::PWM_2;
    const PWM TRANSFER = PWM::PWM_3;
    const PWM INTAKE = PWM::PWM_4;

}