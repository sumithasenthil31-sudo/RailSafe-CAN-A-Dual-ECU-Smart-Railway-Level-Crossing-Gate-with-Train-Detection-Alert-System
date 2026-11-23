#include <LPC21xx.H>
#include "ECU2_HEADER.H"
#include "ECU2_CANDRIVER.H"
volatile unsigned int ms_count = 0;
unsigned int train_state = TRAIN_CLEAR;
unsigned int train_count = 0;
unsigned int distance = 0;
unsigned int gate_state = GATE_OPEN;
unsigned int signal_state = 0;
unsigned int buzzer_state = 0;
unsigned int communication_timeout = 1;
unsigned int last_train_status_update = 0;
unsigned int last_ecu1_alive = 0;
unsigned int alive_received_flag = 0;
unsigned int emergency_active = 0;
unsigned int three_second_timer_start = 0;
unsigned int three_second_start_time = 0;
unsigned int servo_position = 1000;
unsigned int servo_time = 0;
unsigned int red_led_state = 0;
unsigned int red_led_time = 0;
unsigned int ecu2_alive_counter = 0;
unsigned int gate_status_time = 0;
CAN2_MSG msg;
void timer0_init(void);
void gpio_init(void);
void pwm_init(void);
void servo_control(void);
void process_train_status(CAN2_MSG msg);
void signal_control(void);
void send_gate_status(void);
__irq void timer0_isr(void);
void timer0_init(void)
{
    T0TCR = 0x02;
    T0PR = 59;
    T0MR0 = 999;
    T0MCR = 0x03;
    VICIntSelect &= ~(1<<4);
    VICVectAddr0 = (unsigned long)timer0_isr;
    VICVectCntl0 = 0x20 | 4;
    VICIntEnable |= (1<<4);
    T0TCR = 0x01;
}
__irq void timer0_isr(void)
{
    ms_count++;
    T0IR = 1;
    VICVectAddr = 0;
}

void gpio_init(void)
{
    IODIR1 |= (1<<16) | (1<<17) | (1<<18);
    IOSET1 = (1<<16) | (1<<17);
    IOSET1 = (1<<18);
}
void pwm_init(void)
{
    PINSEL0 = (PINSEL0 & ~(3<<14)) | (2<<14);
    PWMTCR = 0x02;
    PWMPR = 59;
    PWMMR0 = 20000;
    PWMMR2 = 1000;
    PWMMCR = 0x02;
    PWMPCR = (1<<10);
    PWMLER = (1<<0) | (1<<2);
    PWMTCR = 0x09;
}
void servo_control(void)
{
    if(ms_count - servo_time >= 20)
    {
      servo_time = ms_count;
    if(gate_state == GATE_CLOSING)
        {
       if(servo_position < 1500)
            {
         servo_position += 10;

      if(servo_position > 1500)
           servo_position = 1500;
                PWMMR2 = servo_position;
                PWMLER = (1<<2);
            }

            if(servo_position >= 1500)
            {
                gate_state = GATE_CLOSED;
            }
        }
        else if(gate_state == GATE_OPENING)
        {
            if(servo_position > 1000)
            {
                servo_position -= 10;

                if(servo_position < 1000)
                    servo_position = 1000;
                PWMMR2 = servo_position;
                PWMLER = (1<<2);
            }
            if(servo_position <= 1000)
            {
                gate_state = GATE_OPEN;
            }
        }
    }
}
void process_train_status(CAN2_MSG msg)
{
    unsigned int received_alive;
    unsigned int received_state;
    if(msg.id == EMERGENCY_ID &&
       msg.dlc == 1 &&
       msg.rtr == 0 &&
       (msg.byteA & 0xFF) == 0xFF)
    {
        emergency_active = 1;

        if(gate_state == GATE_OPEN ||
           gate_state == GATE_OPENING)
        {
            gate_state = GATE_CLOSING;
        }

        return;
    }
    if(msg.id != TRAIN_STATUS_ID ||
       msg.dlc != 4 ||
       msg.rtr != 0)
    {
        return;
    }
    received_state = msg.byteA & 0xFF;

    if(received_state > TRAIN_CLEARED)
        return;
    received_alive = (msg.byteA >> 24) & 0xFF;
    if(alive_received_flag == 0)
    {
        last_ecu1_alive = received_alive;
        alive_received_flag = 1;
    }
    else
    {
        if(received_alive == last_ecu1_alive)
            return;
        last_ecu1_alive = received_alive;
    }
    last_train_status_update = ms_count;
    communication_timeout = 0;
    train_state = received_state;
    train_count = (msg.byteA >> 8) & 0xFF;
    distance = (msg.byteA >> 16) & 0xFF;
    if(train_state == TRAIN_APPROACHING)
    {
        if(three_second_timer_start == 0)
        {
            three_second_start_time = ms_count;
            three_second_timer_start = 1;
        }
    }
    else if(train_state == TRAIN_CROSSING)
    {
        three_second_timer_start = 0;

        if(gate_state == GATE_OPEN ||
           gate_state == GATE_OPENING)
        {
            gate_state = GATE_CLOSING;
        }
    }
    else if(train_state == TRAIN_CLEARED ||
            train_state == TRAIN_CLEAR)
    {
        three_second_timer_start = 0;
        if(train_count == 0 &&
           emergency_active == 0 &&
           (gate_state == GATE_CLOSED ||
            gate_state == GATE_CLOSING))
        {
            gate_state = GATE_OPENING;
        }
    }
}
void signal_control(void)
{
    if(communication_timeout == 1 ||
       emergency_active == 1)
    {
        IOCLR1 = (1<<16);
        IOSET1 = (1<<17);
        IOCLR1 = (1<<18);
        signal_state = 1;
        buzzer_state = 1;
    }
    else if(train_state == TRAIN_APPROACHING)
    {
        IOSET1 = (1<<17);
        if(ms_count - red_led_time >= 250)
        {
            red_led_time = ms_count;
            if(red_led_state == 0)
            {
                IOCLR1 = (1<<16);
                red_led_state = 1;
            }
            else
            {
                IOSET1 = (1<<16);
                red_led_state = 0;
            }
        }
        IOCLR1 = (1<<18);
        signal_state = 1;
        buzzer_state = 1;
    }
    else if(train_state == TRAIN_CROSSING)
    {
        IOCLR1 = (1<<16);
        IOSET1 = (1<<17);

        IOCLR1 = (1<<18);

        signal_state = 1;
        buzzer_state = 1;
    }
    else if(train_state == TRAIN_CLEARED ||
            train_state == TRAIN_CLEAR)
    {
        IOSET1 = (1<<16);

        if(gate_state == GATE_OPEN)
        {
            IOCLR1 = (1<<17);
        }
        else
        {
            IOSET1 = (1<<17);
        }

        IOSET1 = (1<<18);

        signal_state = 0;
        buzzer_state = 0;
    }
}
void send_gate_status(void)
{
    CAN2_MSG tx_msg;
    if(ms_count - gate_status_time >= 100)
    {
        tx_msg.id = GATE_STATUS_ID;
        tx_msg.dlc = 4;
        tx_msg.rtr = 0;
        tx_msg.byteA = gate_state |
                       (signal_state << 8) |
                       (buzzer_state << 16) |
                       (ecu2_alive_counter << 24);
        tx_msg.byteB = 0;
        can2_tx(tx_msg);
        gate_status_time = ms_count;
        ecu2_alive_counter++;
        if(ecu2_alive_counter > 255)
            ecu2_alive_counter = 0;
    }
}
int main(void)
{
    VPBDIV = 1;
    gpio_init();
    timer0_init();
    pwm_init();
    can2_init();
    while(1)
    {
        if(can2_rx(&msg))
        {
            process_train_status(msg);
        }
        if(ms_count - last_train_status_update >= 500)
        {
            communication_timeout = 1;
        }
        if(communication_timeout == 1 ||
           emergency_active == 1)
        {
            three_second_timer_start = 0;

            if(gate_state == GATE_OPEN ||
               gate_state == GATE_OPENING)
            {
                gate_state = GATE_CLOSING;
            }
        }
        if(communication_timeout == 0 &&
           emergency_active == 0 &&
           train_state == TRAIN_APPROACHING &&
           three_second_timer_start == 1 &&
           ms_count - three_second_start_time >= 3000)
        {
            if(gate_state == GATE_OPEN ||
               gate_state == GATE_OPENING)
            {
                gate_state = GATE_CLOSING;
            }
        }
        servo_control();
        signal_control();
        send_gate_status();
    }
}

