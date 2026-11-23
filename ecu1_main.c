#include <LPC21xx.H>
#include "header.h"
#include "can_driver.h"
#define LCD_D (0xF << 16)
#define RS (1 << 5)
#define E (1 << 6)
#define TRAIN_CLEAR 0
#define TRAIN_APPROACHING 1
#define TRAIN_ON_CROSSING 2
#define TRAIN_CLEARED 3
#define GATE_OPEN 0
#define GATE_CLOSING 1
#define GATE_CLOSED 2
#define GATE_OPENING 3
void Timer1_ISR(void) __irq;
void delay_milliseconds(unsigned int);
void LCD_SEND_NIBBLE(unsigned char);
void LCD_INIT(void);
void LCD_COMMAND(unsigned char);
void LCD_DATA(unsigned char);
void LCD_STRING(char *);
void timer1_init(void);
void IR_CHECK(void);
void safety_check(void);
int send_train_status(void);
void receive_gate_status(void);
void check_communication_timeout(void);
void startup_display(void);
void display_lcd(void);
volatile unsigned int ms_count = 0;
unsigned int train_count = 0;
unsigned int train_state = TRAIN_CLEAR;
unsigned int ir1_flag = 0;
unsigned int ir2_flag = 0;
unsigned int ir1_low_start = 0;
unsigned int ir1_high_start = 0;
unsigned int ir2_low_start = 0;
unsigned int ir2_high_start = 0;
unsigned int ir1_low_candidate = 0;
unsigned int ir1_high_candidate = 0;
unsigned int ir2_low_candidate = 0;
unsigned int ir2_high_candidate = 0;
unsigned int delay_2sec_ecu1 = 0;
unsigned int safety_wait_flag = 0;
unsigned int can_tx_time = 0;
unsigned char alive_counter = 0;
unsigned int gate_status = GATE_OPEN;
unsigned int signal_status = 0;
unsigned int buzzer_status = 0;
unsigned int last_gate_rx_time = 0;
unsigned int communication_timeout = 0;
CAN2_MSG tx_msg, rx_msg;
unsigned int old_train_state = 255;
unsigned int old_train_count = 255;
unsigned int old_gate_status = 255;
unsigned int old_communication_timeout = 255;
unsigned int startup_time = 0;
void delay_milliseconds(unsigned int milliseconds)
{
    T0TCR = 0x02;
    T0PR = 60000 - 1;
    T0TC = 0;
    T0TCR = 0x01;
    while(T0TC < milliseconds);

    T0TCR = 0x00;
}
void LCD_SEND_NIBBLE(unsigned char nibble)
{
    IOCLR0 = LCD_D;
    IOSET0 = (nibble & 0x0F) << 16;
    IOCLR0 = RS;
    IOSET0 = E;
    delay_milliseconds(2);
    IOCLR0 = E;
}
void LCD_COMMAND(unsigned char c)
{
    IOCLR0 = LCD_D;
    IOSET0 = (c & 0xF0) << 12;
    IOCLR0 = RS;
    IOSET0 = E;
    delay_milliseconds(2);
    IOCLR0 = E;
    IOCLR0 = LCD_D;
    IOSET0 = (c & 0x0F) << 16;
    IOSET0 = E;
    delay_milliseconds(2);
    IOCLR0 = E;
}
void LCD_DATA(unsigned char d)
{
    IOCLR0 = LCD_D;
    IOSET0 = (d & 0xF0) << 12;
    IOSET0 = RS;
    IOSET0 = E;
    delay_milliseconds(2);
    IOCLR0 = E;
    IOCLR0 = LCD_D;
    IOSET0 = (d & 0x0F) << 16;
    IOSET0 = E;
    delay_milliseconds(2);
    IOCLR0 = E;
}
void LCD_STRING(char *str)
{
    while(*str != '\0')
    {
        LCD_DATA(*str);
        str++;
    }
}
void LCD_INIT(void)
{
    IODIR0 |= LCD_D | RS | E;
    delay_milliseconds(20);
    LCD_SEND_NIBBLE(0x03);
    delay_milliseconds(5);
    LCD_SEND_NIBBLE(0x03);
    delay_milliseconds(2);
    LCD_SEND_NIBBLE(0x03);
    delay_milliseconds(2);
    LCD_SEND_NIBBLE(0x02);
    delay_milliseconds(2);
    LCD_COMMAND(0x28);
    LCD_COMMAND(0x0C);
    LCD_COMMAND(0x06);
    LCD_COMMAND(0x01);
    LCD_COMMAND(0x80);
}

void startup_display(void)
{
    LCD_COMMAND(0x80);
    LCD_STRING("RAILSAFE CAN ");
    LCD_COMMAND(0xC0);
    LCD_STRING("SYSTEM STARTING ");
    startup_time = ms_count;
}
void timer1_init(void)
{
    T1TCR = 0x02;
    T1PR = 59;
    T1MR0 = 999;
    T1MCR = 3;
    VICIntSelect &= ~(1<<5);
    VICVectAddr0 = (unsigned long)Timer1_ISR;
    VICVectCntl0 = 0x25;
    VICIntEnable |= (1<<5);
    T1TCR = 0x01;
}
void Timer1_ISR(void) __irq
{
    T1IR = 1;
    ms_count++;
    VICVectAddr = 0;
}
void IR_CHECK(void)
{

    if((IOPIN1 & (1<<16)) == 0)
    {
        ir1_high_candidate = 0;

        if(ir1_low_candidate == 0)
        {
            ir1_low_candidate = 1;
            ir1_low_start = ms_count;
        }

        if(ir1_flag == 0 &&
           ms_count - ir1_low_start >= 20)
        {
            ir1_flag = 1;

            if(train_count < 9)
                train_count++;

            train_state = TRAIN_APPROACHING;

            safety_wait_flag = 0;
        }
    }
    else
    {
        ir1_low_candidate = 0;

        if(ir1_flag == 1)
        {
            if(ir1_high_candidate == 0)
            {
                ir1_high_candidate = 1;
                ir1_high_start = ms_count;
            }

            if(ms_count - ir1_high_start >= 20)
            {
                ir1_flag = 0;
                ir1_high_candidate = 0;
            }
        }
    }
    if((IOPIN1 & (1<<17)) == 0)
    {
        ir2_high_candidate = 0;

        if(ir2_low_candidate == 0)
        {
            ir2_low_candidate = 1;
            ir2_low_start = ms_count;
        }

        if(ir2_flag == 0 &&
           ms_count - ir2_low_start >= 20)
        {
            ir2_flag = 1;

            if(train_count > 0)
                train_state = TRAIN_ON_CROSSING;
        }
    }
    else
    {
        ir2_low_candidate = 0;

        if(ir2_flag == 1)
        {
            if(ir2_high_candidate == 0)
            {
                ir2_high_candidate = 1;
                ir2_high_start = ms_count;
            }

            if(ms_count - ir2_high_start >= 20)
            {
                ir2_flag = 0;
                ir2_high_candidate = 0;

                if(train_count > 0)
                {
                    train_count--;

                    if(train_count == 0)
                    {
                        delay_2sec_ecu1 = ms_count;
                        safety_wait_flag = 1;
                    }
                }
            }
        }
    }
}
void safety_check(void)
{
    if(train_count > 0)
        safety_wait_flag = 0;

    if(safety_wait_flag == 1)
    {
        if(ms_count - delay_2sec_ecu1 >= 2000)
        {
            if(train_count == 0)
                train_state = TRAIN_CLEARED;

            safety_wait_flag = 0;
        }
    }
}
int send_train_status(void)
{
    tx_msg.id = 0x100;
    tx_msg.dlc = 4;
    tx_msg.rtr = 0;
    tx_msg.byteA = (train_state & 0xFF)
                 | ((train_count & 0xFF) << 8)
                 | ((unsigned int)alive_counter << 24);

    tx_msg.byteB = 0;
    can2_tx(tx_msg);
    alive_counter++;
    return 1;
}
void receive_gate_status(void)
{
    if(can2_rx(&rx_msg))
    {
        if(rx_msg.id == 0x200 &&
           rx_msg.dlc == 4 &&
           rx_msg.rtr == 0)
        {
            gate_status = rx_msg.byteA & 0xFF;
            signal_status = (rx_msg.byteA >> 8) & 0xFF;
            buzzer_status = (rx_msg.byteA >> 16) & 0xFF;

            last_gate_rx_time = ms_count;
        }
    }
}

void check_communication_timeout(void)
{
    if(ms_count - last_gate_rx_time >= 500)
        communication_timeout = 1;
    else
        communication_timeout = 0;
}
void display_lcd(void)
{
    if(train_state != old_train_state ||
       train_count != old_train_count ||
       gate_status != old_gate_status ||
       communication_timeout != old_communication_timeout)
    {
        LCD_COMMAND(0x80);
        if(train_state == TRAIN_APPROACHING)
            LCD_STRING("APPROACH C:0 ");
        else if(train_state == TRAIN_ON_CROSSING)
            LCD_STRING("CROSSING C:0 ");
        else if(train_state == TRAIN_CLEARED)
            LCD_STRING("CLEARED C:0 ");
        else
            LCD_STRING("CLEAR C:0 ");
        LCD_COMMAND(0x8B);
        LCD_DATA(train_count + '0');
        LCD_COMMAND(0xC0);
        if(communication_timeout == 1)
            LCD_STRING("ECU2 LOST ");
        else
        {
            if(gate_status == GATE_OPEN)
                LCD_STRING("GATE: OPEN ");
            else if(gate_status == GATE_CLOSING)
                LCD_STRING("GATE: CLOSING ");
            else if(gate_status == GATE_CLOSED)
                LCD_STRING("GATE: CLOSED ");
            else if(gate_status == GATE_OPENING)
                LCD_STRING("GATE: OPENING ");
        }
        old_train_state = train_state;
        old_train_count = train_count;
        old_gate_status = gate_status;
        old_communication_timeout = communication_timeout;
    }
}
int main(void)
{
    VPBDIV = 1;
    IODIR1 &= ~((1<<16) | (1<<17));
    can2_init();
    timer1_init();
    LCD_INIT();
    startup_display();
    while(1)
    {
        IR_CHECK();
        safety_check();
        if(ms_count - can_tx_time >= 100)
        {
            if(send_train_status())
                can_tx_time = ms_count;
        }
        receive_gate_status();
        check_communication_timeout();
        if(ms_count - startup_time >= 2000)
        {
            display_lcd();
        }
    }
}

