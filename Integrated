
#include<avr/io.h>
#include<avr/interrupt.h>
#include<avr/delay.h>
#define SET_BIT(PORT,BIT) PORT|= (1<<BIT)
#define CLR_BIT(PORT,BIT) PORT&= ~(1<<BIT)
int percent;
unsigned int ADC_value_Brake;    //initilization
volatile unsigned int ADC_value_Wiper;
unsigned int counter=0;
float object_dist=0;  //   integer for access obstacle distance
int pulse = 0;                                      //   integer  to access all though the program
int edge = 0;                                       //   integer  for finding rising / falling edge
int flag = 0;

uint16_t ADC_value;


struct
{
    volatile unsigned int manual_flag:1;
    volatile int door_stat;
}FLAG;
int auto_mode = 1;



void ADC_setup()
{
  CLR_BIT(DDRC,PC2);
  ADMUX = 0b00000010;
  ADCSRA |=(1<<ADEN)|(1<<ADIE);    //starting conversion
}
void ADC_WIPER_setup()
{
  CLR_BIT(DDRC,PC1);
  //DDRC &=~(1<<PC1);
  ADMUX = 0b00000001;
  ADCSRA |=(1<<ADEN)|(1<<ADIE);
}

void ADC_cabinet_setup(){
                CLR_BIT(DDRC,PC0);
    ADMUX =0x00;
    //ADMUX |= (1<<REFS0) ;
    ADCSRA |= (1<<ADEN) | |(1<<ADIE);// | (1<<ADPS1) | (1<<ADPS2);
    //ADC=0x0000;
}


void PWM_setup()
{
                SET_BIT(DDRB,PB2);
                CLR_BIT(PORTB,PB2);
                TCCR0A|=(1<<WGM01);
                TCCR0B|=(1<<CS02)|(1<<CS00);
                OCR0A=0xFF;                                                     //TC0 Output Compare Register A
                OCR0B= 0X80;                                                    //50% DUTY CYCLE--
    TIMSK0 &=~ (1<<OCIE0A)|(1<<OCIE0B);
}

void brake(int x)
{
   if (x==1)
     SET_BIT(PORTB,PB5);
   else
     CLR_BIT(PORTB,PB5);
}

void send_trigger()
{
    SET_BIT(DDRD,PD2);//DDRD|=(1<<PD2);
                SET_BIT(PORTD,PD2);//PORTD|=(1<<PIND2);
                _delay_us(10);      ///triggering the sensor for 10usec
                CLR_BIT(PORTD,PD2);//PORTD &=~(1<<PIND2);
                CLR_BIT(DDRD,PD2);//DDRD&=~(1<<PD2);    // Make Echo pin as a input
}

void distance_calculation()
{
  int b;

  if(object_dist<30)
    {
    SET_BIT(PORTB,PB3);
    CLR_BIT(PORTB,PB0);
    CLR_BIT(PORTB,PB1);
    b=0;
    brake(b);
  }
  if((object_dist>30)&&(object_dist<60)){
    SET_BIT(PORTB,PB0);
    CLR_BIT(PORTB,PB3);
    CLR_BIT(PORTB,PB1);
    b=1;
    brake(b);
  }
  if(object_dist>60){
    SET_BIT(PORTB,PB1);
    CLR_BIT(PORTB,PB3);
    CLR_BIT(PORTB,PB0);
    b=1;
    brake(b);
  }
}



int main()
{
  SET_BIT(DDRD,PD3);
  SET_BIT(DDRD,PD4);
  SET_BIT(DDRD,PD5);
  CLR_BIT(DDRC,PC2);
  CLR_BIT(DDRC,PC1);
  CLR_BIT(DDRD,PD6);
  SET_BIT(PORTD,PD6);//internal pullup
  uint8_t pin_read=0x00;
  uint8_t REVERSE_read=0X00;
  sei();               //Global interrupt enable
  SET_BIT(DDRB,PB1);
  SET_BIT(DDRB,PB3);
  SET_BIT(DDRB,PB0);
  SET_BIT(DDRB,PB5);                     //putting portB output pins
  SET_BIT(DDRD,PD2);
  _delay_ms(50);                              //giving delay of 50ms
  EIMSK|=(1<<INT0);      //enabling ext interrupt
  EICRA|=(1<<ISC00);      //setting interrupt triggering logic change
  Serial.begin(9600);
  PWM_setup();

  ADC_setup();

  FLAG.manual_flag = 0;
    FLAG.door_stat = 0;
  SET_BIT(DDRD,PD0);


    // interrupt Manual Key
    CLR_BIT(DDRD,PD1);
    SET_BIT(PORTD,PD1);
    PCICR |= (1<<PCIE2);
    PCMSK2 |= (1<<PCINT17);


    // interrupt door stat
    CLR_BIT(DDRB,PB4);
    SET_BIT(PORTB,PB4);

    PCICR |= (1<<PCIE0);
    PCMSK0 |= (1<<PCINT4);

  while(1)
  {
    pin_read=PIND;
    REVERSE_read=PIND;
    if(!(pin_read & (1<<6)))
    {
        Serial.println("ENGINE ON");
        if(!(REVERSE_read & (1<<7)))
        {
        Serial.println("REVERSE_GEAR");
                send_trigger();
        _delay_ms(50);
        Serial.begin(9600);
        Serial.println("object_dist=");
        Serial.println(object_dist);
        distance_calculation();
        }
        else
        {
            CLR_BIT(PORTB,PB5);
            CLR_BIT(PORTB,PB0);
            CLR_BIT(PORTB,PB1);
            CLR_BIT(PORTB,PB3);
                Serial.println("REVERSE_GEAR OFF");

        }
        if(counter == 0)
        {
            ADCSRA |=(1<<ADSC);
            ADC_setup();
            _delay_ms(10);
            ADC_value_Brake = ADC;
                Serial.println("ADC :");
                Serial.println(ADC);
            if(ADC_value_Brake >50 && ADC_value_Brake < 300)
            {
                SET_BIT(PORTD,PD3);      //GREEN
                CLR_BIT(PORTD,PD4);
                CLR_BIT(PORTD,PD5);
                Serial.println("NORMAL BRAKING");
            }
           else if(ADC_value_Brake > 300 && ADC_value_Brake < 600)
            {
                CLR_BIT(PORTD,PD3);        //YELLOW
                SET_BIT(PORTD,PD4);
                CLR_BIT(PORTD,PD5);
                Serial.println("HARD BRAKING");
            }
           else if(ADC_value_Brake > 600 && ADC_value_Brake < 1023)
            {
                CLR_BIT(PORTD,PD3);
                CLR_BIT(PORTD,PD4);          //RED
                SET_BIT(PORTD,PD5);
                Serial.println("EMERGENCY BRAKING");
            }
        }
        else if(counter==1)
        {
            ADCSRA |=(1<<ADSC);
            ADC_WIPER_setup();
            Serial.println("ADC :");
            Serial.println(ADC);
            ADC_value_Wiper = map(ADC,0,1024,0,254);
            TIMSK0 |= (1<<OCIE0A)|(1<<OCIE0B);//PWM on
            Serial.println(ADC_value_Wiper);
            OCR0B = ADC_value_Wiper;
            _delay_us(100);
             if(ADC_value_Wiper>=1 && ADC_value_Wiper<=85)
                {
                 
                 Serial.println("WIPER LOW");
                }
            else if (ADC_value_Wiper>=86 && ADC_value_Wiper<=170)
                {
                 
                  Serial.println("WIPER MEDIUM");
                }
            else if (ADC_value_Wiper>=171 && ADC_value_Wiper<=254)
                {
                  Serial.println("WIPER HIGH");
                }
        }
  }
  else
    {
        Serial.println("ENGINE OFF");
                CLR_BIT(PORTD,PD3);
        CLR_BIT(PORTD,PD4);
        CLR_BIT(PORTD,PD5);
                CLR_BIT(PORTB,PB2);
                CLR_BIT(PORTB,PB1);
        CLR_BIT(PORTB,PB3);
        CLR_BIT(PORTB,PB4);
        TIMSK0 &= ~(1<<OCIE0A);
        TIMSK0 &= ~(1<<OCIE0B);
    }
}
}
//--------------ADC interrupt ------------------------------------
ISR(ADC_vect)
{
    if(counter==0)
  {
    counter++;
  }
  else if(counter==1)
  {
                counter++;
  }
  else if(counter==2)
  {
                counter=0;
  }
};

ISR(TIMER0_COMPA_vect)
{
  SET_BIT(PORTB,PB2);
};

ISR(TIMER0_COMPB_vect)
{
  CLR_BIT(PORTB,PB2);
};

ISR(INT0_vect)     //interrupt service routine when there is a change in logic level
{
   if (edge==1)                                    //when logic from HIGH to LOW
                {
                TCCR1B=0;                                          //disabling counter
                pulse=TCNT1;                    //count memory is updated to integer
                object_dist=pulse/2.18543;
                CLR_BIT(PORTD,PD2);
                TCNT1=0;                                             //resetting the counter memory
                edge=0;
    }

                if (edge==0)       //when logic change from LOW to HIGH
     {
                TCCR1B|=((1<<CS12)|(1<<CS10));     //enabling counter PRESCALAR 1024
                edge=1;
     }
};


