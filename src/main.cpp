#include <Arduino.h>
/*
Explicaço do codigo
*/
//Program Defines
//If it becomes too big, create a define file for this
#define SAMPLE_FREQUENCY 20
#define COM_CHECK 75
#define SEL 38
#define OE  39
#define RST 40
#define PWM_MOT4 6
#define PWM_MOT3 7
#define PWM_MOT2 8
#define PWM_MOT1 9
#define DISABLE1 A0
#define DISABLE2 A1
#define DISABLE3 A2
#define DISABLE4 A3
#define SW4      A4
#define SW3      A5
#define SW2      A6
#define SW1      A7
#define DIR_MOT1 A8
#define DIR_MOT2 A9
#define DIR_MOT3 A10
#define DIR_MOT4 A11
//Attention, break pins are active in LOW
#define BRK_MOT1 A12
#define BRK_MOT2 A13
#define BRK_MOT3 A14
#define BRK_MOT4 A15
//      CLK  5  (Apenas para documentacao, essa definicao eh feita de maneira automatica.
#define pi   3.14159

int DA = 0;
int last_position = 0;
int current_position = 0;
float current_speed = 0;
int inByte = 0;
int command=0;

// the setup function runs once when you press reset or power the board
void setup() {
  // start serial port at 9600 bps
  Serial.begin(9600);
  delay(100);
//Coloca os bits do port D (2 a 7) como entradas, sem alterar o valor dos bits 0 e 1, que sao RX e TX
//  DDRD = DDRD & B00000011;
// Coloca o port A como entrada
   DDRA = 0b00000000;
 //Seta pinos de controle do decodificador como saida
   pinMode(RST, OUTPUT);
   pinMode(SEL, OUTPUT);
   pinMode(OE, OUTPUT);
   pinMode(41, OUTPUT);
   //Set drive 1 pin modes
   pinMode(BRK_MOT2, OUTPUT);
   pinMode(DISABLE2, OUTPUT);
   pinMode(DIR_MOT2, OUTPUT);
   pinMode(BRK_MOT1, OUTPUT);
   pinMode(DISABLE1, OUTPUT);
   pinMode(DIR_MOT1, OUTPUT);

 //Da um pulso no pino reset para zerar o decodificador
   digitalWrite(RST, LOW);
   delay(400);
   digitalWrite(RST, HIGH);

   //set some speed and direction in motors
   digitalWrite(DIR_MOT2,HIGH);
   digitalWrite(BRK_MOT2,HIGH);
   digitalWrite(DISABLE2,HIGH);
   digitalWrite(DIR_MOT1,HIGH);
   digitalWrite(BRK_MOT1,HIGH);
   digitalWrite(DISABLE1,HIGH);
   analogWrite(PWM_MOT1, 70);
   analogWrite(PWM_MOT2, 70);

//Configuraçoes do Timer 1, usado para fazer a leitura constante do Encoder
// initialize timer1
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = (int)(16000000/1024/SAMPLE_FREQUENCY);    // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode (zera o timer quando atinge o valor definido)
  TCCR1B |= (1 << CS12);    // 1024 prescaler
  TCCR1B |= (1 << CS10);    // 1024 prescaler
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts

 //CLOCK DE 1MHZ
 //Utiliza-se o timer 3 para gerar um clock de 1MHz no pino OC3A (No arduino mega, pino 5 ou PE3) do arduino mega.
  DDRE = _BV(DDE3);                  //set OC3A
  TCCR3A = _BV(COM3A0);              //toggle OC3A on compare match
  OCR3A = 7;                         //top value for counter
  TCCR3B = _BV(WGM12) | _BV(CS30);   //CTC mode, prescaler clock/1
}

//***********FUNCTIONS*********************//

//Rotina de Interrupçao do timer 1
ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
    last_position = current_position;
    DA=0;  //Int de 16 bits aonde a leitura ser armazenada
//******LEITURA DOS ENCODERS*******
    //LEITURA DOS PINOS HIGH
    //Controle do Decodificador
    digitalWrite(OE, HIGH);
    digitalWrite(SEL, LOW);
    digitalWrite(OE, LOW);

    //Uliza-se shift de 8 bits para leitura do byte mais significativo
    //Using PORTA cause we're using decoder 1
    DA = (PINA << 8);
    //LEITURA DOS PINOS LOW
    //Comando no pino SEL, agora os bits menos significativos serao lidos
    digitalWrite(SEL, HIGH);
    //Em caso de problemas, incluir delay
    DA = DA | PINA;
    current_position = DA;
}

void communicationCheck(){
    Serial.write(COM_CHECK);
}

float returnSpeed(){
    //current_speed eh signed int, a velocidade pode ser negativa ou positiva
    current_speed = (current_position-last_position)*(2*pi/8000)*(SAMPLE_FREQUENCY);
    return current_speed;
}

void returnBatteryState(){};
void returnKeyState(){};
void stopEngine(){};
// the loop function runs over and over again forever
void loop() {
    command = 0;
    //Check if command arrives
    if(Serial.available()) {
    //Leitura Byte a Byte, enquanto chega byte, monta-se o numero
        while(Serial.available()>0){
            //get incoming byte
            char inByte = Serial.read();
            //sending character back to serial port
            //Conversion ASCII to int
            int c = (int)inByte - 48;
            command *= 10;
            command += c;
            delay(100);
        }
        Serial.print("Comando recebido: ");
        Serial.print(command,DEC);
        Serial.print('\n');

        //call the function based on the command received
        switch (command) {
            // Command 1 returns communication ok
            case 1:
                communicationCheck();
                break;
            // Command 2 resolves speed and send back
            case 2:
                returnBatteryState();
                break;
            // Command 3 checks which battery is full
            case 3:
                returnKeyState();
                break;
            // Command 4 returns how many motors
            case 4:
                returnSpeed();
                break;
            // Command 5 stop system
            case 5:
                stopEngine();
                break;
        }
    }
}
