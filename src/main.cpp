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

//Confirmar se alguns não são unsigned
int V1 = 0;
int V2 = 0;
int V3 = 0;
int V4 = 0;
int last_position1 = 0;
int last_position2 = 0;
int last_position3 = 0;
int last_position4 = 0;
int current_position1 = 0;
int current_position2 = 0;
int current_position3 = 0;
int current_position4 = 0;
float current_speed1 = 0;
float current_speed2 = 0;
float current_speed3 = 0;
float current_speed4 = 0;
int inByte = 70;
int command=0;
int motor;
//int i;
byte speed1[] ={0,0};
signed int test = 34000;
float pos4_graus = 0;

unsigned char i;
int j;

unsigned char speeds [9] = {9 , 3, 12, 9, 7, 20, 13, 2, 5};

// the setup function runs once when you press reset or power the board
void setup() {
  analogWrite(PWM_MOT1, 0);
  analogWrite(PWM_MOT2, 0);
  analogWrite(PWM_MOT3, 0);
  analogWrite(PWM_MOT4, 0);
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
   //Set drivers pins modes
   pinMode(BRK_MOT1, OUTPUT);
   pinMode(BRK_MOT2, OUTPUT);
   pinMode(BRK_MOT3, OUTPUT);
   pinMode(BRK_MOT4, OUTPUT);

   pinMode(DISABLE1, OUTPUT);
   pinMode(DISABLE2, OUTPUT);
   pinMode(DISABLE3, OUTPUT);
   pinMode(DISABLE4, OUTPUT);

   pinMode(DIR_MOT1, OUTPUT);
   pinMode(DIR_MOT2, OUTPUT);
   pinMode(DIR_MOT3, OUTPUT);
   pinMode(DIR_MOT4, OUTPUT);

 //Da um pulso no pino reset para zerar o decodificador
   digitalWrite(RST, LOW);
   delay(400);
   digitalWrite(RST, HIGH);

   //set direction in motors
   digitalWrite(DIR_MOT1,HIGH);
   digitalWrite(DIR_MOT2,HIGH);
   digitalWrite(DIR_MOT3,HIGH);
   digitalWrite(DIR_MOT4,HIGH);
   //Turn off break (this signal is active in LOW)
   digitalWrite(BRK_MOT1,HIGH);
   digitalWrite(BRK_MOT2,HIGH);
   digitalWrite(BRK_MOT3,HIGH);
   digitalWrite(BRK_MOT4,HIGH);
   //Chose which motors are disabled (this signal is active in LOW)
   digitalWrite(DISABLE1,LOW);
   digitalWrite(DISABLE2,LOW);
   digitalWrite(DISABLE3,LOW);
   digitalWrite(DISABLE4,LOW);
   //Set the motors speed
   //analogWrite(PWM_MOT1, 50);
   //analogWrite(PWM_MOT2, 50);
   //analogWrite(PWM_MOT3, 50);
   //analogWrite(PWM_MOT4, 50);

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

  //
  delay(1000);

}

//***********FUNCTIONS*********************//

//Rotina de Interrupçao do timer 1
ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
    // Update last position
    last_position1 = current_position1;
    last_position2 = current_position2;
    last_position3 = current_position3;
    last_position4 = current_position4;
    // Clear position variable
    V1 = 0;
    V2 = 0;
    V3 = 0;
    V4 = 0;
//******LEITURA DOS ENCODERS*******
    //LEITURA DOS PINOS HIGH
    //Controle do Decodificador
    digitalWrite(OE, HIGH);
    digitalWrite(SEL, LOW);
    digitalWrite(OE, LOW);

    //Uliza-se shift de 8 bits para leitura do byte mais significativo
    //Using PORTA cause we're using decoder 1
    V1 = (PINA << 8);
    V2 = (PINC << 8);
    V3 = (PINL << 8);
    V4 = (PINB << 8);
    //LEITURA DOS PINOS LOW
    //Comando no pino SEL, agora os bits menos significativos serao lidos
    digitalWrite(SEL, HIGH);
    //Em caso de problemas, incluir delay
    V1 = V1 | PINA;
    V2 = V2 | PINC;
    V3 = V3 | PINL;
    V4 = V4 | PINB;
    //Update current positions
    current_position1 = V1;
    current_position2 = V2;
    current_position3 = V3;
    current_position4 = V4;
}

void motor_test(int motr){

    if(motr==1){
        Serial.println("Motor ativado");
        delay(4000);
        digitalWrite(DISABLE1,HIGH);
        Serial.println("Velocidade Alta");
        analogWrite(PWM_MOT1, 90);
        delay(3000);
        Serial.println("Velocidade Média");
        analogWrite(PWM_MOT1, 60);
        delay(3000);
        Serial.println("Velocidade Baixa");
        analogWrite(PWM_MOT1, 30);
        delay(3000);
        Serial.println("Freio");
        digitalWrite(BRK_MOT1,LOW);
        delay(3000);
        Serial.println("Motor desativado");
        digitalWrite(DISABLE1,LOW);
        // Turn off break, otherwise, when motor is enabled again, it won't spin
        digitalWrite(BRK_MOT1,HIGH);
        // Set speed back to 0
        analogWrite(PWM_MOT1, 0);
    }

    if(motr==2){
        Serial.println("Motor ativado");
        delay(4000);
        digitalWrite(DISABLE2,HIGH);
        Serial.println("Velocidade Alta");
        analogWrite(PWM_MOT2, 90);
        delay(3000);
        Serial.println("Velocidade Média");
        analogWrite(PWM_MOT2, 60);
        delay(3000);
        Serial.println("Velocidade Baixa");
        analogWrite(PWM_MOT2, 30);
        delay(3000);
        Serial.println("Freio");
        digitalWrite(BRK_MOT2,LOW);
        delay(3000);
        Serial.println("Motor desativado");
        digitalWrite(DISABLE2,LOW);
        // Turn off break, otherwise, when motor is enabled again, it won't spin
        digitalWrite(BRK_MOT2,HIGH);
        // Set speed back to 0
        analogWrite(PWM_MOT2, 0);
    }

    if(motr==3){
        Serial.println("Motor ativado");
        delay(4000);
        digitalWrite(DISABLE3,HIGH);
        Serial.println("Velocidade Alta");
        analogWrite(PWM_MOT3, 90);
        delay(3000);
        Serial.println("Velocidade Média");
        analogWrite(PWM_MOT3, 60);
        delay(3000);
        Serial.println("Velocidade Baixa");
        analogWrite(PWM_MOT3, 30);
        delay(3000);
        Serial.println("Freio");
        digitalWrite(BRK_MOT3,LOW);
        delay(3000);
        Serial.println("Motor desativado");
        digitalWrite(DISABLE3,LOW);
        // Turn off break, otherwise, when motor is enabled again, it won't spin
        digitalWrite(BRK_MOT3,HIGH);
        // Set speed back to 0
        analogWrite(PWM_MOT3, 0);
    }

    if(motr==4){
        Serial.println("Motor ativado");
        delay(4000);
        digitalWrite(DISABLE4,HIGH);
        Serial.println("Velocidade Alta");
        analogWrite(PWM_MOT4, 90);
        delay(3000);
        Serial.println("Velocidade Média");
        analogWrite(PWM_MOT4, 60);
        delay(3000);
        Serial.println("Velocidade Baixa");
        analogWrite(PWM_MOT4, 30);
        delay(3000);
        Serial.println("Freio");
        digitalWrite(BRK_MOT4,LOW);
        delay(3000);
        Serial.println("Motor desativado");
        digitalWrite(DISABLE4,LOW);
        // Turn off break, otherwise, when motor is enabled again, it won't spin
        digitalWrite(BRK_MOT4,HIGH);
        // Set speed back to 0
        analogWrite(PWM_MOT4, 0);
    }
}

void loop() {
    delay(100);
    Serial.println("Qual motor?");

    // Wait until some byte arrives
    while(Serial.available()==0){};
    delay(100);
    while(Serial.available()>0){
        //get incoming byte
        char inByte = Serial.read();
        //sending character back to serial port
        //Conversion ASCII to int
        motor = (int)inByte - 48;
        if ( ( motor<1 ) || ( motor>4 ) ){
            Serial.print("Valor ");
            Serial.print(motor);
            Serial.println(" não é aceito");}
        else{
            Serial.print("Iniciando teste com motor ");
            Serial.println(motor);
            motor_test(motor);
            }
        delay(1000);


    }

}
