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

void communicationCheck(){
    Serial.write(COM_CHECK);
}

int returnSpeed1(){
    //current_speed eh signed int, a velocidade pode ser negativa ou positiva
    current_speed1 = (current_position1-0)*(1)*(1);
    return current_speed1;
}

float returnSpeed2(){
    //current_speed eh signed int, a velocidade pode ser negativa ou positiva
    current_speed2 = (current_position2-last_position2)*(2*pi/8000)*(SAMPLE_FREQUENCY);
    return current_speed4;
}

float returnSpeed3(){
    //current_speed eh signed int, a velocidade pode ser negativa ou positiva
    current_speed3 = (current_position3-0)*(2*180.0/8000)*(1);
    return current_speed3;
}

float returnSpeed4(){
    //current_speed eh signed int, a velocidade pode ser negativa ou positiva
    current_speed4 = (current_position4-0)*(2*180.0/8000)*(1);
    return current_speed4;
}

void returnSpeed(){

    Serial.println(returnSpeed1());
    delay(8);
    Serial.println(2012);
    delay(8);
    Serial.println(3023);
    delay(8);
    Serial.println(4034);
    delay(8);
    Serial.println(i);
    delay(8);
}

void returnBatteryState(){};
void returnKeyState(){};
void stopEngine(){};
// the loop function runs over and over again forever
void loop() {

    // put your main code here, to run repeatedly:
    j=0;
    for(unsigned char l=0;l<9;l++)
        speeds[l]=l;

    for (i=0;i<100;i++){
        Serial.write(speeds,9);

        for(int k=0;k<9;k++)
            speeds[k]++;

        if (i>49 && j==0){
            delay(1000);
            j++;
        }
        delay(10);
    }
    delay (1000);
    //command = 0;
    //Check if command arrives
    // if(Serial.available()) {
    //Leitura Byte a Byte, enquanto chega byte, monta-se o numero

        //get incoming byte
        //char command = 70;
        //TODO: Check if command is valid, in this case send ACK, if not, ERROR

        //TODO if command is valid, enter switch case
        //call the function based on the command received
    //    switch (command) {
            // Command 1 returns communication ok
            // case 1:
            //     communicationCheck();
            //     break;
            // // Command 2 resolves speed and send back
            // case 2:
            //     returnBatteryState();
            //     break;
            // // Command 3 checks which battery is full
            // case 3:
            //     returnKeyState();
            //     break;
            // // Command 4 returns how many motors
            // case 0x5E:
            //     returnSpeed();
            //     break;
            // // Command 5 stop system
            // case 5:
            //     stopEngine();
            //     break;

            // default:
            //     Serial.print(command);

        // }
    //}
}
