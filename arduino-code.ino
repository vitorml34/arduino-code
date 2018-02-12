/*
Explicaço do codigo
*/

//Program Defines
#define SAMPLE_FREQUENCY 20
#define COM_CHECK 75
#define SEL 38
#define OE  39
#define RST 40
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
   pinMode(39, OUTPUT);
 //Da um pulso no pino reset para zerar o decodificador
   digitalWrite(RST, LOW);
   delay(400);
   digitalWrite(RST, HIGH);

//Configuraçoes do Timer 4, usado para fazer a leitura constante do Encoder
// initialize timer4
  noInterrupts();           // disable all interrupts
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 0;

  OCR4A = (int)(16000000/1024/SAMPLE_FREQUENCY);    // compare match register 16MHz/256/2Hz
  TCCR4B |= (1 << WGM42);   // CTC mode (zera o timer quando atinge o valor definido)
  TCCR4B |= (1 << CS42);    // 1024 prescaler
  TCCR4B |= (1 << CS40);    // 1024 prescaler
  TIMSK4 |= (1 << OCIE4A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts

 //CLOCK DE 1MHZ
 //Utiliza-se o timer 3 para gerar um clock de 1MHz no pino OC3A (No arduino mega, pino 5 ou PE3) do arduino mega.
  DDRE = _BV(DDE3);                  //set OC3A
  TCCR3A = _BV(COM3A0);              //toggle OC3A on compare match
  OCR3A = 7;                         //top value for counter
  TCCR3B = _BV(WGM12) | _BV(CS30);   //CTC mode, prescaler clock/1

}


//***********FUNCTIONS*********************//

//Rotina de Interrupçao do timer 3
ISR(TIMER4_COMPA_vect)          // timer compare interrupt service routine
{
    last_position = current_position;
    DA=0;  //Int de 16 bits aonde a leitura ser armazenada
//******LEITURA DOS ENCODERS*******
    //LEITURA DOS PINOS HIGH
    //Controle do Decodificador
    digitalWrite(OE, HIGH);
    digitalWrite(SEL, LOW);
    digitalWrite(OE, LOW);

    DA = (PINA << 8); //Uliza-se shift de 8 bits para leitura do byte mais significativoud
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
    while(Serial.available()>0)
    {
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
