#include <Adafruit_Sensor.h>                       // Biblioteca DHT Sensor Adafruit 
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>                     // usando a biblioteca Wire
#include <LiquidCrystal_I2C.h>        // usando a biblioteca LiquidCrystal I2C
LiquidCrystal_I2C lcdI2C(0x27, 16, 2);  // Configura endereço I2C e display com 16 caracteres e 2 linhas
#include <Servo.h> //Biblioteca do Servo Motor
Servo windowBack;
Servo windowFront;
Servo windowSide;
Servo gate;

#define DHTTYPE    DHT11                           // Sensor DHT11
#define DHTPIN 2                                   // Pino do Arduino conectado no Sensor(Data) 
DHT_Unified dht(DHTPIN, DHTTYPE);                  // configurando o Sensor DHT - pino e tipo

uint32_t delayMS; // variável para atraso no tempo
const char sensorAgua = A0;
const char ledPower = A1;
const char ledTemp = A2;
const char ledAgua = A3;
const int buzzer = 3;
const int pushButton = 12;
const int pushButtonGate = 11;
const int fan = 5;

// Variáveis Globais:
int stateWater = 0; // 0 = sem chuva 1 = com chuva
int stateWindows = 0; // 1 = fechada 0 = aberta
int statePushButton = 0; // 0 = nao pressionado 1 = pressionado
int stateSensorWater = 0; // 0 = sem chuva 1 = com chuva
int usePushButton = 0; // 0 = nãp usado 1 = usado
int statePushButtonGate = 0;
int usePushButtonGate = 0;
String statusTemp;

//Array que desenha o simbolo de grau
byte grau[8] = { B00001100,
                 B00010010,
                 B00010010,
                 B00001100,
                 B00000000,
                 B00000000,
                 B00000000,
                 B00000000,
               };

byte aTio[8] = {
  B01010,
  B00101,
  B01110,
  B00001,
  B01111,
  B10001,
  B01111,
  B00000
};
// Programa Principal:
void setup ()
{
  //Pinos
  pinMode (ledPower, OUTPUT);
  pinMode (ledAgua, OUTPUT);
  pinMode (ledTemp, OUTPUT);
  pinMode (sensorAgua, INPUT);
  pinMode (buzzer, OUTPUT);
  pinMode (pushButton, INPUT);
  pinMode (pushButtonGate, INPUT);
  pinMode (fan, OUTPUT);
  //Servos
  windowBack.attach(8);
  windowBack.write(120);
  gate.attach(9);
  gate.write(5);
  windowFront.attach(10);
  windowFront.write(120);
  windowSide.attach(7);
  windowSide.write(130);
  //Buzzer
  digitalWrite(buzzer, LOW);
  //Sensor
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);           // imprime os detalhes do Sensor de Temperatura
  Serial.begin(9600);
  Serial.println ("Iniciando Sensor:");
  Serial.print("Delay mínimo sensor: ");
  Serial.println(sensor.min_delay / 1000);
  Serial.println ("Iniciando a leitura de temperatura: ");
  Serial.println("------------------------------------");
  Serial.println("Temperatura");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Valor max:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Valor min:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolucao:   "); Serial.print(sensor.resolution); Serial.println(" *C");
  Serial.println("------------------------------------");
  dht.humidity().getSensor(&sensor);            // imprime os detalhes do Sensor de Umidade
  Serial.println ("Iniciando a leitura de temperatura: ");
  Serial.println("Umidade");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Valor max:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Valor min:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolucao:   "); Serial.print(sensor.resolution); Serial.println("%");
  Serial.println("------------------------------------");
  delayMS = sensor.min_delay / 1000;            // define o atraso entre as leituras
  Serial.println ("LCD E LEDs em operacao.");
  Serial.println("------------------------------------");
  Serial.println ("Ligando LED power: ");
  analogWrite (ledPower, 1023);
  //Display
  lcdI2C.init();                      // inicializa LCD
  lcdI2C.backlight();    // ativa led de backlight
  lcdI2C.setCursor (3, 0);
  lcdI2C.print ("CARREGANDO.");
  delay (1000);
  lcdI2C.clear();
  lcdI2C.setCursor (3, 0);
  lcdI2C.print ("CARREGANDO..");
  delay(1000);
  lcdI2C.clear();
  lcdI2C.setCursor (3, 0);
  lcdI2C.print ("CARREGANDO...");
  delay(1000);
  lcdI2C.clear();
  lcdI2C.setCursor (3, 0);
  lcdI2C.print ("BEM  VINDO");
  lcdI2C.setCursor (5, 1);
  lcdI2C.print ("SAMUEL");
  delay (2000);
  lcdI2C.clear ();
  lcdI2C.createChar(0, grau);
  lcdI2C.createChar(1, aTio);
}
void loop ()
{
  delay(500);
  //delay(delayMS);
  statePushButton = digitalRead(pushButton);
  stateSensorWater = analogRead(sensorAgua);
  stateWindows = windowBack.read();
  sensors_event_t event;                        // inicializa o evento da Temperatura
  dht.temperature().getEvent(&event);
  int t = event.temperature;
  if (isnan(event.temperature))                 // se algum erro na leitura
  {
    Serial.println("Erro na leitura da Temperatura!");
  }
  else                                          // senão
  {
    Serial.print("Temperatura: ");              // imprime a Temperatura
    Serial.print(event.temperature);
    Serial.println(" *C");
    Serial.println("------------------------------------");

  }
  dht.humidity().getEvent(&event);              // faz a leitura de umidade
  if (isnan(event.relative_humidity))           // se algum erro na leitura
  {
    Serial.println("Erro na leitura da Umidade!");
  }
  else                                          // senão
  {
    Serial.print("Umidade: ");                  // imprime a Umidade
    Serial.print(event.relative_humidity);
    Serial.println("%");
    Serial.println("------------------------------------");
  }
  Serial.print("SENSOR ÁGUA: ");
  Serial.println(verifyWater(analogRead(sensorAgua)));
  Serial.println("------------------------------------");
  Serial.print("Temperatura Tipo: ");
  Serial.println(verifyTemp(t));
  Serial.println("------------------------------------");
  Serial.print("Status Push Button: ");
  Serial.println(digitalRead(pushButton));
  Serial.println("------------------------------------");
  Serial.print("Status Windows: ");
  Serial.println(stateWindows);
  Serial.println("------------------------------------");
  Serial.print("Status Motor: ");
  Serial.println(analogRead(fan));
  Serial.println("------------------------------------");
  //Display I2C
  lcdI2C.setCursor (0, 0);
  lcdI2C.print ("Chuva: ");
  statePortao(digitalRead(pushButtonGate));
  stateWater = verifyWater(stateSensorWater);
  statusLcdWindows(stateWindows);
  openWindows(statePushButton, stateWindows, stateWater);
  closeWindows(statePushButton, stateWindows, stateWater);
  statusLcdWindows(stateWindows);
  lcdI2C.setCursor (0, 1);
  lcdI2C.print ("Temp: ");
  lcdI2C.setCursor (5, 1);
  lcdI2C.print (t);
  lcdI2C.write((byte)0);
  lcdI2C.print ("C");
  lcdI2C.setCursor(10, 1);
  lcdI2C.print("AR:");
  lcdI2C.print (verifyTemp(t));
}

int verifyWater(int data) {
  lcdI2C.setCursor (6, 0);
  if (data > 0) {
    lcdI2C.print ("Sim");
    return 1;
  } else if ( data <= 0) {
    lcdI2C.print ("N");
    lcdI2C.write ((byte)1);
    lcdI2C.print ("o");
    return 0;
  }
}

String verifyTemp(int data) {
  if (data <= 20) {
    analogWrite (ledTemp, 0);
    analogWrite(fan, 0);
    return "OFF";
  } else if (data > 20 && data <= 26) {
    analogWrite (ledTemp, 341);
    analogWrite(fan, 0);
    return "OFF";
  } else if (data > 26 && data < 28) {
    analogWrite (ledTemp, 682);
    analogWrite(fan, 0);
    return "OFF";
  } else if (data >= 28) {
    analogWrite (ledTemp, 1023);
    analogWrite(fan, 1023);
    return "ON ";
  } else {
    analogWrite(fan, 0);
    return "ON ";
  }
}

void openWindows(int statePushButton, int stateWindows, int stateWater) {
  if (statePushButton == 1 && stateWindows == 2) {
    digitalWrite(buzzer, HIGH);
    delay(1000);
    digitalWrite(buzzer, LOW);
    windowFront.write(120);
    delay(700);
    windowSide.write(130);
    delay(700);
    windowBack.write(120);
    usePushButton = 1;
  } else if (stateWater == 0 && stateWindows == 2 && usePushButton == 0) {
    digitalWrite(buzzer, HIGH);
    delay(1000);
    digitalWrite(buzzer, LOW);
    windowFront.write(120);
    delay(700);
    windowSide.write(130);
    delay(700);
    windowBack.write(120);
  } else if (stateWater = 0 && stateWindows == 2 && usePushButton == 1) {
    usePushButton = 0;
  }
}

void closeWindows(int statePushButton, int stateWindows, int stateWater) {
  if (statePushButton == 1 && stateWindows == 120) {
    digitalWrite(buzzer, HIGH);
    delay(1000);
    digitalWrite(buzzer, LOW);
    windowFront.write(8);
    delay(500);
    windowSide.write(8);
    delay(500);
    windowBack.write(2);
    usePushButton = 1;
  } else if (stateWater == 1 && stateWindows == 120 && usePushButton == 0) {
    digitalWrite(buzzer, HIGH);
    delay(1000);
    digitalWrite(buzzer, LOW);
    windowFront.write(8);
    delay(700);
    windowSide.write(8);
    delay(700);
    windowBack.write(2);
  } else if (stateWater = 1 && stateWindows == 120 && usePushButton == 1) {
    usePushButton = 0;
  }
}

void statusLcdWindows(int statusWindows) {
  lcdI2C.setCursor(10, 0);
  if (statusWindows == 120) {
    lcdI2C.print ("Opened");
  } else if (statusWindows == 2) {
    lcdI2C.print ("Closed");
  }
}

void statePortao(int pushButtonPortao) {
  if (pushButtonPortao == 1) {
    usePushButtonGate = !usePushButtonGate;
  }
  if (usePushButtonGate == 1) {
    gate.write(120);
  } else if (usePushButtonGate == 0) {
    gate.write(5);
  }
}
