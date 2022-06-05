#include <Adafruit_Sensor.h>                       // Biblioteca DHT Sensor Adafruit 
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>                     // usando a biblioteca Wire
#include <LiquidCrystal_I2C.h>        // usando a biblioteca LiquidCrystal I2C
LiquidCrystal_I2C lcdI2C(0x27, 16, 2);  // Configura endereço I2C e display com 16 caracteres e 2 linhas

// selecione um sensor, retirando o comentário - duas barras
#define DHTTYPE    DHT11                           // Sensor DHT11
//#define DHTTYPE      DHT22                       // Sensor DHT22 ou AM2302

#define DHTPIN 2                                   // Pino do Arduino conectado no Sensor(Data) 
DHT_Unified dht(DHTPIN, DHTTYPE);                  // configurando o Sensor DHT - pino e tipo
uint32_t delayMS; // variável para atraso no tempo

const int ledAviso = 8;
const int ledPower = 3;
const int ledTempMin = 4;
const int ledTempMed = 5;
const int ledTempMax = 6;
const int ledAgua = 7;
const int sensorAgua = A0;

// Variáveis Globais:
int temperatura;
int tempMax = 40;
int statusWindows = 1; // 0 = fechada 1 = aberta

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
  Serial.begin(9600);
  //Sensor
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);           // imprime os detalhes do Sensor de Temperatura
  Serial.println ("Iniciando Sensor:");
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
  pinMode (ledPower, OUTPUT);
  digitalWrite (ledPower, HIGH);
  pinMode (ledAgua, OUTPUT);
  pinMode (ledTempMin, OUTPUT);
  pinMode (ledTempMed, OUTPUT);
  pinMode (ledTempMax, OUTPUT);
  pinMode (sensorAgua, INPUT);
  //Display
  lcdI2C.init();                      // inicializa LCD
  lcdI2C.backlight();    // ativa led de backlight
  lcdI2C.setCursor (3, 0);
  lcdI2C.print ("CARREGANDO");
  lcdI2C.setCursor (4, 1);
  lcdI2C.print ("E-FREEZE");
  delay (2000);
  lcdI2C.clear ();
  digitalWrite (ledPower, HIGH);
  delay (250);
  lcdI2C.createChar(0, grau);
  lcdI2C.createChar(1, aTio);
}
void loop ()
{
  delay(delayMS);
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
  Serial.print("LED Min");
  Serial.println(digitalRead(ledTempMin));
  Serial.print("LED Med");
  Serial.println(digitalRead(ledTempMed));
  Serial.print("LED Max");
  Serial.println(digitalRead(ledTempMax));

  Serial.println("------------------------------------");
  //Display I2C
  lcdI2C.setCursor (0, 0);
  lcdI2C.print ("Chuva: ");
  lcdI2C.setCursor (6, 0);
  if (verifyWater(analogRead(sensorAgua))) {
    lcdI2C.print ("Sim");
    statusWindows = 0;
  } else {
    lcdI2C.print ("N");
    lcdI2C.write ((byte)1);
    lcdI2C.print ("o");
    statusWindows = 1;
  }
  lcdI2C.setCursor(10, 0);
  if (statusWindows == 0) {
    lcdI2C.print ("Closed");
  } else {
    lcdI2C.print ("Opened");
  }
  lcdI2C.setCursor (0, 1);
  lcdI2C.print ("Temp: ");
  lcdI2C.setCursor (5, 1);
  lcdI2C.print (t);
  lcdI2C.write((byte)0);
  lcdI2C.print ("C");
}

boolean verifyWater(int data) {
  if (data >= 100) {
    return true;
  } else {
    return false;
  }
}

int verifyTemp(int data) {
  if (data <= 20) {
    digitalWrite (ledTempMin, HIGH);
    digitalWrite (ledTempMed, LOW);
    digitalWrite (ledTempMax, LOW);
    return 1;
  } else if (data > 20 && data <= 29) {
    digitalWrite (ledTempMed, HIGH);
    digitalWrite (ledTempMin, LOW);
    digitalWrite (ledTempMax, LOW);
    return 2;
  } else if (data >= 30) {
    digitalWrite (ledTempMax, HIGH);
    digitalWrite (ledTempMin, LOW);
    digitalWrite (ledTempMed, LOW);
    return 3;
  }
}
