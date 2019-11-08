// SENSORES
#define IR                              4 // mirar pin de sensor 
#define S0                              A0
#define S1                              A1
#define S2                              A2
#define S3                              A3
#define S4                              A4
#define S5                              A5
#define S6                              A6
#define S7                              A7
// MOTORES
#define PWMA                            11
#define AIN2                            10
#define AIN1                            9
#define STBY                            8
#define BIN1                            7
#define BIN2                            6
#define PWMB                            5
// BOTON
#define boton                           3
//INTERRUPTOR
#define interruptor                     12
//LED
#define led                             13

#define kp 0.5  //valor de la cte. proporcional 
#define kd 10  //valor de la cte. diferencial

const int sensores[8] = {S0, S1, S2, S3, S4, S5, S6, S7};
signed int error = -4 * sensores[0] - 3 * sensores[1] - 2 * sensores[2] - sensores[3] + sensores[4] + 2 * sensores[5] + 3 * sensores[6] + 4 * sensores[7];
signed int lectura_valores;
float valor_proporcional;
float valor_diferencial;
signed int last_error;
float correccion;
char x;
byte valor_digital;
unsigned int sensorMax[8];
unsigned int sensorMin[8];
unsigned int posicion;
int valores[8], velocidad_motorA, velocidad_motorB, velocidad_avance;
int valores_digitales[8];
bool sentido_motorA, sentido_motorB, avance = 1, retroceso = 0;
//declaracion vectores motores
const int motorA[3] = {PWMA, AIN1, AIN2};
const int motorB[3] = {PWMB, BIN1, BIN2};
unsigned int tiempo_calibrado;
unsigned int media_blanco[8];
float paramos, tiempo_inicial;
bool empezar;

void calculo_pid() {

  digitalWrite(IR, 1);
    valor_digital = 0;
  // Serial.print("lectura_valores: ");
  for (char x = 0; x < 8; x++) {
    valores[x] = analogRead(sensores[x]);

    // apply the calibration to the sensor reading
    valores[x] = map(valores[x], sensorMin[x], sensorMax[x], 0, 1024);
  
   // Serial.print(valores[x]);

   // Serial.print(" , ");
    if (valores[x] > media_blanco[x])
    {
          valores_digitales[x] = 1;
    } else
    {
         valores_digitales[x] = 0;
    }
    valor_digital = valor_digital + valores_digitales[x];

  }
  


 // Serial.println();


  error = -4 * valores[0] - 3 * valores[1] - 2 * valores[2] - valores[3] + valores[4] + 2 * valores[5] + 3 * valores[6] + 4 * valores[7];
  error = map(error,-7000,7000,-500,500);
  if(valor_digital == 0 && last_error < 0)
  {
    error = -500;
  }else if(valor_digital == 0 && last_error > 0)
  {
    error = 500;
  }
  //Serial.print("error: ");
  
 // Serial.print(error);
//Serial.print("    ");

 // Serial.println();
  //  delay(250);
  // delay(500);
  // algoritmo PID
  valor_proporcional = error * kp;
  valor_diferencial = (error - last_error) * kd;
//  if (error > 20 || error < -20)
 // {
    correccion = valor_proporcional + valor_diferencial;
 // }


  last_error = error;
 // valor_digital = 0;
}
//-----------------------------------  FUNCIONES DE MOVIMIENTO  ------------------------------------------//

void avanzar ()
{
  // lectura_orientacion();
  // calculo_pid();
  velocidad_motorA = velocidad_avance;// - (signed int)correccion;
  velocidad_motorB = velocidad_avance;// + (signed int)correccion;

  if (velocidad_motorA > 255)
  {
    velocidad_motorA = 255;
  } else if (velocidad_motorA < 0)
  {
    velocidad_motorA = 0;
  }
  if (velocidad_motorB > 255)
  {
    velocidad_motorB = 255;
  } else if (velocidad_motorB < 0)
  {
    velocidad_motorB = 0;
  }
  mover_motor(motorA, avance, velocidad_motorA);
  mover_motor(motorB, avance, velocidad_motorB);
}

void retroceder ()
{
  mover_motor(motorA, retroceso, velocidad_avance);
  mover_motor(motorB, retroceso, velocidad_avance);
}

void parada_total()
{
  apagar_motor(motorA, 0, 0);
  apagar_motor(motorB, 0, 0);

}
void parada()
{
  mover_motor(motorA, avance, 0);
  mover_motor(motorB, avance, 0);
}
void encender_motores()
{
  digitalWrite(STBY, 1);
}
void apagar_motor(const int motor[3], bool sentido, int velocidad)
{
  digitalWrite(STBY, 0);

  digitalWrite(motor[1], sentido);
  digitalWrite(motor[2], sentido);

  analogWrite(motor[0], velocidad);
}
void mover_motor(const int motor[3], bool sentido, int velocidad)
{
  digitalWrite(motor[1], sentido);
  digitalWrite(motor[2], !sentido);

  analogWrite(motor[0], velocidad);
}
//-----------------------------------  FIN FUNCIONES MOVIMIENTO  ----------------------------------------//

void setup() {
//  Serial.begin(9600);
  //declaracion de pines de motores
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  // declaracion de pines de sensores
  pinMode(IR, OUTPUT);
  pinMode(S0, INPUT);
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
  pinMode(S6, INPUT);
  pinMode(S7, INPUT);
  // declaracion de pin de boton
  pinMode(boton, INPUT_PULLUP);
  pinMode(interruptor, INPUT_PULLUP);
  pinMode(led, OUTPUT);
  digitalWrite(IR, 1); // IR a 1, enciende sensores de linea

  // El LED se enciende para indicar el periodo de calibración

  // ---------------------------------------- CALIBRADO ----------------------------------------
  if(digitalRead(boton) == 0)
  {
    velocidad_avance = 255;
  }else
  {
    velocidad_avance = 200;
  }
  delay(500);
  for (x = 0; x < 8; x++) {
    lectura_valores = analogRead(sensores[x]);
    sensorMin[x] = lectura_valores;
  }
  tiempo_calibrado = millis() + 5000;
  digitalWrite(led, HIGH);
  while (millis() < tiempo_calibrado) { // Tiempo de calibrado
    for (x = 0; x < 8; x++) {
      lectura_valores = analogRead(sensores[x]);
      if (lectura_valores > sensorMax[x]) {      // Registrar el valor máximo del sensor
        sensorMax[x] = lectura_valores;
      }
      if (lectura_valores < sensorMin[x]) {      // Registrar el valor mínimo del sensor
        sensorMin[x] = lectura_valores;
      }
    }
    // Señal para el fin de la calibración

  }
  for (x = 0; x < 8; x++) {
  media_blanco[x] = (sensorMin[x] + sensorMax[x]) / 2;
  }
  digitalWrite(led, LOW);
  delay(500);
  /*for (x = 0; x < 8; x++) {
    Serial.print(sensorMin[x]);
    Serial.print("  ,  ");
    Serial.print(sensorMax[x]);
    Serial.print("  ,  ");
    media_blanco[x] = (sensorMin[x] + sensorMax[x]) / 2;
    Serial.println(media_blanco[x]);
  }*/
  encender_motores();
}



void loop() {
   velocidad_avance = 220;
   // avanzar();
   // return;
 /*    mover_motor(motorA, 0,225);
       mover_motor(motorB, 1,225);
   while(1);*/
  if (digitalRead(boton) == 0)
  {
    for (x = 0; x < 5; x++)
    {
      digitalWrite(led, HIGH);
      delay(500);
      digitalWrite(led, LOW);
      delay(500);
    }
    
    //  avanzar();
   
    
    while (1)
    {
      calculo_pid();
      velocidad_motorA = velocidad_avance + correccion;
      velocidad_motorB = velocidad_avance - correccion;
        /*    Serial.print(velocidad_motorB);
            Serial.print("     ");
            Serial.println(velocidad_motorA);*/
           // Serial.println();
      //  delay(500);
      // seguridad de sobre-velocidad
      if (velocidad_motorA > 250) {
        velocidad_motorA = 250;
      }
      if (velocidad_motorB > 250) {
        velocidad_motorB = 250;
      }
      if (velocidad_motorA < 0) {
        velocidad_motorA = -velocidad_motorA;
        sentido_motorA = retroceso;
      } else
      {
        sentido_motorA = avance;
      }
      if (velocidad_motorB < 0) {
        velocidad_motorB = -velocidad_motorB;
        sentido_motorB = retroceso;
      } else
      {
        sentido_motorB = avance;
      }

      // implementacion de los valores en los motores para su ejecucion
      mover_motor(motorA, sentido_motorA, velocidad_motorA);
      mover_motor(motorB, sentido_motorB, velocidad_motorB);

      /*if((valor_digital == 0))
      {
        if(!empezar)
        {
       //   paramos = 0;
          empezar = 1;
           digitalWrite(led,1);
            parada();
            while(digitalRead(boton) != 0);
        }
        paramos++;
        if(paramos >= 150)
        {
          parada();
         
       /*  mover_motor(motorA, 0, 0);
        mover_motor(motorB, 0, 0);
          while(1){
            digitalWrite(led,1);
            delay(200);
            digitalWrite(led,0);
            delay(200);

          }
        }
      }else
      {
        empezar = 0;
      }*/
    }
  }

}
