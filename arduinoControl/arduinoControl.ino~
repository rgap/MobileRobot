#include <Servo.h>


/////////////
int actionTime = 1000;

// servos pala superior
Servo srvPalaSupIzq;
Servo srvPalaSupDer;
// servos parte inferior
Servo srvPalaInfIzq;
Servo srvPalaInfDer;

int posPalaSupIzq;
int posPalaInfIzq;

int dataSerial = 0; // variable to store the data from the serial port

///////////////////////////// LLANTAS

void action_parar() {
  //PARAR eje 1
  digitalWrite(11, LOW);
  digitalWrite(10, LOW);
  //PARAR eje 2
  digitalWrite(9, LOW);
  digitalWrite(8, LOW);

  Serial.println("parar");
}

void action_avanzar(int miliseconds) {
  //avanzarConEje1
  digitalWrite(11, HIGH);
  digitalWrite(10, LOW);
  //retrocederEje2
  digitalWrite(9, LOW);
  digitalWrite(8, HIGH);  

  Serial.println("avanzar");

  delay(miliseconds);
}

void action_retroceder(int miliseconds) {
  //avanzarConEje2
  digitalWrite(9, HIGH);
  digitalWrite(8, LOW);
  //retrocederEje1
  digitalWrite(11, LOW);
  digitalWrite(10, HIGH);

  Serial.println("retroceder");

  delay(miliseconds);
}

void action_girarDer(int miliseconds) {
  //retrocederEje1
  digitalWrite(11, LOW);
  digitalWrite(10, HIGH);
  //retrocederEje2
  digitalWrite(9, LOW);
  digitalWrite(8, HIGH);

  Serial.println("girar der");

  delay(miliseconds);
}

void action_girarIzq(int miliseconds) {
  //avanzarConEje1
  digitalWrite(11, HIGH);
  digitalWrite(10, LOW);
  //avanzarConEje2
  digitalWrite(9, HIGH);
  digitalWrite(8, LOW);

  Serial.println("girar izq");
  
  delay(miliseconds);
}

///////////////////////////// PALA

void palaSup_moveToAngle(int angle_Sup){
  srvPalaSupIzq.write(angle_Sup);
  srvPalaSupDer.write(180 - angle_Sup);
  posPalaSupIzq = angle_Sup;
}

void palaInf_moveToAngle(int angle_Inf){
  srvPalaInfIzq.write(angle_Inf);
  srvPalaInfDer.write(180 - angle_Inf);
  posPalaInfIzq = angle_Inf;
}


void action_palaPosInicial(){
  //ANGULO IZQUIERDO
  int angle_Sup = 0;
  int angle_Inf = 120;
  //pala superior abajo
  palaSup_moveToAngle(angle_Sup);
  //pala inferior arriba
  palaInf_moveToAngle(angle_Inf);

  Serial.println("posicion inicial palas");
}

void setTime(int &timeAc, int &time, const int &timeval){
  time = timeval;
  timeAc += time;
}

void action_recogerLata(){
  
  action_palaPosInicial();
  
  int time = 0, timeAc = 0;

  //AVANZAR
  setTime(timeAc, time, 1000);
  action_avanzar(time); 
  action_parar();

  // MOVER PALA INFERIOR ABAJO
  palaInf_moveToAngle(0);
  palaSup_moveToAngle(0);
  setTime(timeAc, time, 500);
  delay(time);

  //////////
  int i, j, notCompletedSup, notCompletedInf;
  i = posPalaSupIzq, j = posPalaInfIzq;
  notCompletedSup = notCompletedInf = 1;
  
  int posPalaSupIzq_, posPalaInfIzq_;
  posPalaSupIzq_ = 108, posPalaInfIzq_ = 90; // ANGULOS FINALES

  while (notCompletedSup || notCompletedInf) {
    
    if (j <= posPalaInfIzq_) {
      palaInf_moveToAngle(j);
      j++;
    } else {
      notCompletedInf = 0;
    }
    
    if (i <= posPalaSupIzq_) {
      palaSup_moveToAngle(i);
      i++;
    } else {
      notCompletedSup = 0;
    }
    setTime(timeAc, time, 20);
    delay(time);

  }

  setTime(timeAc, time, 1000);
  delay(time);
  setTime(timeAc, time, 250);
  action_avanzar(time);
  action_parar();
  
  // MOVER PALA INFERIOR ABAJO
  palaInf_moveToAngle(10);
  setTime(timeAc, time, 500);
  delay(time);
  
  action_palaPosInicial();

  Serial.println("recoger lata: ");
  Serial.print(timeAc);
  Serial.println("");
}

void action_buscar(){
  action_avanzar(1200);
  delay(1000);
  action_girarDer(1000);
  action_parar();
}

///////////////////////////// INITIALIZATION

void setup() {
  pinMode(11, OUTPUT); // declare the LED's pin as output
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  srvPalaSupIzq.attach(2); //servo superior de la izquierda
  srvPalaSupDer.attach(3); //servo superior  de la derecha

  srvPalaInfIzq.attach(4); //servo inferior izquierda
  srvPalaInfDer.attach(5); //servo inferior derecha

  Serial.begin(9600); // connect to the serial port

  ///////////////////

  action_palaPosInicial();
}

void loop() {
  dataSerial = Serial.read(); // read the serial port

  if (dataSerial > '0' && dataSerial <= 'F') {

    switch (dataSerial) {
    case '1': //PARAR
      action_parar();
      break;
    case '2': //AVANZAR
      action_avanzar(0);
      break;
    case '3': //RETROCEDER
      action_retroceder(0);
      break;
    case '4': //GIRO DERECHA
      action_girarDer(0);
      break;
    case '5': //GIRO IZQUIERDA
      action_girarIzq(0);
      break;
    case '6': //POSICION INICIAL PALAS
      action_palaPosInicial();
      break;
    case '7': //RECOGER LATA
      action_recogerLata();
      break;
    case '8': //BUSCAR
    action_girarDer(500);
      action_parar();
      action_avanzar(300);
      action_parar();
      break;
    case '9':
      palaInf_moveToAngle(0);
      Serial.println("TEST");
      break;
    case 'A':
      palaInf_moveToAngle(110);
      Serial.println("TEST");
      break;  
    case 'B':
      action_avanzar(500);
      action_parar();
      Serial.println("TEST");
      break;    
    case 'C':
      palaSup_moveToAngle(8);
      Serial.println("TEST");
      break;
    case 'D':
      palaSup_moveToAngle(95);
      Serial.println("TEST");
      break;
    case 'E':
      action_girarIzq(300);
      action_parar();
      Serial.println("TEST");
      break;  
    case 'F':
      action_girarDer(300);
      action_parar();
      Serial.println("TEST");
      break;   
    default:
      digitalWrite(11, LOW);
      digitalWrite(12, LOW);
      digitalWrite(11, LOW);
      digitalWrite(12, LOW);
    }

  }
}

