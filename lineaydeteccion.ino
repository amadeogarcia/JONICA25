// ruedas chiquitas. ojo que en el medio de 2 sensores nunca puede dar 0 pq sino se va al pingo, x eso baje el umbral.

// --- Pines de motores 
static int PinIN1 = 11; 
static int PinIN2 = 9;
static int PinIN3 = 6;
static int PinIN4 = 3;

// --- Pines de sensores (¿INTERCAMBIAMOS IZQUIERDO Y DERECHO PARA LA PRUEBA?)
static int Pin_sensor_izq = A2; // El sensor físico izquierdo ahora se lee en el pin A2
static int Pin_sensor_cen = A1; // El del centro no cambia
static int Pin_sensor_der = A0; // El sensor físico derecho ahora se lee en el pin A0
static int Pin_sensor_uder = A4; 
static int Pin_sensor_uizq = A3; 

static int PinSemaforo = 13;  // Pin que recibe la señal de la Raspberry
bool semaforo_verde = 0, arranque=0;
static int PinPare = 12;  // Pin que recibe la señal de la Raspberry
bool senialpare=0, stopflag=0;
unsigned long tiempo_actual = 0, tiempo_inicio=0, tiempo_pasado=0;


int digital[5];
int sensoresanalog[5];
long int sumap, suma, pos, poslast, posicion;

// --- UMBRAL DE SENSIBILIDAD ---
int UMBRAL = 400;
int umbralmedio=200; 


float KP=0.5;
float KD=0.3;
float KI=0.01;   // probar 0.01


/// datos para la integral
int error1=0;
int error2=0;
int error3=0;
int error4=0;
int error5=0;
int error6=0;
/////////////////////////

///////////variable PID///////////////
int proporcional=0;
int integral=0;
int derivativo=0;
int diferencial=0;
int last_prop;
int setpoint=200;


void setup() {
  Serial.begin(9600);
  pinMode(PinIN1, OUTPUT);
  pinMode(PinIN2, OUTPUT);
  pinMode(PinIN3, OUTPUT);
  pinMode(PinIN4, OUTPUT);
  pinMode(PinSemaforo, INPUT_PULLUP);
  pinMode(PinPare, INPUT_PULLUP);

  Serial.println("--- Inicio de Depuración (Pines Cruzados) ---");
}



void loop() {

  if (arranque==0){
    semaforo_verde = digitalRead(PinSemaforo);  // HIGH si la Raspberry manda señal
    if (semaforo_verde==1)
      arranque=1;
  }

  if(arranque==1){
    posicion=lectura();
    PID();
    frenos();
  }

  senialpare = digitalRead(PinPare);  // HIGH si la Raspberry manda señal
  tiempo_actual = millis();

  if(senialpare==1 && stopflag==0){
    motores(0,0);
    delay(4000);
    tiempo_inicio=tiempo_actual;
    stopflag=1;
  }

  if(stopflag==1){
    tiempo_pasado = tiempo_actual - tiempo_inicio;
      if(tiempo_pasado >= 15000){
        stopflag=0;
      }
  }

  Serial.print(posicion);
  /*
  Serial.print("         proporcional:  ");
  Serial.print(proporcional);
  */
  Serial.println();

  Serial.print("                                 Valores[ui,I,C,D,ud]:    ");
  for (int i=0; i<5; i++){
    Serial.print(sensoresanalog[i]);
    Serial.print("\t");
  }
  Serial.print("          ");

  //Serial.print("       semaforo verde, arranque, senialpare, stopflag   "); Serial.print(semaforo_verde); Serial.print(arranque); Serial.print(senialpare); Serial.print(stopflag);
  
}

void motores(int izq, int der){//0 hasta 255    0 hasta -255
  izq=constrain(izq,-255,255);
  der=constrain(der,-255,255);
  ////////////////motor LEFT "IZQUIERDO" ////////////////////////
  if(izq>=0){
    digitalWrite(PinIN2,HIGH);
    digitalWrite(PinIN1,LOW);
    analogWrite(PinIN2,izq);
  }
  else{
    digitalWrite(PinIN2,LOW);
    digitalWrite(PinIN1,HIGH);
    izq=izq*(-1);
    analogWrite(PinIN1,izq);
  }

  ////////////////motor RIGHT "DERECHO" ////////////////////////
  if(der>=0){
    digitalWrite(PinIN3,HIGH);
    digitalWrite(PinIN4,LOW);
    analogWrite(PinIN3,der);
  }
  else{
    digitalWrite(PinIN3,LOW);
    digitalWrite(PinIN4,HIGH);
    der=der*(-1);
    analogWrite(PinIN4,der);
  }
  /*
  Serial.print("motores:    ");
  Serial.print(izq); Serial.print("  "); Serial.print(der);
  Serial.println(" ");
  */
}


// Variables globales nuevas
int vel_min = 120;   // velocidad mínima    en 120 andaba bien, aunque un poco lento 
int vel_max = 180;   // velocidad máxima    en 220 andaba bien
int vel_base = vel_min;  // velocidad actual
int recto_counter = 0; // cuenta cuántos ciclos va recto

void PID(){
  proporcional = pos - setpoint;
  derivativo = proporcional - last_prop;
  integral = error1 + error2 + error3 + error4 + error5 + error6;
  last_prop = proporcional;

  // actualizar historial de errores para la integral
  error6 = error5;
  error5 = error4;
  error4 = error3;
  error3 = error2;
  error2 = error1;
  error1 = proporcional;

  int diferencial = (proporcional*KP) + (derivativo*KD) + (integral*KI);

  // --- Control de velocidad según si va recto o en curva ---
  if (abs(proporcional) < 60) {  // probar menor a 110 para que no desacelere en la parte recta
    // casi centrado en la línea
    recto_counter++;
    if (recto_counter > 3 && vel_base < vel_max) { 
      vel_base += 20; // subir velocidad progresivamente en +=15 estaba bien
    }
  } else if (abs(proporcional) > 160)  {  // agregue este if para que no desacelere en la parte recta. probar con otro if de abs proporcional entre 60 y 160 que baje -=5
    recto_counter = 0;
    //vel_base -= 30;  // frenar rápido al entrar en curva   en -=5 estaba bien     bajarlo un poco para que no desacelere en la parte recta
    vel_base = vel_min;
    if (vel_base < vel_min) vel_base = vel_min;
  }
  else  if (abs(proporcional) >= 60 && abs(proporcional) <= 160){  // agregue este if para que no desacelere tanto en la parte recta. prueba con if de abs proporcional entre 60 y 160 que baje -=5
    recto_counter = 0;
    vel_base -= 5; 
    if (vel_base < vel_min) vel_base = vel_min;
  }


  // --- Aplicar corrección con la velocidad adaptada ---
  if(diferencial < 0)
    motores(vel_base, vel_base + diferencial);
  else
    motores(vel_base - diferencial, vel_base);
}


int lectura(void){
  int val_izq = analogRead(Pin_sensor_izq);
  int val_cen = analogRead(Pin_sensor_cen);
  int val_der = analogRead(Pin_sensor_der);
  int val_uder = analogRead(Pin_sensor_uder);
  int val_uizq = analogRead(Pin_sensor_uizq);

  sensoresanalog[0] = val_uizq; sensoresanalog[1] = val_izq; sensoresanalog[2] = val_cen; sensoresanalog[3] = val_der; sensoresanalog[4] = val_uder;

  bool negro_izq = val_izq > 120;
  bool negro_cen = val_cen > 120;
  bool negro_der = val_der > 120;
  bool negro_uder = val_uder > UMBRAL;
  bool negro_uizq = val_uizq > UMBRAL;

  digital[0] = negro_uizq; digital[1] = negro_izq; digital[2] = negro_cen; digital[3] = negro_der; digital[4] = negro_uder;
  sumap=400*digital[0]+300*digital[1]+200*digital[2]+100*digital[3]+0*digital[4];
  suma=digital[0]+digital[1]+digital[2]+digital[3]+digital[4];
  pos=(sumap/suma);
  
  
  if ((digital[0]==1 && digital[1]==0 && (digital[2]+digital[3]+digital[4])>0) || (digital[4]==1 && digital[3]==0 && (digital[2]+digital[1]+digital[0])>0) || (digital[0]==1 && digital[1]==1 && digital[2]==0 && (digital[3] + digital[4])>0) || (digital[4]==1 && digital[3]==1 && digital[2]==0 && (digital[1] + digital[0])>0) )
    {pos=poslast; // ignorar medicion pues se esta pisando una de las líneas externas al circuito.
    Serial.print("     linea externa pisada   ");
    }
  if(poslast<=60 && suma==0){
    pos=0;
  }
  if(poslast>=340 && suma==0){
    pos=400;
  }
  poslast=pos;
  return pos; 
}

int veladelante=170, velatras=80;
void frenos(){
  if(pos<=40){  // lo cambie, antes estaba este en 60 y el otro en 340
    motores(veladelante, -velatras);
    Serial.print("     FRENO DER   ");
  }
  if(pos>=360){
    motores(-velatras, veladelante);
    Serial.print("     FRENO IZQ   ");
  }
}
