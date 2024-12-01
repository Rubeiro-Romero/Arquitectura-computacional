/*
JAVIER ALEXIS CERON ANACONA
104621021203
RUBEIRO ROMERO
104616020665
JAVIER MAURICIO SOLANO PAEZ
104622020724
*/
#include "StateMachineLib.h"
#include "AsyncTaskLib.h"
#include "DHT.h"
#include <LiquidCrystal.h>
#include <Keypad.h>

//CR7 en la casa
//***************************************Configuracion del RGB_LED****
const int RED_PIN = 10; 
//*******************************************************************
const byte ROWS = 4; //four rows
const byte COLS = 3; //three columns
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte rowPins[ROWS] = {24, 26, 28, 30}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {32, 34, 36}; //connect to the column pinouts of the keypad

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);



//#define TEMP_DIGITAL

#define DHTPIN 46     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);


#define DEBUG(a) Serial.print(millis()); Serial.print(": "); Serial.println(a);

#define analogPin A0 //the thermistor attach to
#define beta 4090 //the beta of the thermistor

float tempC = 0.0;
//*****Sensor de LUZ************************************
#define LDR_PIN A1
float lux=0.0;
//*********************************************************
//*****Configuracion Buzzer*********************************


const int BUZZER_PIN = 6; // Pin donde está conectado el buzzer}
#define NOTE_C5  523 // Do (octava superior)


//**********************************************************
// State Alias
enum State
{
	INICIO = 0,
	BLOQUEADO = 1,
	MONITOREO = 2,
	EVENTOS = 3,
  ALARMA = 4
};

// Input Alias
enum Input
{
	Sign_T = 0,
	Sign_P = 1,
	Sign_S = 2,
	Unknown = 3,
};

// Create new StateMachine
StateMachine stateMachine(5, 8);

// Stores last user input
Input input;


void read_Temperatura(void){

	#if defined TEMP_DIGITAL
		Serial.println("TEMP_DIGITAL");
		// Read temperature as Celsius (the default)
		tempC = dht.readTemperature();
	#else
		Serial.println("TEMP_ANALOG");
		//read thermistor value
		long a =1023 - analogRead(analogPin);
		//the calculating formula of temperature
		tempC = beta /(log((1025.0 * 10 / a - 10) / 10) + beta / 298.0) - 273.0;
	#endif

		Serial.print("TEMP: ");
		Serial.println(tempC);
		if(tempC > 42){
			input = Input::Sign_P;
		}
}

float Hum;
void read_Humedad(void){
  Hum = dht.readHumidity(); 
}






#define PIN_IR 48 //Pinndel Infra ROjo
#define PIN_HALL 50 //Pin del 

unsigned char valIR = HIGH;
unsigned char valHall = LOW;

void read_Sensores(void){
  valIR = digitalRead(PIN_IR);//sensor pir
  valHall = digitalRead(PIN_HALL);
	lcd.setCursor(0, 1);
	lcd.print("IR: ");
	lcd.print(valIR);

	lcd.setCursor(7, 1);
	lcd.print("Hall: ");
	lcd.print(valHall);
  //|| (valHall == HIGH)
	if((valIR == LOW) ){
		input = Input::Sign_S;
    lcd.print("hola123");
    delay(3000);
	}
}

const float GAMMA = 0.7;
const float RL10 = 50;
void leerLuz(void){
	int analogValue = analogRead(LDR_PIN);
  float voltage = analogValue / 1024. * 5;
  float resistance = 2000 * voltage / (1 - voltage / 5);
  lux = pow(RL10 * 1e3 * pow(10, GAMMA) / resistance, (1 / GAMMA));
}

char key;
void funct_keypad(void){
	key = keypad.getKey();
}
unsigned long tiempoInicio = 0;     // Marca de tiempo de inicio
bool tiempoEnProceso = false;       // Indica si el conteo está activo
bool tiempoCumplido = false;        // Indica si el tiempo ya se cumplió

unsigned long duracion = 0; // Duración en milisegundos (3 segundos)
// Función para manejar el conteo del temporizador
void contarTiempo() {
    if (!tiempoEnProceso) {
        tiempoInicio = millis();  // Registrar el tiempo de inicio
        tiempoEnProceso = true;   // Iniciar el conteo
        tiempoCumplido = false;   // Reiniciar el estado de tiempo cumplido
    }
		unsigned long tiempoActual = millis() - tiempoInicio;
    if (tiempoActual >= duracion) {
        tiempoEnProceso = false;  // Detener el conteo
        tiempoCumplido = true;    // Marcar que el tiempo se cumplió
    }
}

// Función para verificar y realizar una acción al cumplirse el tiempo
void verificarTiempoCumplido() {
    if (tiempoCumplido) {
				input = Input::Sign_T;
        delay(500);
        //tiempoCumplido = true;          // Resetear el estado
    }
}

// Tareas asincrónicas
AsyncTask TaskTemporizador(100, true, contarTiempo); // Ejecuta contarTiempo cada 50 ms
AsyncTask TaskVerificar(100, true, verificarTiempoCumplido); // Verifica el estado cada 100 ms

void encenderBuzzer(void){
	if(key == '#' || (input == Input::Sign_T)){
		tiempoCumplido=false;
		input = Input::Sign_T;
		noTone(BUZZER_PIN);
	}else{
		tone(BUZZER_PIN, 1000);
	}
}
void darDelay(int sec){
	int DelayMills=sec*1000;
	delay(DelayMills);
}
void funct_Display(void){
	lcd.clear();
  // print the number of seconds since reset:
  lcd.setCursor(0, 0);
	lcd.print("T: ");
	lcd.print(tempC);

	lcd.setCursor(7, 0);
	lcd.print(" ");
	lcd.print("H: ");
	lcd.print(Hum);
	
	lcd.setCursor(0, 1);
	lcd.print("LUZ: ");
	lcd.print(lux);

}
// Función para encender el LED RGB en rojo
void encenderRojo() {
    analogWrite(RED_PIN, 255);   // Enciende el rojo al máximo
    //analogWrite(GREEN_PIN, 0);   // Apaga el verde
    //analogWrite(BLUE_PIN, 0);    // Apaga el azul
}

// Función para apagar el LED RGB
void apagarLED() {
    analogWrite(RED_PIN, 0);      // Apaga el rojo
    //analogWrite(GREEN_PIN, 0);    // Apaga el verde
    //analogWrite(BLUE_PIN, 0);     // Apaga el azul
}


unsigned long permanenciaLedMillis = 0; // Tiempo a contar en millis
unsigned long tiempoInicioLed; // Variable para almacenar el tiempo de inicio
bool tiempoCompletoLed = false; // Variable para indicar si el tiempo se ha completado
void encenderLed(void){
	//Serial.println(permanenciaLedMillis);
	if (contarMillisLed(permanenciaLedMillis)) {
		// Si el tiempo se completó, envía un mensaje de retorno
		//Serial.println("Tiempo completo!");
		// Resetea el estado para permitir otro conteo si es necesario
		apagarLED();
		permanenciaLedMillis = 0;
  }
}
bool contarMillisLed(unsigned long miliSeg) {
    // Si no hemos iniciado el conteo, lo hacemos ahora
    if (!tiempoCompletoLed) {
        tiempoInicioLed = millis(); // Guarda el tiempo actual
        tiempoCompletoLed = true;   // Cambia el estado a completo
    }
    // Verifica si ha pasado el tiempo especificado
    if (millis() - tiempoInicioLed >= miliSeg) {
				//Serial.println(millis() - tiempoInicioLed);
        tiempoCompletoLed = false; // Resetea el estado para el próximo conteo
        return true; // Retorna verdadero indicando que el tiempo se ha completado
    }

    return false; // Retorna falso si el tiempo aún no se ha completado
}
String contrasenia="9";
int limitIntentos=3;
int intentos =0;
String contrasenaLeida = "";
// Tarea sincrónica para validar la contraseña cuando sea necesario

void monitorearContrasena() {
  if (keypad.getKey() == '#') {
    validarContrasena();
  }
}
// Función para manejar la entrada del teclado
void leerTeclado() {
	key = keypad.getKey();
  if (key) {
    if (key == '#') { // Validar contraseña cuando se presiona '#'
      validarContrasena();
    } else if (key == '*') { // Borrar entrada al presionar '*'
      contrasenaLeida = "";
      lcd.setCursor(0, 1);
      lcd.print("        "); // Borrar los asteriscos del LCD
      lcd.setCursor(0, 1); // Resetear el cursor a la posición inicial
    } else if (key >= '0' && key <= '9') { // Agregar dígitos numéricos
      if (contrasenaLeida.length() < contrasenia.length()) { // Limitar a la longitud de la contraseña
        contrasenaLeida += key; // Agregar carácter
        lcd.print('*'); // Mostrar asterisco
      }
    }
  }
}

// Función para validar la contraseña
void validarContrasena () {
  if (contrasenaLeida == contrasenia) { // Comparar con la contraseña predefinida
    lcd.clear();
    lcd.print("Clave Correcta");
    Serial.println("Clave Correcta");
    delay(1000);
    // Reiniciar
    contrasenaLeida = "";
		intentos =0;
		key="";
		input = Input::Sign_P;
  } else {
    intentos++;
    lcd.clear();
    lcd.print("Clave Incorrecta");
    Serial.println("Clave Incorrecta");
    delay(1000);

    if (intentos == limitIntentos) { // Bloquear tras demasiados intentos fallidos
      lcd.clear();
      lcd.print("Bloqueado");
      input = Input::Sign_S;
    } else {
      contrasenaLeida = "";
      lcd.clear();
      lcd.print("Dijite Clave:");
      lcd.setCursor(0, 1);
    }
  }

}

//***************************************************
AsyncTask TaskPassword(100, true, monitorearContrasena);
AsyncTask TaskLed(100, true, encenderLed);
AsyncTask TaskLuz(100, true, leerLuz);
AsyncTask TaskHumedad(100, true, read_Humedad);
AsyncTask TaskTemperatura(1500, true, read_Temperatura);
AsyncTask TaskDisplay(500, true, funct_Display);
AsyncTask TaskSensor(1000, true, read_Sensores);
AsyncTask TaskKeypad(100, true, funct_keypad);
AsyncTask TaskKeypadSeguridad(50, true, leerTeclado);
AsyncTask TaskBuzzer(100, true, encenderBuzzer);

// Setup the State Machine
void setupStateMachine()
{
	// Add transitions
	stateMachine.AddTransition(INICIO, MONITOREO, []() { return input == Sign_P; });

	stateMachine.AddTransition(INICIO, BLOQUEADO, []() { return input == Sign_S; });
	stateMachine.AddTransition(BLOQUEADO, INICIO, []() { return input == Sign_T; });
	stateMachine.AddTransition(MONITOREO, EVENTOS, []() { return input == Sign_T; });

	stateMachine.AddTransition(EVENTOS, MONITOREO, []() { return input == Sign_T; });
	stateMachine.AddTransition(EVENTOS, ALARMA, []() { return input == Sign_S; });
	stateMachine.AddTransition(MONITOREO, ALARMA, []() { return input == Sign_P; });

	stateMachine.AddTransition(ALARMA, INICIO, []() { return input == Sign_T; });

	// Add actions
	stateMachine.SetOnEntering(INICIO, funct_Inicio);
	stateMachine.SetOnEntering(BLOQUEADO, funct_Bloqueado);
	stateMachine.SetOnEntering(MONITOREO, funct_Monitoreo);
	stateMachine.SetOnEntering(EVENTOS, funct_Eventos);
  stateMachine.SetOnEntering(ALARMA, funct_Alarma);

	stateMachine.SetOnLeaving(INICIO, funct_out_Inicio);
	stateMachine.SetOnLeaving(BLOQUEADO, funct_out_Bloqueado);
	stateMachine.SetOnLeaving(MONITOREO, funt_out_Monitoreo);
	stateMachine.SetOnLeaving(EVENTOS, funct_out_Eventos);
  stateMachine.SetOnLeaving(ALARMA, funct_out_Alarma);
}
//******************** VARIABLES NECESAIAS PARA ELÑ KEYPAD***********************

void funct_Inicio(void) {
 intentos =0;
 contrasenaLeida = "";
	key = "";  // Limpiar tecla previa
  contrasenaLeida = "";
  Serial.println("INICIO");
	TaskKeypadSeguridad.Start();
  TaskPassword.Start();
  // Configurar el LCD para mostrar el mensaje inicial
  lcd.clear();
  lcd.print("Dijite Clave:");
  lcd.setCursor(0, 1); // Colocar el cursor donde comienzan los asteriscos
  // Iniciar la tarea asincrónica para monitorear el teclado
  
	//Serial.print("PASO");
}


void funct_out_Inicio(void){
	Serial.println("Leaving INICIO");
	key = "";
	lcd.clear();
	TaskKeypadSeguridad.Stop();
	TaskPassword.Stop();
}

void funct_Bloqueado(void){
  key = "";
	tiempoCompletoLed=false;
	Serial.println("BLOQUEADO");
	lcd.print("BLOQUEADO");
	//TaskBuzzer.Start();
	//tiempoParaTransicion=7000;
	
	duracion=7000;
	TaskTemporizador.Start();
	TaskVerificar.Start();
	permanenciaLedMillis=500;//500 MS
	encenderRojo();
	//delay(1000);
	
	TaskLed.Start();
	TaskBuzzer.Start();
}

void funct_out_Bloqueado(void){
	Serial.println("Leaving BLOQUEADO");
    tiempoCumplido = false;
    TaskLed.Stop();
    TaskTemporizador.Stop();
    TaskVerificar.Stop();

    // Reiniciar tareas del teclado y contraseñas
    key = "";
    contrasenaLeida = "";
    TaskKeypadSeguridad.Start();
    TaskPassword.Start();
    TaskBuzzer.Stop();
	//*************
}

void funct_Monitoreo(void){
	lcd.clear();
	Serial.println("MONITOREO");
	lcd.print("MONITOREO");
	//tiempoCumplido=false;
	duracion=5000;
	TaskTemporizador.Start();
	TaskVerificar.Start();
	//*******************************
	TaskTemperatura.Start();
	TaskHumedad.Start();
	TaskLuz.Start();
	TaskDisplay.Start();
	//TaskTimeout.Start();
}

void funt_out_Monitoreo(void){
	Serial.println("Leaving MONITOREO");
	
	TaskTemporizador.Stop();
  TaskVerificar.Stop();
	TaskTemperatura.Stop();
	TaskHumedad.Stop();
	TaskLuz.Stop();
	//TaskTimeout.Stop();
	tiempoCumplido = false;
  TaskDisplay.Stop();
	lcd.clear();
}

void funct_Eventos(void){
  valIR = HIGH;
  valHall = LOW;
tiempoCumplido=false;
	Serial.println("EVENTOS");
	//lcd.print("EVENTOS");
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("EVENTOS");
	//lcd.print("EVENTOS");

	duracion=3000;
	
	//TaskSensor.Start();
	TaskTemporizador.Start();
	TaskVerificar.Start();
	TaskSensor.Start();
	
	//TaskTimeout.Start();
	
}

void funct_out_Eventos(void){
	lcd.clear();
	Serial.println("Leaving EVENTOS");
		tiempoCumplido=false;
    TaskTemporizador.Stop();
    TaskVerificar.Stop();
		TaskSensor.Stop();
	//TaskTimeout.Stop();
}

void funct_Alarma(void){
	
	Serial.println("ALARMA");
	permanenciaLedMillis=150;
	TaskDisplay.Stop();
	key = "";
	TaskKeypad.Start();
	TaskBuzzer.Start();
	TaskLed.Start();
	lcd.clear();
  // print the number of seconds since reset:
  lcd.setCursor(0, 0);
	lcd.print("ALARMA");

}

void funct_out_Alarma(void){
	Serial.println("Leaving ALARMA");
	TaskKeypad.Stop();
  key = "";

	TaskLed.Stop();
	TaskBuzzer.Stop();
}


void setup() 
{
	// set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
	pinMode(PIN_IR, INPUT);
  pinMode(PIN_HALL, INPUT);
	pinMode(RED_PIN, OUTPUT);
  //pinMode(BLUE_PIN, OUTPUT);
	//pinMode(BUZZER_PIN, OUTPUT);

	Serial.begin(9600);

	lcd.print("Bienvenido");
	darDelay(1);
	

	Serial.println("Starting State Machine...");
	setupStateMachine();	
	Serial.println("Start Machine Started");

	lcd.clear();
  //lcd.print("Dijite Clave");

	
	dht.begin();
	// Initial state
	stateMachine.SetState(INICIO, false, true);
	TaskKeypadSeguridad.Start();
}

void loop() 
{
	
	// Read user input
	input = static_cast<Input>(readInput());
	TaskLed.Update();
	TaskTemporizador.Update();
	TaskVerificar.Update();
	TaskDisplay.Update();
	TaskSensor.Update();
	TaskKeypadSeguridad.Update();
	TaskKeypad.Update();
	TaskTemperatura.Update();
	TaskHumedad.Update();
	TaskLuz.Update();
	TaskBuzzer.Update();
	TaskTemporizador.Update();
	
	//TaskTimeout.Update();
	// Update State Machine
	stateMachine.Update();
  input = Input::Unknown;
}

// Auxiliar function that reads the user input
int readInput()
{
	Input currentInput = Input::Unknown;
	char incomingChar="";
	switch (incomingChar)
	{
		case 'P': currentInput = Input::Sign_P; 	break;
		case 'T': currentInput = Input::Sign_T; break;
		case 'S': currentInput = Input::Sign_S; break;
		default: break;
	}
	return currentInput;
}
