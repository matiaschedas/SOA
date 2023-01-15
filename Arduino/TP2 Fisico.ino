// C++ code
//
#include <Servo.h>
#include <SoftwareSerial.h>   // Incluimos la librería  SoftwareSerial  
SoftwareSerial BT(4,2); 
// Habilitacion de debug para la impresion por el puerto serial ...
//----------------------------------------------
#define SERIAL_DEBUG_ENABLED 0

#if SERIAL_DEBUG_ENABLED
  #define DebugPrint(str)\
      {\
        Serial.println(str);\
      }
#else
  #define DebugPrint(str)
#endif

#define DebugPrintEstado(estado,evento)\
      {\
        String est = estado;\
        String evt = evento;\
        String str;\
        str = "-----------------------------------------------------";\
        DebugPrint(str);\
        str = "EST-> [" + est + "]: " + "EVT-> [" + evt + "].";\
        DebugPrint(str);\
        str = "-----------------------------------------------------";\
        DebugPrint(str);\
      }
//----------------------------------------------

//----------------------------------------------


//----------------------------------------------
// Estado de un sensor ...
#define ESTADO_SENSOR_OK                            108
#define ESTADO_SENSOR_ERROR                         666
//----------------------------------------------

//----------------------------------------------
// Estado de un mensaje ...
#define MENSAJE_ENVIADO_OK                          10
#define MENSAJE_ENVIADO_ERROR                       666
//----------------------------------------------

// Otras constantes ....
//----------------------------------------------
#define FLEXION_MINIMA                   			620
#define FLEXION_MAXIMA                     			500
#define UMBRAL_DISTANCIA                    		1
#define UMBRAL_DIFERENCIA_TIMEOUT					150
#define MAX_CANT_SENSORES                           3
#define SENSOR_FLEXION                          	0
#define SENSOR_PROXIMIDAD							1 
#define SENSOR_BLUETOOTH              2	
#define PIN_SENSOR_FLEXION                      	A0
#define PIN_LED                               		13
#define TIEMPO_MAX_MILIS 							6000	// Medio segundo.
#define ABIERTO										45
#define CERRADO										0
#define SERVO_PIN									9
#define TIMER_ACTIVO								1
#define TIMER_INACTIVO								0
#define PIN_SENSOR_IR									7

//----------------------------------------------

//----------------------------------------------
struct stSensor
{
  int  pin;
  int  estado;
  long valor_actual;
  long valor_previo;
};
stSensor sensores[MAX_CANT_SENSORES];
//----------------------------------------------

enum estados          { ST_INICIO,  ST_REPOSO        , ST_ALIMENTANDO   ,ST_REPOSO_VACIO	, ST_ERROR                                                  } current_state;
String estados_s [] = {"ST_INICIO", "ST_REPOSO"      , "ST_ALIMENTANDO" ,"ST_REPOSO_VACIO"	, "ST_ERROR"                                                };

enum eventos          { EV_CONT,   EV_ALIMENTAR   ,   EV_PLATO_LLENO          , EV_LLENAR   , EV_LLENO	,EVTIEMPO_CUMPLIDO 	,EV_BLUETOOTH_PRENDIDO  	,EV_BLUETOOTH_APAGADO 	,EV_DESCONOCIDO  } new_event;
String eventos_s [] = {"EV_CONT", "EV_ALIMENTAR"  , "EV_PLATO_LLENO"        , "EV_LLENAR", "EV_LLENO", "EVTIEMPO_CUMPLIDO" ,"EV_BLUETOOTH_PRENDIDO" ,"EV_BLUETOOTH_APAGADO" ,"EV_DESCONOCIDO" };



#define MAX_ESTADOS 5
#define MAX_EVENTOS 8

typedef void (*transition)();

void init_()
{
  DebugPrintEstado(estados_s[current_state], eventos_s[new_event]);
  apagar_leds();
  descansar();
}

transition tabla_estado[MAX_ESTADOS][MAX_EVENTOS] =
{
      {init_    , error         , error            , error        , error            , none  	      , none                  , none 		                } , // state ST_INICIO
      {none     , alimentar		  , none		         , llenar	      , none             , none  		    , llenar                , none 	                  } , // state ST_REPOSO
      {none     , none	        , descansar        , llenar	      , none             , descansar    , none                  , none  		              } , // state ST_ALIMENTANDO
      {none     , none	        , none	           , none	        , detener_llenado  , none         , none                  , detener_llenado 			  } , // state ST_REPOSO_VACIO
      {error    , error         , error            , error        , error            , error        , error                 , error  			            }   // state ST_ERROR
      
     //EV_CONT  , EV_ALIMENTAR  , EV_PLATO_LLENO  , EV_LLENAR    , EV_LLENO 		,EVTIEMPO_CUMPLIDO ,EV_BLUETOOTH_PRENDIDO  	,EV_BLUETOOTH_APAGADO
};

bool timeout;
long lct;
bool temperatura;
int pos = 0;
int valor;
long cm = 0;
Servo servo_9;
int flag_esperar = TIMER_INACTIVO;
unsigned long tiempo_actual;
unsigned long tiempo_inicial;

//----------------------------------------------
void do_init()
{
  servo_9.attach(SERVO_PIN, 500, 2500);
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_SENSOR_IR, INPUT);
  BT.begin(9600);
  Serial.begin(9600);

  sensores[SENSOR_FLEXION].pin    = PIN_SENSOR_FLEXION;
  sensores[SENSOR_FLEXION].estado = ESTADO_SENSOR_OK;
  
  sensores[SENSOR_PROXIMIDAD].estado = ESTADO_SENSOR_OK;
  sensores[SENSOR_BLUETOOTH].estado = ESTADO_SENSOR_OK;
  
  // Inicializo el evento inicial
  current_state = ST_INICIO;
  
  timeout = false;
  lct     = millis();
}
//----------------------------------------------

//----------------------------------------------
long leer_sensor_flexion( )
{
  return analogRead(PIN_SENSOR_FLEXION);
}
//----------------------------------------------

//----------------------------------------------
//----------------------------------------------

//----------------------------------------------
char leer_bluetooth( )
{ char nada='-';
  if(BT.available())    // Si llega un dato por el puerto BT se envía al monitor serial
  {
    return BT.read();
  }
  return nada;
}
//----------------------------------------------

//----------------------------------------------
long leer_sensor_ir()
{
  return digitalRead(PIN_SENSOR_IR);
  
}
//----------------------------------------------

//----------------------------------------------
void apagar_leds( )
{
  digitalWrite(PIN_LED, false);

}
//----------------------------------------------

//----------------------------------------------
void mover_servo(int grados)
{
  servo_9.write(grados);

}
//----------------------------------------------

//----------------------------------------------

bool verificar_estado_sensor_flexion()
{
  
  sensores[SENSOR_FLEXION].valor_actual = leer_sensor_flexion( );
  
  int valor_actual = sensores[SENSOR_FLEXION].valor_actual;
  int valor_previo = sensores[SENSOR_FLEXION].valor_previo;
 
  
  if( valor_actual != valor_previo )
  {
    sensores[SENSOR_FLEXION].valor_previo = valor_actual;
    
    if( valor_actual > FLEXION_MINIMA && valor_previo < FLEXION_MINIMA )
    {
      new_event = EV_ALIMENTAR;
      
    }
    
    else if( valor_actual < FLEXION_MAXIMA )
    {
      new_event = EV_PLATO_LLENO;
      
    }
    
    return true;
  }
  
  return false;
}
//----------------------------------------------

//----------------------------------------------
bool verificar_estado_sensor_ir()
{
  
  sensores[SENSOR_PROXIMIDAD].valor_actual = leer_sensor_ir();
  int valor_actual = sensores[SENSOR_PROXIMIDAD].valor_actual;
  int valor_previo = sensores[SENSOR_PROXIMIDAD].valor_previo;
  
  
  if( valor_actual != valor_previo )
  {
    sensores[SENSOR_PROXIMIDAD].valor_previo = valor_actual;
       
    if( valor_actual == 0 )
    {
      new_event = EV_LLENO;
     
	  BT.print("LLENO: ");
    BT.println(valor_actual);
    }
    
    if( valor_actual == 1 )
    {
      new_event = EV_LLENAR;
	 
	  BT.print("LLENAR: ");
    BT.println(valor_actual);
    }
    
    return true;
  }
  
  return false;
}
//----------------------------------------------

//----------------------------------------------
bool verificar_bluetooth()
{
  
  sensores[SENSOR_BLUETOOTH].valor_actual = leer_bluetooth( );
  
  int valor_actual = sensores[SENSOR_BLUETOOTH].valor_actual;
  int valor_previo = sensores[SENSOR_BLUETOOTH].valor_previo;
  
  if( valor_actual !='-' && valor_actual != valor_previo )
  {
    sensores[SENSOR_BLUETOOTH].valor_previo = valor_actual;
    
    if( valor_actual=='p')
    {
      new_event =EV_BLUETOOTH_PRENDIDO;
    }
    
    else if(valor_actual =='a')
    {
      new_event =EV_BLUETOOTH_APAGADO;
    }
    
    return true;
  }
  
  return false;
}
//----------------------------------------------

//----------------------------------------------
bool verificar_tiempo_cumplido(){

  if ( flag_esperar == TIMER_ACTIVO )
  {
	tiempo_actual=millis();

	if( (tiempo_actual-tiempo_inicial) >= TIEMPO_MAX_MILIS )
	{
		new_event= EVTIEMPO_CUMPLIDO;
		flag_esperar = TIMER_INACTIVO;
      return true;
	}  
  }
  return false;
}
//----------------------------------------------

//----------------------------------------------
void obtener_nuevo_evento( )
{
      
   	if( verificar_estado_sensor_ir() || verificar_estado_sensor_flexion() || verificar_tiempo_cumplido() || verificar_bluetooth() )
    {
      return;
    }
  
  
  // Genero evento dummy ....
  new_event = EV_CONT;
}
//----------------------------------------------



void error()
{
}

void none()
{
}

void alimentar()
{
  servo_9.write(ABIERTO);
  esperar();
  current_state = ST_ALIMENTANDO;
}

void esperar()
{
  tiempo_inicial = millis();
  flag_esperar = TIMER_ACTIVO;
  
}

void descansar()
{
  servo_9.write(CERRADO);
  current_state = ST_REPOSO;
  flag_esperar = TIMER_INACTIVO;
}

void llenar()
{
  digitalWrite(PIN_LED,HIGH);
  servo_9.write(CERRADO);
  current_state = ST_REPOSO_VACIO;
}

void detener_llenado()
{
  digitalWrite(PIN_LED,LOW);
  current_state = ST_REPOSO;
}

//----------------------------------------------
void maquina_estados_dispenser_comida( )
{
  obtener_nuevo_evento();

  if( (new_event >= 0) && (new_event < MAX_EVENTOS) && (current_state >= 0) && (current_state < MAX_ESTADOS) )
  {
    if( new_event != EV_CONT )
    {
      DebugPrintEstado(estados_s[current_state], eventos_s[new_event]);
    }
    
    tabla_estado[current_state][new_event]();
  }
  else
  {
    DebugPrintEstado(estados_s[ST_ERROR], eventos_s[EV_DESCONOCIDO]);
  }
    
  // Consumo el evento...
  new_event   = EV_CONT;
}
//----------------------------------------------


// Funciones de arduino !. 
//----------------------------------------------
void setup()
{
  do_init();
}
//----------------------------------------------

//----------------------------------------------
void loop()
{ 
  maquina_estados_dispenser_comida();
}
//----------------------------------------------

