/*
 * ARDUINO 2 - SISTEMA DE CONTROLE PARA TANQUE 2
 * 
 * Funcionalidades:
 * 1. Leitura do sensor de nível MPX5010DP
 * 2. Comunicação Modbus RTU com SCADA Elipse E3
 * 3. Recepção dos dados do Arduino 1 via TTL
 * 4. Controle PWM da bomba (Desligado/Manual/PID/Realimentação Linearizante)
 */

/*******************************
 * SEÇÃO 1: BIBLIOTECAS E DEFINIÇÕES
 *******************************/
#include <ModbusRTU.h>      // Para comunicação Modbus
#include <SoftwareSerial.h> // Para porta serial adicional
#include <PID_v1_bc.h>      // Para controle PID

// Definição dos pinos de hardware
#define RX_PIN 3            // Pino RX para comunicação RS485
#define TX_PIN 4            // Pino TX para comunicação RS485
#define DE_RE_PIN 2         // Pino de controle DE/RE para RS485
#define RPWM 10             // Pino PWM para rotação positiva da bomba
#define LPWM 9              // Pino PWM para rotação negativa da bomba
#define R_EN 8              // Pino enable para rotação positiva
#define L_EN 7              // Pino enable para rotação negativa
const int sensorPin = A0;   // Pino analógico para o sensor

/*******************************
 * SEÇÃO 2: PARÂMETROS DO SENSOR DE NÍVEL
 *******************************/
// Constantes de calibração do sensor
const float Vcc = 5.0;          // Tensão de alimentação do sensor
const float offset = 0.2;       // Tensão de offset do sensor
const float sensitivity = 0.45; // Sensibilidade (V/kPa)
const float nivel_morto = 2.5;  // Nível morto do tanque em cm

/*******************************
 * SEÇÃO 3: VARIÁVEIS GLOBAIS
 *******************************/
SoftwareSerial rs485(RX_PIN, TX_PIN); // Objeto para comunicação RS485
ModbusRTU mb;                         // Objeto Modbus RTU

// Variáveis de estado do sistema
double h_cm = 0;                     // Nível no tanque (cm)
float h_cm_tanque1 = 0;              // Armazena nível do Tanque 1 (cm)
uint16_t currentPWM = 0;             // Valor atual de PWM aplicado à bomba
double PumpInput = 0;

// Limites do PWM PID
double pwm_min = -0.105;             // Mínimo valor de PWM
double pwm_max = 0.895;              // Máximo valor de PWM

// Limites do PWM RL
double pwm_min_RL = 0.0;             // Mínimo valor de PWM (0%)
double pwm_max_RL = 1.0;             // Máximo valor de PWM (100%)

// Variáveis de Controle PID (modo 2)
double setpoint = 0.0;
double last_setpoint = 0.0;               
double Kp = 0.04605;                 // Ganho proporcional
double Ki = 0.00029;                 // Ganho integral
double Kd = 0.0;                     // Ganho derivativo
double pidOutput = 0.0;              // Saída do PID

// Variáveis do Controle por Realimentação Linearizante (modo 3)
double Kp_RL = 0.0520308300;                // Ganho proporcional
double Ki_RL = 0.0001038827;                   // Ganho integral

// Variáveis usadas no modo 3
double erro = 0;
double sigma = 0;
double dt = 1.0;                     // intervalo de tempo em segundos
double termo1 = 0.0;
double termo2 = 0.0;
double termo3 = 0.0;

//Parâmetros do sistema
          
double cv1 = 15.4887;                 // Coef. de vazão
double cv2 = 13.5643;                 // Coef. de vazão
double k2 = 224.2262;                 // Constante da bomba
double h_lat = 0.9386;               // Altura lateral
double b1 = 8.4;
double b2 = 20.0;
double H = 48.60;
double alpha = b1;
double beta = (b2-b1)/H;
double AREA = 0; // Área do tanque 2 (cm²)

// Controlador PID
PID myPID(&h_cm, &pidOutput, &setpoint, Kp, Ki, Kd, DIRECT);

/*******************************
 * SEÇÃO 4: CONFIGURAÇÃO INICIAL
 *******************************/
void setup() {
  // Inicialização da comunicação serial
  Serial.begin(9600);               // Para comunicação com Arduino 1
  rs485.begin(9600);                // Para comunicação Modbus
  
  // Configuração do PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(pwm_min, pwm_max);
  myPID.SetSampleTime(1000);

  // Configuração do protocolo Modbus
  mb.begin(&rs485, DE_RE_PIN);      // Inicializa Modbus
  mb.slave(2);                      // Define ID deste dispositivo como 2
  
  // Configuração dos registradores Modbus
  mb.addHreg(0, 0);                 // Registrador 0: Nível do tanque
  mb.addHreg(1, 0);                 // Registrador 1: Valor PWM manual (0-255)
  mb.addHreg(2, 0);                 // Registrador 2: Feedback do PWM atual
  mb.addHreg(3, 0);                 // Registrador 3: Nível do Tanque 1
  mb.addHreg(4, 0);                 // Registrador 4: Modo de controle (0=Desligado, 1=Manual, 2=PID, 3=Realimentação Linearizante)
  mb.addHreg(5, 0);                 // Registrador 5: Setpoint de nível
  mb.addHreg(6, 0);                 // Registrador 6: Feedback Setpoint de nível
  
  // Inicia em modo Desligado
  mb.Hreg(4, 0);

  // Configuração dos pinos da bomba
  pinMode(RPWM, OUTPUT);            // Configura pino PWM positivo
  pinMode(LPWM, OUTPUT);            // Configura pino PWM negativo
  pinMode(R_EN, OUTPUT);            // Configura enable positivo
  pinMode(L_EN, OUTPUT);            // Configura enable negativo
  digitalWrite(R_EN, HIGH);         // Ativa enable positivo
  digitalWrite(L_EN, HIGH);         // Ativa enable negativo
  
  // Configuração do pino DE/RE para RS485
  pinMode(DE_RE_PIN, OUTPUT);       // Configura como saída
  digitalWrite(DE_RE_PIN, LOW);     // Inicia em modo recepção
}

/*******************************
 * SEÇÃO 5: LOOP PRINCIPAL
 *******************************/
void loop() {
  // 1. Processa requisições Modbus
  mb.task();
  
  // 2. Atualiza leitura do sensor
  updateSensor();
  
  // 3. Controla a bomba (Manual/PID/Realimentação Linearizante)
  controlPump();

   // 4. Comunicação com Arduino 1
  receiveTanque1Data();
  
}

/*******************************
 * SEÇÃO 6: FUNÇÕES DO SISTEMA
 *******************************/

/**
 * Atualiza a leitura do sensor a cada 1 segundo
 */
void updateSensor() {
  static unsigned long lastRead = 0;
  
  if (millis() - lastRead > 1000) {
    lastRead = millis();
    
    int rawValue = analogRead(sensorPin);
    float voltage = rawValue * (Vcc / 1023.0);
    float pressure = (voltage - offset) / sensitivity;
    h_cm = (pressure * 100.0) / 9.81;       // kPa para cmH2O
    h_cm = max(0.0, h_cm - nivel_morto);
    
    // Atualiza registrador Modbus (cm × 10)
    mb.Hreg(0, (uint16_t)(h_cm * 10));
  }
}

/**
 * Controla a bomba conforme o modo selecionado
 */
void controlPump() {
  uint16_t mode = mb.Hreg(4);  // Lê o modo de controle

  switch (mode) {
    case 0: // Modo desligado
      currentPWM = 0;
      break;

    case 1: // Modo Manual
      PumpInput = mb.Hreg(1);
      currentPWM = (uint16_t)((PumpInput/100.0)*95 + 160); // 160-255
      break;

    case 2: // Modo PID
      setpoint = (double)mb.Hreg(5); // Lê setpoint em cm
      myPID.Compute();
      
      if (setpoint == 0) {
        currentPWM = 160;
      } else {
        currentPWM = (uint16_t)(95*(pidOutput + 0.105) + 160); // 160-255
      }
      break;

    case 3: // Modo Realimentação Linearizante
      static unsigned long lastControl = 0;
      
      
      if (millis() - lastControl < 1000) return;
      lastControl = millis();

      AREA = pow((alpha +beta*h_cm),2);

      setpoint = (double)(mb.Hreg(5));

      if (last_setpoint != setpoint){
        sigma = 0.0;
      } else {
        sigma += erro * dt;
      }

      last_setpoint = setpoint;
      erro = h_cm - setpoint;

      termo1 = (cv2 * sqrt(h_cm) - cv1 * sqrt(max(h_cm_tanque1 - h_lat, 0))) / AREA;
      termo2 = Kp_RL * erro;
      termo3 = Ki_RL * sigma;

      pidOutput = (AREA / k2) * (termo1 - termo2 - termo3);

      if (pidOutput > pwm_max_RL) {
        pidOutput = pwm_max_RL;
        sigma -= erro * dt;  // anti-windup
      } else if (pidOutput < pwm_min_RL) {
        pidOutput = pwm_min_RL;
        sigma -= erro * dt;  // anti-windup
      }

      if (setpoint == 0) {
        currentPWM = 160;
      } else {
        currentPWM = (uint16_t)(95 * pidOutput + 160); // 160-255
      }
      break;

    default:
      // Se for um modo inválido, desliga a bomba
      currentPWM = 0;
      break;
  }

  // Aplica o PWM
  analogWrite(RPWM, currentPWM);
  analogWrite(LPWM, 0);

  // Atualiza registradores de feedback
  mb.Hreg(2, currentPWM);
  mb.Hreg(6, (uint16_t)(setpoint));
}

/**
 * Recebe dados do Arduino 1 via TTL (formato: 2 bytes big-endian)
 */
void receiveTanque1Data() {
  if (Serial.available() >= 2) {
    h_cm_tanque1 = ((Serial.read() << 8) | Serial.read()) / 10.0; // Converte para cm
    mb.Hreg(3, (uint16_t)(h_cm_tanque1 * 10));  // Atualiza registrador
  }
}