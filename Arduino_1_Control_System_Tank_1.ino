/*
 * ARDUINO 1 - SISTEMA DE CONTROLE PARA TANQUE 1
 * 
 * Funcionalidades principais:
 * 1. Leitura dos sensores de nível MPX5010DP e vazão FS300A
 * 2. Comunicação Modbus RTU com SCADA Elipse E3
 * 3. Comunicação serial TTL com Arduino 2
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
#define DE_RE_PIN 13        // Pino de controle DE/RE para RS485
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
const float nivel_morto = 3.76; // Nível morto do tanque em cm

/*******************************
 * SEÇÃO 3: VARIÁVEIS GLOBAIS
 *******************************/
SoftwareSerial rs485(RX_PIN, TX_PIN); // Objeto para comunicação RS485
ModbusRTU mb;                         // Objeto Modbus RTU

// Variáveis de estado do sistema
double h_cm = 0;                     // Nível no tanque (cm)
uint16_t currentPWM = 0;             // Valor atual de PWM aplicado à bomba
double PumpInput = 0;
// Limites do PWM PID
double pwm_min = -0.5;                // Mínimo valor de PWM (0%)
double pwm_max = 0.5;                 // Máximo valor de PWM (100%)
// Limites do PWM RL
double pwm_min_RL = 0.0;                // Mínimo valor de PWM (0%)
double pwm_max_RL = 1.0;                 // Máximo valor de PWM (100%)

// Parâmetros de Controle PID (modo 2)
double setpoint = 0.0;
double last_setpoint = 0.0;               
double Kp = 0.03863;                // Ganho proporcional
double Ki = 0.00051;                // Ganho integral
double Kd = 0.0;                    // Ganho derivativo
double pidOutput = 0.0;             // Saída do PID

// Parâmetros do Controle por Realimentação Linearizante (modo 3)
double Kp_RL = 0.0246980792; // Ganho proporcional
double Ki_RL = 0.0000065882; // Ganho integral


// Variáveis usadas no Controle por Realimentação Linearizante (modo 3)
double erro = 0;
double sigma = 0;
double dt = 1.0;  // intervalo de tempo em segundos
double termo1 = 0.0;
double termo2 = 0.0;
double termo3 = 0.0;

//Parâmetros do sistema
double AREA = 19.5 * 19.5;         // Área do tanque 1 (cm²)
double cv1 = 15.4887;              // Coef. de vazão
double cv3 = 21.7436;              // Coef. de vazão
double k1 = 220.0513;             // Constante da bomba

/*******************************
 * SEÇÃO 2: PARÂMETROS DO SENSOR DE VAZÃO
 *******************************/
volatile int flowPulseCount = 0;
float flowRate = 0.0;
unsigned long lastFlowCalcTime = 0;
const byte FLOW_SENSOR_PIN = 2;

void IRAM_ATTR countFlowPulse() {
  flowPulseCount++;
}

// Controlador PID
PID myPID(&h_cm, &pidOutput, &setpoint, Kp, Ki, Kd, DIRECT);

/*******************************
 * SEÇÃO 4: CONFIGURAÇÃO INICIAL
 *******************************/
void setup() {
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), countFlowPulse, RISING);
  // Inicialização da comunicação serial
  Serial.begin(9600);               // Para comunicação com Arduino 2
  rs485.begin(9600);                // Para comunicação Modbus
  
  // Configuração do PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(pwm_min, pwm_max);
  myPID.SetSampleTime(1000);

  // Configuração do protocolo Modbus
  mb.begin(&rs485, DE_RE_PIN);      // Inicializa Modbus
  mb.slave(1);                      // Define ID deste dispositivo como 1
  
  // Configuração dos registradores Modbus
  mb.addHreg(0, 0);                 // Registrador 0: Nível do tanque
  mb.addHreg(1, 0);                 // Registrador 1: Valor PWM manual (0-255)
  mb.addHreg(2, 0);                 // Registrador 2: Feedback do PWM atual
  mb.addHreg(3, 0);                 // Registrador 3: Modo de controle (0=Desligado, 1=Manual, 2=PID, 3=Realimentação Linearizante)
  mb.addHreg(4, 0);                 // Registrador 4: Setpoint de nível
  mb.addHreg(5, 0);                 // Registrador 5: Feedback Setpoint de nível
  mb.addHreg(6, 0);                 // Registrador 6: Vazão
  // Inicia em modo Desligado
  mb.Hreg(3, 0);

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
  
  // 2. Atualiza leitura do sensor de nível
  updateSensor();

  // 3. Atualiza leitura do sensor de vazão
  atualizaVazao();
  
  // 4. Controla a bomba (Manual/PID/Realimentação Linearizante)
  controlPump();
  
  // 5. Comunicação com Arduino 2
  sendToArduino2();
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
    h_cm = max(0,h_cm - nivel_morto);
    // Atualiza registrador Modbus (cm × 10)
    mb.Hreg(0, (uint16_t)(h_cm * 10));
  }
}

/**
 * Controla a bomba conforme o modo selecionado (Manual/PID/Realimentação Linearizante)
 */
void controlPump() {
  uint16_t mode = mb.Hreg(3);  // Lê o modo de controle

  switch (mode) {
    case 0: // Modo desligado
      currentPWM = 0;
      break;

    case 1: // Modo Manual
      PumpInput = mb.Hreg(1);
      currentPWM = (uint16_t)((PumpInput / 100.0) * 80 + 175);
      break;

    case 2: // Modo PID
      setpoint = (double)(mb.Hreg(4));
      myPID.Compute();

      if (setpoint == 0) {
        currentPWM = 175;
      } else {
        currentPWM = (uint16_t)(80 * (pidOutput + 0.5) + 175);
      }
      break;

    case 3: // Modo Realimentação Linearizante

      static unsigned long lastControl = 0;

      if (millis() - lastControl < 1000) return;
      lastControl = millis();

      setpoint = (double)(mb.Hreg(4));

      if (last_setpoint != setpoint){
        sigma = 0.0;
      }else {
        sigma += erro * dt;
    }

      last_setpoint = setpoint;
      erro = h_cm - setpoint;
      sigma += erro * dt;
      termo1 = (cv3 * sqrt(h_cm) + cv1 * sqrt(max(h_cm - h_lat, 0))) / AREA;
      termo2 = Kp_RL * erro;
      termo3 = Ki_RL * sigma;

      pidOutput = (AREA / k1) * (termo1 - termo2 - termo3);

      if (pidOutput > pwm_max_RL) {
        pidOutput = pwm_max_RL;
        sigma -= erro * dt;  // anti-windup
      } else if (pidOutput < pwm_min_RL) {
        pidOutput = pwm_min_RL;
        sigma -= erro * dt;  // anti-windup
      }

      if (setpoint == 0) {
        currentPWM = 175;
      }else {
        currentPWM = (uint16_t)(80 * (pidOutput) + 175);
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
  mb.Hreg(5, (uint16_t)(setpoint));
}

/**
 * Envia dados para Arduino 2 via serial (altura em cm × 10)
 */
void sendToArduino2() {
  static unsigned long lastSend = 0;
  
  if (millis() - lastSend > 1000) {
    lastSend = millis();
    
    uint16_t heightToSend = (uint16_t)(h_cm * 10);
    Serial.write(heightToSend >> 8);    // MSB
    Serial.write(heightToSend & 0xFF);  // LSB
  }
}

void atualizaVazao() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastFlowCalcTime >= 1000) { // Executa a cada 1 segundo
    noInterrupts();
    int pulseCount = flowPulseCount;
    flowPulseCount = 0;
    interrupts();

    // Cálculo da vazão
    flowRate = pulseCount / 5.5;

    // Armazena em registrador Modbus
    mb.Hreg(6, (uint16_t)(flowRate * 100)); // Ex: 3.25 L/min → grava 325

    lastFlowCalcTime = currentMillis;
  }
}