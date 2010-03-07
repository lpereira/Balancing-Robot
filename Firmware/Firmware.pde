/*
 * Sensor de Ângulo Microprocessado
 * Prova de Conceito: Plataforma Robótica Automaticamente Equilibrável
 *
 * Trabalho de Conclusão de Curso - 2009
 * Faculdade de Engenharia de Computação
 * Centro de Ciências Exatas, Ambientais e de Tecnologias
 * Pontifícia Universidade Católica de Campinas
 *
 *         Aluno: Leandro Augusto Fogolin Pereira <leandro@tia.mat.br>
 *            RA: 05356787
 *    Orientador: Prof. Carlos M. T. Toledo <tobar@puc-campinas.edu.br>
 * Co-Orientador: Prof. Gustavo E. A. P. A. Batista <gbatista@icmc.usp.br>
 */

/****************************************************************************************************************/
/** Includes ****************************************************************************************************/
/****************************************************************************************************************/

#include <avr/sleep.h>  /* Dormência do processador */
#include <Servo.h>      /* Controle dos motores */
#include <Wire.h>       /* Leitura do giroscópio (via I2C) */

/****************************************************************************************************************/
/** Constantes **************************************************************************************************/
/****************************************************************************************************************/

/* ID das tarefas */
enum {
  TASK_ID_BLINK,      /* Tarefa para piscar LED */
  TASK_ID_JOYSTICK,   /* Tarefa de leitura de joystick */
  TASK_ID_BALANCE,    /* Tarefa de equilíbrio: maior prioridade */
  N_TASKS
} 
TaskIDs;

/* Período de execução das tarefas (em ms) */
#define TASK_PERIOD_BLINK    150  /* 6.67Hz */
#define TASK_PERIOD_JOYSTICK 100  /* 10Hz */
#define TASK_PERIOD_BALANCE  10   /* 100Hz */

/* Constantes para o sensor de ângulo */
#define GYRO_ADDR    0x68           /* Endereço I²C do ADC do giroscópio */
#define ACC_PORT     1              /* Porta de I/O do ADC do acelerômetro */
#define ACC_OFFSET_PORT 3           /* Porta de I/O com o trimpot de controle do offset do acelerômetro */
#define GYRO_SCALE   1.2024835820f  /* Constante de escala do giroscópio (0.67mV/grau/s) */
#define ACC_SCALE    0.215805f	    /* Constante de escala do acelerômetro (800mV/g) */

/* Constantes para os filtros passa alta e passa baixa */
#define HIGHPASS  0.966f            /* Calculado para 100Hz, tau=0.75 */
#define LOWPASS   (1.00f - HIGHPASS)

/* Constantes para o controlador PIDW */
#define P_GAIN       3.6f	/* Constante Proporcional */
#define I_GAIN       2.0571428570f	/* Constante Integrativa */
#define D_GAIN       3.5f	/* Constante Derivativa */
#define W_GAIN       3.00f	/* Constante para Velocidade Angular */

/* Constantes para o controlador de motor */
#define LEFT_MOTOR  10		/* Porta de I/O para PWM do servo esquerdo */
#define RIGHT_MOTOR 9		/* Porta de I/O para PWM do servo direito */
#define LEFT_MOTOR_ZERO 0       /* Offset de zero para motor esquerdo */
#define RIGHT_MOTOR_ZERO 0      /* Offset de zero para motor direito */

/* Constantes para o driver de joystick */
#define JOYSTICK_DATA 6		/* Porta de I/O para dados do joystick */
#define JOYSTICK_CLOCK 3	/* Porta de I/O para clock do joystick */
#define JOYSTICK_ATT 4		/* Porta de I/O para attention do joystick */
#define JOYSTICK_CMD 5		/* Porta de I/O para comando do joystick */

/* Constantes para o LED de Status */
#define STATUS_LED_GREEN 7      /* Porta de I/O para LED verde */
#define STATUS_LED_RED   8      /* Porta de I/O para LED vermelho */

/* Constantes para a porta serial */
#define SERIAL_BAUD_RATE 9600  /* Baud rate para a porta serial */

/****************************************************************************************************************/
/** Tipos e Protótipos ******************************************************************************************/
/****************************************************************************************************************/

class Task;                  /* Tarefa */
class TaskManager;           /* Gerenciador/escalonador de tarefas */
class StatusLED;             /* LED de status */
class JoystickController;    /* Controlador de Joystick */
class AngleSensor;           /* Sensor de Ângulo */
class MotorController;       /* Controlador de Motor */
class PIDWController;        /* Controlador PID */

/* Definição para o callback de tarefa */
typedef void (*TaskCallback)(TaskManager &);

/****************************************************************************************************************/
/** Classes *****************************************************************************************************/
/****************************************************************************************************************/

/**
 * Tarefa
 *
 * Encapsula uma tarefa. O construtor recebe dois parâmetros: um ponteiro para uma função
 * e o período de execução da tarefa em milisegundos.
 */
class Task {
private:
  long m_countdown, m_period;
  TaskCallback m_callback;

  /*
   * Apenas a classe TaskManager pode acessar os métodos de uma Task.
   * Para obter/ajustar o período de uma tarefa que está em execução, veja os
   * métodos getTaskPeriod e setTaskPeriod da TaskManager.
   */
  friend class TaskManager;

  /**
   * Decrementa o contador de tempo de "by" unidades. Retorna o novo valor.
   */
  long decrementCountdownBy(long by) {
    return m_countdown -= by;
  }

  /**
   * Recarrega o contador de tempo com o período da tarefa.
   */
  void inline resetCountdown(void) {
    m_countdown = m_period;
  }

  /**
   * Obtém o período da tarefa.
   */
  long getPeriod(void) const {
    return m_period;
  }

  /**
   * Ajusta o período da tarefa. 
   */
  void setPeriod(unsigned long period) {
    m_period = m_countdown = period;
  }

  /**
   * Executa uma iteração da tarefa.
   */
  void inline run(TaskManager &tm) {
    m_callback(tm);
  }

public:
  Task(TaskCallback callback, long period) : 
  m_callback(callback), m_countdown(period), m_period(period) {

  }
};

/**
 * Gerenciador de Tarefa
 *
 * Responsável por gerenciar uma lista de tarefas. O construtor recebe um array de Task e o
 * número de tarefas deste array. O usuário deve periodicamente chamar o método loop() do
 * gerenciador para que o escalonamento de tarefas seja feito.
 *
 * Nota: O escalonador usa a instrução "sleep". Certifique-se de que uma ISR irá acordar
 *       o microcontrolador periodicamente. Por padrão, no Arduino, isso ocorre.
 *
 * As tarefas têm que ser executadas no menor tempo possível. Se necessário, divida-a em
 * tarefas menores. Evite usar funções bloqueantes como delay().
 *
 * A prioridade é feita da maneira mais simples: as tarefas declaradas primeiro no vetor
 * têm maior prioridade. Um exemplo simples, que usa Task e TaskManager:
 *
 * void test_task(void) { Serial.println("hello from test_task"); }
 * Task tasks[] = { Task(test_task, 1000) };
 * TaskManager task_manager(tasks, 1);
 *
 * void setup(void) { Serial.begin(9600); }
 * void loop(void) { task_manager.loop(); }
 *
 * Isto irá fazer a mensagem "hello from test_task" ser impressa pela interface serial a cada
 * 1000ms aproximadamente (devido ao overhead do Task Manager).
 */
class TaskManager {
private:
  Task *m_tasks;
  unsigned int m_nTasks;
  unsigned long m_lastTime;

  void sleep(void) {
    /* Coloca o processador em modo de espera; a interrupção de timer irá acordá-lo 1ms depois */
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    sleep_mode();
    sleep_disable();
  }
public:
  TaskManager(Task *tasks, int n_tasks) : 
  m_tasks(tasks), m_nTasks(n_tasks), m_lastTime(0L) {

  }

  /**
   * Obtém o período da tarefa com determinado ID.
   */
  long getTaskPeriod(const unsigned int id) const {
    if (id < m_nTasks) {
      return m_tasks[id].getPeriod();
    } 
    else {
      return 0;
    }
  }

  /**
   * Ajusta o período da tarefa com determinado ID.
   */
  void setTaskPeriod(const unsigned int id, const long period) {
    if (id < m_nTasks && period > 0) {
      m_tasks[id].setPeriod(period);
    }
  }

  /**
   * Executa uma iteração do escalonador de tarefas.
   *
   * Para cada iteração, o tempo decorrido desde a última chamada deste método
   * é descontado do contador de tempo de cada tarefa; se este for 0 ou negativo,
   * a tarefa é executada e o contador carregado novamente com o valor do
   * período de execução da tarefa.
   */
  void inline loop() {
    Task *task;
    unsigned int n_task;
    unsigned long elapsedTime = millis() - m_lastTime;

    for (n_task = m_nTasks; n_task > 0; n_task--) {
      task = &m_tasks[n_task];

      if (task->decrementCountdownBy(elapsedTime) <= 0) {
        task->run(*this);
        task->resetCountdown();
      }
    }

    m_lastTime = millis();
    sleep();
  }
};

/* Constantes para a classe de controle de LED de status */
#define STATUS_NONE    (0)
#define STATUS_GREEN   (1<<0)
#define STATUS_RED     (1<<1)
#define STATUS_YELLOW  (STATUS_GREEN | STATUS_RED)

/**
 * Controle de LED de Estado
 *
 * Esta classe implementa o controle do LED de status; geralmente é implementada usando
 * um LED bicolor, podendo produzir três cores (verde, vermelho e amarelo).
 *
 * O uso consiste em dizer em quais pinos o LED está conectado e usar o método setStatus()
 * com a constante de estado desejada:
 * - STATUS_NONE: Apaga o LED
 * - STATUS_GREEN: Acende apenas a cor verde
 * - STATUS_RED: Acende apenas a cor vermelha
 * - STATUS_YELLOW: Acende a luz vermelha e a luz verde
 */

class StatusLED {
private:
  uint8_t m_red, m_green;
  uint8_t m_status;

  void setLEDs(boolean red, boolean green) const {
    digitalWrite(m_green, green);
    digitalWrite(m_red, red);
  }
public:
  void setStatus(uint8_t status) {
    setLEDs(status & STATUS_RED, status & STATUS_GREEN);
    m_status = status;
  }

  void turnLEDs(boolean on) const {
    if (on) {
      setLEDs(m_status & STATUS_RED, m_status & STATUS_GREEN);       
    } else {
      setLEDs(false, false);
    }
  }

  StatusLED(uint8_t redPin, uint8_t greenPin) :
  m_red(redPin), m_green(greenPin), m_status(0) {
    pinMode(m_red, OUTPUT);
    pinMode(m_green, OUTPUT);
  }
};

/* Comandos para o driver de joystick de playstation */
static const uint8_t cmd_poll[] = { 0x01, 0x42 };
static const uint8_t cmd_enter_cfg[] = { 0x01, 0x43, 0x00, 0x01 };
static const uint8_t cmd_exit_cfg[] = { 0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A };
static uint8_t cmd_ad_mode[] = { 0x01, 0x44, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };

class JoystickController {
  /*
   *  Driver de Joystick de Playstation2
   *
   *  Baseado no código por Studio Gyokimae (gyokimae@hotmail.com)
   *  Baseado nas especificações por Curious Inventor (http://curiousinventor.com/guides/ps2)
   *  Código reescrito para menor uso de RAM, melhor desempenho e melhor
   *  adequação a orientação a objetos por
   *     Leandro A. F. Pereira (leandro.pereira@gmail.com)
   */
private:
  uint8_t m_padState[10];
  boolean m_isDigital;

  void sendCommand(const uint8_t *msg, uint8_t msg_cnt) {
    uint8_t cmd;

    /* Avisa o controle que ele ira receber um comando */
    digitalWrite(JOYSTICK_ATT, LOW);

    /* Envia o comando e armazena a resposta */
    for (uint8_t stateBufferIdx = 0;
	     stateBufferIdx < sizeof(m_padState);
	     stateBufferIdx++) {
      cmd = (stateBufferIdx < msg_cnt) ? msg[stateBufferIdx] : 0x00;
      m_padState[stateBufferIdx] = readWriteByte(cmd);
    }

    /* Avisa o controle que o comando ja foi recebido */
    digitalWrite(JOYSTICK_ATT, HIGH);
  }

  uint8_t readWriteByte(uint8_t cmd) const {
    uint8_t received = 0;

    for (int i = 8; i > 0; i--) {
      /* Escreve o primeiro bit do comando na porta de comando */
      digitalWrite(JOYSTICK_CMD, cmd & 0x01);

      /* Rotaciona o byte de comando para a direita */
      cmd >>= 1;
      /* Prepara o byte de resposta para receber o bit da porta de dados */
      received >>= 1;

      /* Desce o clock */
      digitalWrite(JOYSTICK_CLOCK, LOW);
      /* Sobe o clock */
      digitalWrite(JOYSTICK_CLOCK, HIGH);

      /* Armazena o bit lido da porta de dados no byte de resposta */
      if (digitalRead(JOYSTICK_DATA))
        received |= 0x80;
    }

    return received;
  }

  void setMode(uint8_t mode, uint8_t lock) {
    cmd_ad_mode[3] = mode;
    cmd_ad_mode[4] = lock;

    sendCommand(cmd_enter_cfg, sizeof(cmd_enter_cfg));
    sendCommand(cmd_ad_mode, sizeof(cmd_ad_mode));
    sendCommand(cmd_exit_cfg, sizeof(cmd_exit_cfg));
  }

public:
  void update(void) {
    sendCommand(cmd_poll, sizeof(cmd_poll));
  }

  boolean isDigital(void) const {
    return m_isDigital;
  }

  void modeDigital(boolean lock) {
    m_isDigital = true;
    setMode(0x00, lock ? 0x03 : 0x02);
  }

  void modeAnalog(boolean lock) {
    m_isDigital = false;
    setMode(0x01, lock ? 0x03 : 0x02);
  }

  boolean inline turnDown (void) const {
    return !(m_padState[3] & 1 << 5);
  }

  boolean inline turnRight(void) const {
    return !(m_padState[3] & 1 << 4);
  }

  boolean inline turnLeft(void) const {
    return !(m_padState[3] & 1 << 6);
  }

  boolean inline turnUp(void) const {
    return !(m_padState[3] & 1 << 3);
  }

  boolean inline isSquareDown(void) const {
    return !(m_padState[4] & 1 << 6);
  }

  boolean inline isCrossDown(void) const {
    return !(m_padState[4] & 1 << 5);
  }

  boolean inline isCircleDown(void) const {
    return !(m_padState[4] & 1 << 4);
  }

  boolean inline isTriangleDown(void) const {
    return !(m_padState[4] & 1 << 3);
  }

  void init(void)  {
    /* Ajusta a direcao dos pinos de I/O */
    pinMode(JOYSTICK_CLOCK, OUTPUT);
    pinMode(JOYSTICK_CMD, OUTPUT);
    pinMode(JOYSTICK_DATA, INPUT);
    pinMode(JOYSTICK_ATT, OUTPUT);

    /* Estado inicial dos pinos */
    digitalWrite(JOYSTICK_CLOCK, HIGH);
    digitalWrite(JOYSTICK_CMD, LOW);
    digitalWrite(JOYSTICK_ATT, HIGH);
    digitalWrite(JOYSTICK_DATA, LOW);

    update();
  }

  JoystickController(void) {
    for (int i = 0; i < sizeof(m_padState); i++) {
      m_padState[i] = 0;
    }
  }
};

/**
 * Sensor de ângulo
 * Esta classe provê um sensor de ângulo utilizando valores fornecidos por um giroscópio e por
 * um acelerômetro. O valor do ângulo é calculado usando uma combinação de dois filtros: um
 * passa alta e um passa baixa.
 *
 * O sensor de passa alta é aplicado ao ângulo, calculado através de uma integração numérica
 * levando em consideração a velocidade angular obtida pelo giroscópio. A maior parte do
 * sinal final (90%) vem deste ângulo: que tem uma boa precisão, mas pode deslizar depois
 * de um tempo, sobretudo por mudanças de temperatura.
 *
 * O restante do sinal final (10%) é resultado do filtro passa baixa, que é aplicada na
 * aceleração obtida pelo acelerômetro. Isto dá uma boa idéia de qual é o ângulo.
 *
 * Para usar esta classe, deve-se conhecer:
 * - Porta de I/O do ADC para o giroscópio
 * - Porta de I/O do ADC para o acelerômetro
 * - Constante de proporcionalidade do giroscópio (*)
 * - Constante de proporcionalidade do acelerômetro (**)
 * - Período, em segundos, entre cada chamada ao método update()
 *
 * (*) e (**) Estas constantes são responsáveis pela conversão do valor do ADC em valores
 *            fisicamente palpáveis: velocidade angular no caso do giroscópio e aceleração,
 *            no caso do acelerômetro. Estas constantes são fornecidas nos datasheets dos
 *            respectivos sensores.
 * 
 * O uso resume-se em:
 * - Instanciar um objeto da classe
 * - Posicionar o sensor em 90°
 * - Chamar o método calibrate
 * - Chamar o método update, passando como parâmetro a diferença de tempo (em segundos)
 *   para a última chamada ao método update
 * - Obter o ângulo com o método getAngle()
 * - Obter a velocidade angular com getAngularVelocity()
 *
 */
class AngleSensor {
private:
  uint8_t m_gyroAddr, m_accPort, m_accOffsetPort;
  double m_gyroShift, m_gyroScale;
  double m_accShift, m_accScale, m_accShiftIncrease;

  double m_angularVel;
  double m_angle;

  double readAccelerometerRaw(void) const {
    return analogRead(m_accPort);
  }

  double readGyroscopeRaw(void) const {       
    Wire.requestFrom((uint8_t)m_gyroAddr, (uint8_t)3);

    if (Wire.available() == 3) {
      return (Wire.receive() << 8) | Wire.receive();
    } 
    else {
      return 0;
    }
  }

  double readAccelerometer(void) const {
    return ((double)readAccelerometerRaw() - m_accShift) * m_accScale;
  }

  double readGyroscope(void) const {
    return ((double)readGyroscopeRaw() - m_gyroShift) * m_gyroScale;
  }

public:
  AngleSensor(uint8_t gyro_addr, double gyro_scale, uint8_t acc_port, double acc_scale,
  uint8_t acc_offset_port):
  m_gyroAddr(gyro_addr),
  m_gyroScale(gyro_scale),
  m_accPort(acc_port),
  m_accScale(acc_scale),
  m_angle(0.0f),
  m_accShift(0.0f),
  m_accShiftIncrease(0.0f),
  m_gyroShift(0.0f),
  m_accOffsetPort(acc_offset_port) {
  }
  
  double getGyroShift(void) const {
    return m_gyroShift;
  }

  void calibrateGyro(StatusLED &statusLed) {
    double offsetGyro = 0.0f;
    int i = 0;

    /* Calibra os sensores: acumula a leitura deles por 10 vezes; o LED
     de status piscará em vermelho durante este passo */
    for (i = 0; i < 10; i++) {
      statusLed.setStatus(STATUS_NONE);
      delay(100);

      offsetGyro += readGyroscopeRaw();

      statusLed.setStatus(STATUS_RED);
      delay(100);
    }

    /* Calcula o offset: média aritmética das leituras acumuladas */
    m_gyroShift = offsetGyro / 10.0f;
  }

  void inline calibrateAccelerometer(StatusLED &statusLed) {
    m_accShift = analogRead(m_accOffsetPort) + m_accShiftIncrease;
  }

  void inline setAccShiftIncrease(double acc_shift_increase) {
    m_accShiftIncrease = acc_shift_increase;
  }

  void calibrate(StatusLED &statusLed) {
    calibrateGyro(statusLed);
    calibrateAccelerometer(statusLed);
  }

  double getAngle(void) const {
    return m_angle;
  }

  double getAngularVelocity(void) const {
    return m_angularVel;
  }

  void update(double dt) {
    double readingAcc = readAccelerometer();
    double readingGyro = readGyroscope();

    m_angle = HIGHPASS * (m_angle + (readingGyro * dt)) + LOWPASS * readingAcc;
    m_angularVel = readingGyro;
  }
};

/**
 * Controlador de Motor
 *
 * Encapsula a classe Servo (inclusa no SDK do Arduino) para controlar os dois motores
 * do robô. 
 * Leva em consideração que os motores são servo motores adaptados para rotação contí-
 * nua e que a posição parada dá-se quando o ângulo e 90°.
 */
class MotorController {
private:
  Servo m_leftServo, m_rightServo;
  unsigned int m_leftZeroOffset, m_rightZeroOffset;

public:
  void zero(StatusLED &statusLed) {
    drive(90 - m_leftZeroOffset, 90 - m_rightZeroOffset);
  }

  void drive(int torqueLeft, int torqueRight) {
      m_rightServo.write(torqueRight - m_rightZeroOffset);
      m_leftServo.write(torqueLeft - m_leftZeroOffset);
  }

  void drive(double torque) {
    drive(torque, torque);
  }
  
  void init(int left_io, int right_io) {
    m_leftServo.attach(left_io);
    m_rightServo.attach(right_io);
  }

  MotorController(uint8_t leftZeroOffset, uint8_t rightZeroOffset) :
  m_leftZeroOffset(leftZeroOffset),
  m_rightZeroOffset(rightZeroOffset) {
  }
};

/**
 * Controlador PID
 *
 * Utiliza PID para calcular o torque do motor:
 * - (P)roporcional
 *   Altera o torque baseado na informação atual, diretamente baseada na
 *   diferença do ângulo atual para o ângulo correto. É usado também para
 *   calcular o termo da velocidade angular (w).
 * - (I)ntegral
 *   Altera o torque baseado em informações passadas.
 * - (D)erivativo
 *   Altera o torque baseado na trajetória futura.
 *
 * O uso da classe consiste-se em:
 * - Conhecer os ganhos P, I, D e W.
 * - Instanciar o objeto.
 * - Chamar o método update() passando o ângulo e a velocidade angular atuais.
 * - Obter ou cada termo independentemente usando os getters fornecidos, ou
 *   obter a soma deles, usando getPID() ou getPIDW().
 */
class PIDWController {
private:
  double m_PTerm, m_ITerm, m_DTerm, m_WTerm;
  double m_oldAngleError;
  double m_PGain, m_IGain, m_DGain, m_WGain;

public:
  double setPTerm(double p) {
    m_PTerm = p;
  }

  double setITerm(double i) {
    m_ITerm = i;
  }

  double setDTerm(double d) {
    m_DTerm = d;
  }

  double setWTerm(double w) {
    m_WTerm = w;
  }

  double getPTerm(void) const {
    return m_PTerm;
  }

  double getITerm(void) const {
    return m_ITerm;
  }

  double getDTerm(void) const {
    return m_DTerm;
  }

  double getWTerm(void) const {
    return m_WTerm;
  }

  double getPID(void) const {
    return m_PTerm + m_DTerm + m_ITerm;
  }

  double getPIDW(void) const {
    return getPID() + m_WTerm;
  }

  void update(double angle, double angular_vel) {        
    double angle_error = 0.0 - angle;

    /* Calcula os termos proporcionais, integrativos e derivativos */
    m_PTerm = angle_error * m_PGain;
    m_ITerm = m_ITerm + (angle_error * m_IGain);
    m_DTerm = (angle_error - m_oldAngleError) * m_DGain;
    m_WTerm = angular_vel * m_WTerm;

    m_ITerm = constrain(m_ITerm, -500.0, 500.0);

    /* Salva o valor do erro do ângulo atual para cálculo do termo derivativo */
    m_oldAngleError = angle_error;        
  }

  PIDWController(double p_gain, double i_gain, double d_gain,
  double w_gain):
  m_PGain(p_gain), m_IGain(i_gain),
  m_DGain(d_gain), m_WGain(w_gain), m_oldAngleError(0.0f),
  m_PTerm(0.0f), m_ITerm(0.0f), m_DTerm(0.0f), m_WTerm(0.0f) {

  }
};

/****************************************************************************************************************/
/** Variáveis Globais *******************************************************************************************/
/****************************************************************************************************************/

/* Sensor de ângulo */
AngleSensor angleSensor(GYRO_ADDR, GYRO_SCALE, ACC_PORT, ACC_SCALE, ACC_OFFSET_PORT);

/* Controlador PIDW (proporcional, integrativo, derivativo, velocidade angular) */
PIDWController pidw(P_GAIN, I_GAIN, D_GAIN, W_GAIN);

/* Controlador de Motor */
MotorController motor(LEFT_MOTOR_ZERO, RIGHT_MOTOR_ZERO);

/* Controlador de Joystick */
JoystickController joystick;

/* LED de Status */
StatusLED status(STATUS_LED_GREEN, STATUS_LED_RED);

/****************************************************************************************************************/
/** Tarefas *****************************************************************************************************/
/****************************************************************************************************************/

/**
 * Tarefa de Equilíbrio
 * Responsável por manter o robô equilibrado.
 */
void balance_task(TaskManager &tm)
{
  double torque; 		        /* Torque a ser aplicado nos motores */
  int torqueLeft, torqueRight;

  /* Obtém a inclinação atual */
  angleSensor.calibrateAccelerometer(status);
  angleSensor.update(0.010);

  /* Calcula as novas constantes PID e F de acordo com o ângulo calculado */
  pidw.update(angleSensor.getAngle(), angleSensor.getAngularVelocity());

  /* Obtém o torque para ser aplicado no motor */
  torque = pidw.getPIDW();

  /* Mantém o valor do torque entre -400 e 400 */
  torque = map(torque, -400, 400, 180, 0);
  torqueLeft = constrain(torque, 0, 180);
  torqueRight = map(torqueLeft, 180, 0, 0, 180);

  /* Movimenta o robo de acordo com as teclas pressionadas */
  if (joystick.turnRight()) {
    torqueLeft += 20;
    torqueRight += 20;
  } else if (joystick.turnLeft()) {
    torqueLeft -= 20;
    torqueRight -= 20;
  } 

  /* Controla o motor. */ 
  motor.drive(torqueLeft, torqueRight);
}

/**
 * Tarefa de leitura do Joystick
 * Responsável por obter o estado atual do joystick, e entrar no modo de configuração.
 */
void joystick_poll_task(TaskManager &tm)
{
  /* Pede ao joystick que envie novo estado */
  joystick.update();
  
  if (joystick.turnUp()) {
    angleSensor.setAccShiftIncrease(15.0f);
  } else if (joystick.turnDown()) {
    angleSensor.setAccShiftIncrease(-15.0f);
  } else {
    angleSensor.setAccShiftIncrease(0.0f);
  }
}

/**
 * Tarefa de controle do LED.
 */
void blink_status_task(TaskManager &tm)
{
  static char led_status = false;

  status.turnLEDs(led_status ^= 1);
}


/****************************************************************************************************************/
/** Definição das Tarefas ***************************************************************************************/
/****************************************************************************************************************/

/* Lista de Tarefas */
Task tasks[] = {
  Task(blink_status_task, TASK_PERIOD_BLINK),
  Task(joystick_poll_task, TASK_PERIOD_JOYSTICK),
  Task(balance_task, TASK_PERIOD_BALANCE),
};

/* Gerenciador de Tarefas */
TaskManager task_manager(tasks, sizeof(tasks) / sizeof(tasks[0]));

/****************************************************************************************************************/
/** Rotinas de Configuração e Laço Principal ********************************************************************/
/****************************************************************************************************************/
/*
 * Rotina de laço
 * Chamada em um laço infinito. 
 */
void inline loop()
{
  task_manager.loop();
}

/*
 * Rotina de configuração
 * Executada apenas uma vez, ao ligar o microcontrolador e ter os dispositivos básicos inicializados.
 */
void inline setup()
{
  /* Inicia a calibragem do robô */
  status.setStatus(STATUS_RED);

  /* Configura a porta serial (para depuração) */
  Serial.begin(SERIAL_BAUD_RATE);

  /* Initializa a biblioteca de I²C (para o giroscópio) */
  Wire.begin();

  /* Inicializa o joystick */
  joystick.init();

  /* Ajusta o joystick para modo digital */
  joystick.modeDigital(true);

  /* Move os motores para a posição central */
  motor.init(LEFT_MOTOR, RIGHT_MOTOR);
  motor.zero(status);

  /* Calibra o sensor de ângulo */
  angleSensor.calibrate(status);

  /* Pisca o LED para avisar que o robô está pronto para começar a balancear */
  for (int i = 0; i < 10; i++) {
    status.setStatus(STATUS_YELLOW);
    delay(100);
    status.setStatus(STATUS_NONE);
    delay(100);
  }

  /* Rotina de calibragem terminada: acende o LED verde */
  status.setStatus(STATUS_GREEN);
}

