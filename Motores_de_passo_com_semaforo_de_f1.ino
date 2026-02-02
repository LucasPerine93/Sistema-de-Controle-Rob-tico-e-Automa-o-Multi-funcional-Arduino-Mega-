/* =================================================================================
   1. BIBLIOTECAS E ESTRUTURAS DE DADOS
   ================================================================================= */
#include <AccelStepper.h>
#include <Servo.h>
#include <LiquidCrystal.h>
#include <NewPing.h>
#include <LinkedList.h>
#include <EEPROM.h>

// Listas para armazenamento dinâmico
LinkedList<int> listaDeMedidas;
LinkedList<int> listaDeMedidas2;
LinkedList<float> listaTemposF1;

/* =================================================================================
   2. MAPEAMENTO DE HARDWARE (PINOUT)
   ================================================================================= */
// --- Entradas Analógicas (Potenciômetros) ---
#define pot1 A12
#define pot2 A13
#define pot3 A14
#define pot4 A15

// --- Drivers de Motor de Passo (Step/Dir) ---
#define STEP_PIN1 4
#define DIR_PIN1 7

#define STEP_PIN2 2
#define DIR_PIN2 5

#define STEP_PIN3 3
#define DIR_PIN3 6

#define STEP_PIN4 23
#define DIR_PIN4 25

#define PINO_ENABLE 8

// --- Ponte H (Motor DC Auxiliar) ---
#define BASE1 42
#define BASE2 44
#define BASE3 46
#define BASE4 48

// --- Interface de Usuário (Botões e LEDs) ---
#define botao1 53
#define botao2 38
#define botao3 40
#define botao4 15

#define LED_PIN1 43
#define LED_PIN2 45
#define LED_PIN3 47
#define LED_PIN4 49
#define LED_PIN5 51
#define LED_INTERNO 13

/* =================================================================================
   3. CONFIGURAÇÕES E CONSTANTES DO SISTEMA
   ================================================================================= */
#define MAX_FRAMES 650          // Memória para Servos (byte)
#define MAX_FRAMES2 20          // Memória para Passos (long)
const int TAMANHO_PASTA = 330;  // Tamanho do bloco na EEPROM

/* =================================================================================
   4. INSTANCIAMENTO DE OBJETOS
   ================================================================================= */
// Displays LCD
LiquidCrystal lcd(22, 24, 26, 28, 30, 32);
LiquidCrystal lcd2(27, 29, 31, 33, 35, 37);

// Sensores Ultrassônicos
NewPing sonar(39, 41, 400);
NewPing sonar2(50, 52, 400);

// Servos
Servo meuServo;
Servo meuServo2;

// Motores de Passo
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN1, DIR_PIN1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN2, DIR_PIN2);
AccelStepper stepper3(AccelStepper::DRIVER, STEP_PIN3, DIR_PIN3);
AccelStepper stepperA(AccelStepper::DRIVER, STEP_PIN4, DIR_PIN4);

/* =================================================================================
   5. MÁQUINA DE ESTADOS E MODOS DE OPERAÇÃO
   ================================================================================= */
// Definição dos IDs dos Modos
const byte MODO_SERVO_SERIAL = 0;
const byte MODO_SERVO_POT = 1;
const byte MODO_PASSO_SERIAL = 2;
const byte MODO_PASSO_POT = 3;
const byte MODO_MEDIDOR = 4;
const byte MODO_VELOCIDADE_PASSOS = 5;
const byte MODO_DISTANCIA_SERVOS = 6;
const byte SEMAFORO_DE_F1 = 7;
const byte MODO_GHOST_MOTORES = 8;
const byte MODO_GHOST_PASSOS = 9;
const byte MODO_GHOST_SERVOS = 10;
const byte AJUDA = 11;
const byte MANUAL = 12;
const byte MODO_MARCHAS = 13;
const byte TRACADO = 14;
const byte MOTOR_VELOCIDADE = 15;
const byte OFF = 16;
const byte ON = 17;
const byte STANDBY = 18;
const byte IA_F1 = 19;

// Controle de Estado Atual
byte ModoAtual = 0;
bool motoresDesativados = false;
byte estado = 0;  // Usado para sub-estados (ex: rec/play/stop)

// Lista dos modos que funcionam sem Serial
byte listaModosOffline[] = {
  STANDBY,
  MODO_SERVO_POT,
  MODO_PASSO_POT,
  MODO_VELOCIDADE_PASSOS,
  MODO_MEDIDOR,
  MODO_MARCHAS,
  SEMAFORO_DE_F1,
  MOTOR_VELOCIDADE
};

int indiceModosOffline = 0;

const int totalDeModosOffline = sizeof(listaModosOffline);

/* =================================================================================
   6. VARIÁVEIS GLOBAIS - CONTROLE DE MOTORES
   ================================================================================= */
// Configurações Gerais
int MaxSpeed = 6000;
long Aceleracao = 0;  // Usado no modo Traçado

// Variáveis dos Servos
int ultimaPosServo1 = -1;
int ultimaPosServo2 = -1;

// Variáveis dos Motores de Passo
int lastPosition1 = 0;
int lastPosition2 = 0;
int lastPosition3 = 0;
int lastPosition4 = 0;

long speed = 0;
long speed2 = 0;
long speed3 = 0;
long speed4 = 0;

long passos = 0;  // Armazena comando serial temporário

// Posições Atuais (Referência)
int posAtual1 = 0;
int posAtual2 = 0;
int posAtual3 = 0;
int posAtual4 = 0;

// Modo Marchas (Limites de RPM simulados)
int limites[7] = { 1000, 200, 400, 600, 800, 1000, 1400 };

/* =================================================================================
   7. VARIÁVEIS DO MÓDULO "GHOST MODE" (GRAVAÇÃO)
   ================================================================================= */
// Buffers de Memória RAM
byte ghostM1[MAX_FRAMES];  // Servo 1
byte ghostM2[MAX_FRAMES];  // Servo 2
byte ghostM3[MAX_FRAMES];  // (Reservado/Extra)
byte ghostM4[MAX_FRAMES];  // (Reservado/Extra)

long ghostMP1[MAX_FRAMES2];  // Passo 1
long ghostMP2[MAX_FRAMES2];  // Passo 2
long ghostMP3[MAX_FRAMES2];  // Passo 3
long ghostMP4[MAX_FRAMES2];  // Passo 4

// Controle de Gravação/Reprodução
const byte delayGhost = 95;
unsigned long tempoApGhost = 0;
unsigned long tempoRecord = 0;
unsigned long frameRate = 0;
int totalGravado = 0;
int pegarvalor1 = 0;

/* =================================================================================
   8. VARIÁVEIS DO MÓDULO "F1 SEMÁFORO"
   ================================================================================= */
typedef enum { IDLE,
               WAITING,
               TIMING } State;
State state = IDLE;

unsigned long startTime = 0;
unsigned long reactionTime = 0;
bool buttonPressed = false;
int largadasInvalidas = 0;

/* =================================================================================
   9. Molde para IA de desgaste
   ================================================================================= */

class CerebroIA {
public:
  String nome;
  float alvoReal1;
  float alvoReal2;
  float alvoIA1;
  float alvoIA2;

  CerebroIA(String n, float real1, float real2) {
    nome = n;
    alvoReal1 = real1;
    alvoReal2 = real2;
    alvoIA1 = 0.0;
    alvoIA2 = 0.0;
  }

  void treinar() {
    float voltas = random(5, 71);

    float valorCorreto = (voltas * alvoReal1) + alvoReal2;
    float chuteIA = (voltas * alvoIA1) + alvoIA2;
    float erro = valorCorreto - chuteIA;

    alvoIA1 = alvoIA1 + (erro * voltas * 0.0001);
    alvoIA2 = alvoIA2 + (erro * 0.01);

    imprimirTelemetria(voltas, valorCorreto, chuteIA, alvoIA1, alvoIA2);
  }
  void imprimirTelemetria(float voltas, float valorCorreto, float chuteIA, float alvoIA1, float alvoIA2) {

    if (nome == "COMBUSTIVEL") {
      Serial.print("\nVoltas: ");
      Serial.print(voltas, 0);

      Serial.print(" | Alvo: ");
      Serial.print(valorCorreto);

      Serial.print("L | IA Chutou: ");
      Serial.print(chuteIA);
      Serial.print("L");

      Serial.print(" || APRENDENDO -> Consumo: ");
      Serial.print(alvoIA1, 3);
      Serial.print(" | Reserva: ");
      Serial.println(alvoIA2, 3);

    } else {

      Serial.print("Alvo: ");
      Serial.print(valorCorreto);

      Serial.print(" | IA Chutou: ");
      Serial.print(chuteIA);
      Serial.print("");

      Serial.print(" || APRENDENDO -> Desgaste: ");
      Serial.println(alvoIA1, 2);
      Serial.print("\n\n");
    }
  }
};

/* =================================================================================
   10. VARIÁVEIS AUXILIARES E INTERFACE
   ================================================================================= */
unsigned long tempoMensagemSalvo = 0;
bool mostrandoMensagem = false;

void setup() {
  lcd.begin(20, 4);
  lcd2.begin(16, 4);
  Serial.begin(115200);

  pinMode(PINO_ENABLE, OUTPUT);
  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  pinMode(LED_PIN3, OUTPUT);
  pinMode(LED_PIN4, OUTPUT);
  pinMode(LED_PIN5, OUTPUT);
  pinMode(LED_INTERNO, OUTPUT);
  pinMode(BASE1, OUTPUT);
  pinMode(BASE2, OUTPUT);
  pinMode(BASE3, OUTPUT);
  pinMode(BASE4, OUTPUT);
  pinMode(botao1, INPUT_PULLUP);
  pinMode(botao2, INPUT_PULLUP);
  pinMode(botao3, INPUT_PULLUP);
  pinMode(botao4, INPUT_PULLUP);

  digitalWrite(LED_INTERNO, HIGH);

  randomSeed(analogRead(11));

  stepper.setMaxSpeed(MaxSpeed);
  stepper2.setMaxSpeed(MaxSpeed);
  stepper3.setMaxSpeed(MaxSpeed);
  stepperA.setMaxSpeed(MaxSpeed);

  Serial.println(F(""));
  Serial.println(F("[SYSTEM] Iniciando Kernel do Arduino Mega..."));
  delay(200);
  Serial.println(F("[CHECK]  Verificando Memoria SRAM......... [OK]"));
  Serial.println(F("[CHECK]  Carregando Drivers de Motor...... [OK]"));
  delay(100);
  Serial.println(F("[CHECK]  Inicializando Display LCD........ [OK]"));
  Serial.println(F("[CHECK]  Testando Sensor Ultrassonico..... [OK]"));
  Serial.println(F("--------------------------------------------------"));
  Serial.println(F("       >>> SISTEMA OPERACIONAL ATIVO <<<          "));
  Serial.println(F("--------------------------------------------------"));
  Serial.println(F("Comando Mestre: Digite 'H' para abrir o Painel."));
  Serial.println(F(""));

  meuServo.attach(34);
  meuServo2.attach(36);

  Serial.println(F("O modo padrão de inicialização é o controle dos Servos por potenciometro"));
  Serial.println(F("\nQualquer duvida que tiver digite 'MANUAL' um menu com todas as explicações da maquina"));
  Serial.print(F("vai se abrir para lhe informar sobre todas as funcionalidades do projeto"));
  Serial.print(F("\n\n\n\n\n"));


  lcd.clear();
  lcd2.clear();

  lcd2.setCursor(0, 0);
  lcd2.print("Controle dos    ");
  lcd2.setCursor(0, 1);
  lcd2.print("servos por      ");
  lcd2.setCursor(0, 2);
  lcd2.print("potenciometro   ");

  ModoAtual = MODO_SERVO_POT;
  motoresDesativados = true;
}

void loop() {
  String input;
  TrocaDeModoBotao();
  if ((ModoAtual == MODO_GHOST_PASSOS) || (ModoAtual == MODO_VELOCIDADE_PASSOS) || (ModoAtual == MODO_PASSO_SERIAL) || (ModoAtual == MODO_PASSO_POT) || (ModoAtual == MODO_MARCHAS) || (ModoAtual == TRACADO) || (ModoAtual == MOTOR_VELOCIDADE)) {

    stepper.run();
    stepper2.run();
    stepper3.run();
    stepperA.run();
  }

  if (Serial.available() != 0) {
    input = Serial.readStringUntil('\n');
    input.trim();
    String comando = input;
    comando.toUpperCase();

    desligarMotorVel();

    if (ModoAtual != TRACADO) {
      Aceleracao = 0;
    }

    if (comando.equals("SERVO SERIAL") || comando.equals("SERIAL SERVO") || comando.equals("CMSS")) {
      ModoAtual = MODO_SERVO_SERIAL;
      motoresDesativados = true;

      meuServo.attach(34);
      meuServo2.attach(36);

      Serial.println(F(""));
      Serial.println(F("+------------------------------------------------+"));
      Serial.println(F("| MODO ATIVO: SERVO MOTOR (SERIAL)               |"));
      Serial.println(F("+------------------------------------------------+"));
      Serial.println(F("| STATUS: Aguardando comando...                  |"));
      Serial.println(F("| DICA:   Digite 'A 90' ou 'B 180'               |"));
      Serial.println(F("+------------------------------------------------+"));

      lcd.clear();
      lcd2.clear();

      resetarInercia();

      lcd2.setCursor(0, 0);
      lcd2.print("Controle dos");
      lcd2.setCursor(0, 1);
      lcd2.print("servos por  ");
      lcd2.setCursor(0, 2);
      lcd2.print("serial");

      return;

    } else if (comando.equals("SERVO POT") || comando.equals("CONTROLE DE SERVO POR POTENCIOMETRO") || comando.equals("CSP")) {
      ModoAtual = MODO_SERVO_POT;
      motoresDesativados = true;

      meuServo.attach(34);
      meuServo2.attach(36);

      Serial.println(F(""));
      Serial.println(F("+------------------------------------------------+"));
      Serial.println(F("| MODO ATIVO: SERVO MOTOR (POTENCIOMETRO)        |"));
      Serial.println(F("+------------------------------------------------+"));
      Serial.println(F("| STATUS: Controle Manual Ativado                |"));
      Serial.println(F("| DICA:   Use os Pots 1 e 2 para mover           |"));
      Serial.println(F("+------------------------------------------------+"));

      lcd.clear();
      lcd2.clear();

      resetarInercia();

      lcd2.setCursor(0, 0);
      lcd2.print("Controle dos    ");
      lcd2.setCursor(0, 1);
      lcd2.print("servos por      ");
      lcd2.setCursor(0, 2);
      lcd2.print("potenciometro   ");

      return;

    } else if (comando.equals("MOTOR DE PASSO POR SERIAL") || comando.equals("CONTROLE DE MOTORES DE PASSO POR SERIAL") || comando.equals("CMPS")) {

      meuServo.detach();
      meuServo2.detach();

      ModoAtual = MODO_PASSO_SERIAL;
      motoresDesativados = false;

      Serial.println(F(""));
      Serial.println(F("+------------------------------------------------+"));
      Serial.println(F("| MODO ATIVO: MOTOR DE PASSO (SERIAL)            |"));
      Serial.println(F("+------------------------------------------------+"));
      Serial.println(F("| STATUS: Aguardando coordenadas...              |"));
      Serial.println(F("| DICA:   Digite 'M1 200' ou 'M2 -500'           |"));
      Serial.println(F("+------------------------------------------------+"));

      lcd.clear();
      lcd2.clear();

      resetarInercia();

      lcd2.setCursor(0, 0);
      lcd2.print("Controle dos     ");
      lcd2.setCursor(0, 1);
      lcd2.print("motores de passo ");
      lcd2.setCursor(0, 2);
      lcd2.print("por serial");

      return;

    } else if (comando.equals("MOTOR DE PASSO POR POT") || comando.equals("MPP")) {

      meuServo.detach();
      meuServo2.detach();

      ModoAtual = MODO_PASSO_POT;
      motoresDesativados = false;

      Serial.println(F(""));
      Serial.println(F("+------------------------------------------------+"));
      Serial.println(F("| MODO ATIVO: MOTOR DE PASSO (POSICAO POT)       |"));
      Serial.println(F("+------------------------------------------------+"));
      Serial.println(F("| STATUS: Controle de Posicao Manual             |"));
      Serial.println(F("| DICA:   O motor segue o movimento do Pot       |"));
      Serial.println(F("+------------------------------------------------+"));

      lcd.clear();
      lcd2.clear();

      resetarInercia();

      posAtual1 = stepper.currentPosition();
      posAtual2 = stepper2.currentPosition();
      posAtual3 = stepper3.currentPosition();
      posAtual4 = stepperA.currentPosition();

      lcd2.setCursor(0, 0);
      lcd2.print("Controle dos     ");
      lcd2.setCursor(0, 1);
      lcd2.print("motores de passo ");
      lcd2.setCursor(0, 2);
      lcd2.print("potenciometro    ");

      return;

    } else if (comando.equals("MODO VELOCIDADE") || comando.equals("ATIVAR MODO CONTINUO") || comando.equals("ATIVAR O MODO VELOCIDADE DOS MOTORES DE PASSO") || comando.equals("MPV")) {

      meuServo.detach();
      meuServo2.detach();

      ModoAtual = MODO_VELOCIDADE_PASSOS;
      motoresDesativados = false;

      Serial.println(F(""));
      Serial.println(F("+------------------------------------------------+"));
      Serial.println(F("| MODO ATIVO: MOTOR DE PASSO (VELOCIDADE)        |"));
      Serial.println(F("+------------------------------------------------+"));
      Serial.println(F("| STATUS: Controle Continuo (RPM)                |"));
      Serial.println(F("| DICA:   Centro (512) = Parado | Pontas = Max   |"));
      Serial.println(F("+------------------------------------------------+"));

      lcd.clear();
      lcd2.clear();

      resetarInercia();

      lcd2.setCursor(0, 0);
      lcd2.print("Controle dos     ");
      lcd2.setCursor(0, 1);
      lcd2.print("motores de passo ");
      lcd2.setCursor(0, 2);
      lcd2.print("por velocidade   ");

      return;

    } else if (comando.equals("TRACADO") || comando.equals("MODO TRACADO") || comando.equals("MOTORES NO MODO TRACADO")) {

      ModoAtual = TRACADO;
      motoresDesativados = false;

      meuServo.detach();
      meuServo2.detach();

      Serial.println(F(""));
      Serial.println(F("+================================================+"));
      Serial.println(F("|   >>> MODO TRAÇADO (TEACH & REPEAT CNC) <<<    |"));
      Serial.println(F("+================================================+"));
      Serial.println(F("| STATUS:  Sistema de Coordenadas Ativo.         |"));
      Serial.println(F("|------------------------------------------------|"));
      Serial.println(F("| COMO FUNCIONA:                                 |"));
      Serial.println(F("| 1. Digite a aceleração dos motores.             |"));
      Serial.println(F("| 2. [PLAY] Aperte o BOTAO para armar.           |"));
      Serial.println(F("| 3. [REC]  Envie coordenadas: 'M1 2000', 'M2 0' |"));
      Serial.println(F("| 4. [REPEAT] Aperte de novo para EXECUTAR tudo. |"));
      Serial.println(F("| 5. [SAVE] Salve traçados em pastas: [SALVAR 1] |"));
      Serial.println(F("| 6. [CARREGAR] Carregue traçados: [CARREGAR 1]  |"));
      Serial.println(F("+================================================+"));
      Serial.println(F(""));

      Serial.print(F("Digite a aceleração que quiser nos motores [Ex: 1000]"));

      lcd.clear();
      lcd2.clear();

      resetarInercia();

      lcd2.setCursor(0, 0);
      lcd2.print("Modo Tracado");
      lcd2.setCursor(0, 1);
      lcd2.print("CNC Ativo");

      lcd.setCursor(0, 0);
      lcd.print("Aguardando CMD");
      lcd.setCursor(0, 1);
      lcd.print("via Serial...");

      return;

    } else if (comando.equals("LISTAR PASTAS") || comando.equals("LISTAR") && (ModoAtual == TRACADO)) {
      listarPastas();
      return;

    } else if (comando.equals("DESATIVAR MOTORES") || comando.equals("DESLIGAR MOTORES") || comando.equals("MOTOR OFF") || comando.equals("OFF")) {
      ModoAtual = OFF;

      Serial.println(F(""));
      Serial.println(F("+------------------------------------------------+"));
      Serial.println(F("| >>> SISTEMA PARADO / EM PAUSA <<<              |"));
      Serial.println(F("+------------------------------------------------+"));
      Serial.println(F("| STATUS: Motores livres (Soltos).               |"));
      Serial.println(F("| INFO:   Seguro para manuseio.                  |"));
      Serial.println(F("+------------------------------------------------+"));

      lcd.clear();
      lcd2.clear();

      resetarInercia();
      desligarMotorVel();

      return;

    } else if (comando.equals("ATIVAR MOTORES") || comando.equals("LIGAR MOTORES") || comando.equals("MOTOR ON") || comando.equals("ON")) {
      ModoAtual = ON;

      Serial.println(F(""));
      Serial.println(F("+------------------------------------------------+"));
      Serial.println(F("| >>> SISTEMA INICIADO / ENERGIZADO <<<          |"));
      Serial.println(F("+------------------------------------------------+"));
      Serial.println(F("| STATUS: Motores travados e prontos.            |"));
      Serial.println(F("| ATENCAO: Cuidado com partes moveis!            |"));
      Serial.println(F("+------------------------------------------------+"));

      lcd.clear();
      lcd2.clear();

      resetarInercia();

      return;

    } else if (comando.equals("MODO MEDIDOR") || comando.equals("MODO ULTRASSONICO") || comando.equals("MUS")) {
      meuServo.detach();
      meuServo2.detach();

      ModoAtual = MODO_MEDIDOR;
      motoresDesativados = true;

      Serial.println(F(""));
      Serial.println(F("+------------------------------------------------+"));
      Serial.println(F("| MODO ATIVO: MEDIDOR ULTRASSONICO               |"));
      Serial.println(F("+------------------------------------------------+"));
      Serial.println(F("| STATUS: Lendo distancias...                    |"));
      Serial.println(F("| DICA: Pressione o botão p/ salvar medidas      |"));
      Serial.println(F("+------------------------------------------------+"));

      lcd.clear();
      lcd2.clear();

      resetarInercia();

      return;

    } else if ((comando.equals("VER MEDIDAS") || comando.equals("MOSTRAR RELATORIO DE MEDIAS") || comando.equals("MEDIDAS")) && ModoAtual == MODO_MEDIDOR) {
      mostrarResultados();
      mostrarResultados2();

      return;

    } else if ((comando.equals("LIMPAR MEDIDAS") || comando.equals("DELETE MEDIDAS") || comando.equals("DEL MED")) && ModoAtual == MODO_MEDIDOR) {
      apagarResultados();

      return;

    } else if (comando.equals("MODO SERVO POR DISTANCIA") || comando.equals("CONTROLE DO SERVO POR DISTANCIA") || comando.equals("CSPS")) {
      ModoAtual = MODO_DISTANCIA_SERVOS;
      motoresDesativados = true;

      Serial.println(F(""));
      Serial.println(F("+------------------------------------------------+"));
      Serial.println(F("| MODO ATIVO: SERVO MOTOR (SENSOR SONAR)         |"));
      Serial.println(F("+------------------------------------------------+"));
      Serial.println(F("| STATUS: Controle por Gestos / Distancia        |"));
      Serial.println(F("| DICA:   Perto (5cm)=0 | Longe (30cm)=180       |"));
      Serial.println(F("+------------------------------------------------+"));

      lcd.clear();
      lcd2.clear();

      meuServo.attach(34);
      meuServo2.attach(36);

      resetarInercia();

      lcd2.setCursor(0, 0);
      lcd2.print("Modo de controle");
      lcd2.setCursor(0, 1);
      lcd2.print("dos servos por");
      lcd2.setCursor(0, 2);
      lcd2.print("ultrassonico");

      return;

    } else if (comando.equals("MODO MARCHA") || comando.equals("MARCHA") || comando.equals("AIVAR MODO MARCHA") || comando.equals("MARCHAS")) {

      meuServo.detach();
      meuServo2.detach();

      ModoAtual = MODO_MARCHAS;
      motoresDesativados = false;

      Serial.println(F(""));
      Serial.println(F("+================================================+"));
      Serial.println(F("|   >>> SIMULADOR DE CAMBIO (H-SHIFTER) <<<      |"));
      Serial.println(F("+================================================+"));
      Serial.println(F("| STATUS:  Motor Engrenado e Pronto.             |"));
      Serial.println(F("|------------------------------------------------|"));
      Serial.println(F("| CONTROLES DE PILOTAGEM:                        |"));
      Serial.println(F("| > BOTAO 3 : Subir Marcha (+)                   |"));
      Serial.println(F("| > BOTAO 2 : Descer Marcha (-)                  |"));
      Serial.println(F("| > POT 1   : Acelerador (Pedal)                 |"));
      Serial.println(F("+================================================+"));
      Serial.println(F("| DICA: Acelere tudo no NEUTRO para cortar giro! |"));
      Serial.println(F("+================================================+"));
      Serial.println(F(""));

      resetarInercia();

      lcd.clear();
      lcd2.clear();

      lcd2.setCursor(0, 0);
      lcd2.print("Modo marchas");
      lcd2.setCursor(0, 1);
      lcd2.print("ativo");

      lcd.setCursor(0, 0);
      lcd.print("Marcha: 1");

      return;

    } else if (comando.equals("SEMAFORO DE F1") || comando.equals("ATIVAR SEMAFORO DE F1") || comando.equals("SDF1")) {

      meuServo.detach();
      meuServo2.detach();

      ModoAtual = SEMAFORO_DE_F1;
      motoresDesativados = true;

      Serial.println(F(""));
      Serial.println(F("+================================================+"));
      Serial.println(F("|     >>> SIMULADOR DE LARGADA F1 (SDF1) <<<     |"));
      Serial.println(F("+================================================+"));
      Serial.println(F("| STATUS:  Sistema de Cronometragem ARMADO       |"));
      Serial.println(F("| MISSAO:  Aperte o botao quando a luz APAGAR!   |"));
      Serial.println(F("|------------------------------------------------|"));
      Serial.println(F("| COMANDOS DE TELEMETRIA:                        |"));
      Serial.println(F("| > TEMPOS    : Exibe o Ranking e seus Tempos    |"));
      Serial.println(F("| > DEL F1    : Apaga o historico da sessao      |"));
      Serial.println(F("+================================================+"));
      Serial.println(F(""));

      state = IDLE;
      buttonPressed = false;

      resetarInercia();

      lcd.clear();
      lcd2.clear();

      lcd.setCursor(0, 0);
      lcd.print("Pressione o botao");
      lcd.setCursor(0, 1);
      lcd.print("para iniciar");

      lcd2.setCursor(0, 0);
      lcd2.print("Semaforo de F1");
      lcd2.setCursor(0, 1);
      lcd2.print("ativado");

      return;

    } else if ((comando.equals("VER TEMPOS F1") || comando.equals("TELEMETRIA DE TEMPO") || comando.equals("TEMPOS")) && ModoAtual == SEMAFORO_DE_F1) {
      mostrarRelatorioF1();

      return;

    } else if ((comando.equals("LIMPAR TEMPOS") || comando.equals("DELETE TEMPOS") || comando.equals("EXCLUIR TEMPOS") || comando.equals("DEL F1")) && ModoAtual == SEMAFORO_DE_F1) {
      limparDadosF1();

      return;

    } else if (comando.equals("CONSUMO E DESGASTE F1") || comando.equals("IA CONSUMO")) {
      meuServo.detach();
      meuServo2.detach();

      ModoAtual = IA_F1;
      motoresDesativados = true;

      resetarInercia();

      lcd.clear();
      lcd2.clear();

      Serial.println(F(""));
      Serial.println(F("+================================================+"));
      Serial.println(F("|   >>> IA DE ESTRATEGIA DE CORRIDA (ML) <<<     |"));
      Serial.println(F("+================================================+"));
      Serial.println(F("| STATUS:  Rede Neural aguardando parametros...  |"));
      Serial.println(F("| MISSAO:  Calcular Consumo e Desgaste via IA.   |"));
      Serial.println(F("|------------------------------------------------|"));
      Serial.println(F("| COMO FUNCIONA O PROCESSO DE TREINAMENTO:       |"));
      Serial.println(F("| 1. Voce insere os dados REAIS da telemetria.   |"));
      Serial.println(F("| 2. A IA simula milhares de corridas aleatorias.|"));
      Serial.println(F("| 3. Ela ajusta os pesos ate convergir no Alvo.  |"));
      Serial.println(F("|------------------------------------------------|"));
      Serial.println(F("| ESTRUTURA DE COMANDO (SINTAXE):                |"));
      Serial.println(F("| > COMBUSTIVEL, [Litros/Volta] : [Reserva Ini]  |"));
      Serial.println(F("|   Ex: 'COMBUSTIVEL, 2.5 : 5.0'                 |"));
      Serial.println(F("|                                                |"));
      Serial.println(F("| > PNEU, [Desgaste/Volta] : [Fator Extra]       |"));
      Serial.println(F("|   Ex: 'PNEU, 0.8 : 0.0'                        |"));
      Serial.println(F("+================================================+"));
      Serial.println(F("| DICA: A IA para sozinha ao atingir precisão!   |"));
      Serial.println(F("+================================================+"));
      Serial.println(F(""));

      lcd2.setCursor(0, 0);
      lcd2.print("IA consumo e");
      lcd2.setCursor(0, 1);
      lcd2.print("desgaste");

      return;

    } else if (comando.equals("AJUDA") || comando.equals("H")) {
      ModoAtual = AJUDA;

      lcd.clear();
      lcd2.clear();

      lcd2.print("Menu ajuda esta");
      lcd2.setCursor(0, 1);
      lcd2.print("impresso no");
      lcd2.setCursor(0, 2);
      lcd2.print("serial");

      mostrarAjuda();

      return;

    } else if (comando.equals("MANUAL DA MAQUINA") || comando.equals("MEU MANUAL") || comando.equals("MANUAL")) {
      ModoAtual = MANUAL;

      lcd.clear();
      lcd2.clear();

      lcd2.print("Menu manual esta");
      lcd2.setCursor(0, 1);
      lcd2.print("impresso no");
      lcd2.setCursor(0, 2);
      lcd2.print("serial");

      mostrarManualCompleto();

      return;

    } else if (comando.equals("ATIVAR MODO REPLAY") || comando.equals("ATIVAR MODO GHOST") || comando.equals("GHOST") || comando.equals("GHOST MODE")) {

      meuServo.detach();
      meuServo2.detach();

      estado = 0;

      ModoAtual = MODO_GHOST_MOTORES;
      motoresDesativados = true;

      Serial.println(F("\n"));
      Serial.println(F("+================================================+"));
      Serial.println(F("|     >>> GHOST MODE - CENTRAL DE GRAVACAO <<<   |"));
      Serial.println(F("+================================================+"));
      Serial.println(F("| SELECIONE O SISTEMA QUE DESEJA GRAVAR:         |"));
      Serial.println(F("|                                                |"));
      Serial.println(F("| [1] MOTORES DE PASSO (Steppers)                |"));
      Serial.println(F("|     > Comando de Acesso: Digite 'PASSO'        |"));
      Serial.println(F("|                                                |"));
      Serial.println(F("| [2] SERVO MOTORES (Braços Roboticos)           |"));
      Serial.println(F("|     > Comando de Acesso: Digite 'SERVO'        |"));
      Serial.println(F("|                                                |"));
      Serial.println(F("+------------------------------------------------+"));
      Serial.println(F("| INFO: O sistema vai 'aprender' seus movimentos |"));
      Serial.println(F("|       e repeti-los automaticamente.            |"));
      Serial.println(F("+================================================+"));
      Serial.println(F("\n"));

      lcd.clear();
      lcd2.clear();

      resetarInercia();

      lcd2.setCursor(0, 0);
      lcd2.print("Modo GHOST ativo");
      lcd2.setCursor(0, 1);
      lcd2.print("digite no serial");
      lcd2.setCursor(0, 2);
      lcd2.print("o motor desejado");

      return;

    } else if ((comando.equals("MOTORES DE PASSO") || comando.equals("PASSO") || comando.equals("PASSOS")) && (ModoAtual == MODO_GHOST_MOTORES || ModoAtual == MODO_GHOST_SERVOS)) {

      meuServo.detach();
      meuServo2.detach();

      estado = 0;

      ModoAtual = MODO_GHOST_PASSOS;
      motoresDesativados = false;

      Serial.println(F(""));
      Serial.println(F("+================================================+"));
      Serial.println(F("|    >>> GHOST MODE (MEMORIA DE MOVIMENTO) <<<   |"));
      Serial.println(F("+================================================+"));
      Serial.println(F("| STATUS:  Modulo de Gravacao INICIADO           |"));
      Serial.println(F("| ALVO:    MOTORES DE PASSO (STEPPERS) [X]       |"));
      Serial.println(F("|          SERVO MOTORES               [ ]       |"));
      Serial.println(F("|------------------------------------------------|"));
      Serial.println(F("| INSTRUCOES:                                    |"));
      Serial.println(F("| 1. Use o BOTAO para alternar os estados.       |"));
      Serial.println(F("|    [0] PRONTO -> [1] GRAVAR -> [2] PLAY        |"));
      Serial.println(F("| 2. Mova os Pots para ensinar o movimento.      |"));
      Serial.println(F("+================================================+"));
      Serial.println(F(""));

      lcd.clear();
      lcd2.clear();

      posAtual1 = stepper.currentPosition();
      posAtual2 = stepper2.currentPosition();
      posAtual3 = stepper3.currentPosition();
      posAtual4 = stepperA.currentPosition();

      resetarInercia();

      if (estado == 0) {
        lcd2.clear();
        lcd2.setCursor(0, 0);
        lcd2.print("Modo GHOST ativo");
        lcd2.setCursor(0, 1);
        lcd2.print("nos motores de ");
        lcd2.setCursor(0, 2);
        lcd2.print("passo");
        lcd.setCursor(0, 0);
        lcd.print("Pronto p/ gravar");
      }

      return;

    } else if ((comando.equals("SERVO") || comando.equals("MOTORES SERVO") || comando.equals("SERVOS") || comando.equals("SERVO MOTORES")) && (ModoAtual == MODO_GHOST_MOTORES || ModoAtual == MODO_GHOST_PASSOS)) {

      meuServo.attach(34);
      meuServo2.attach(36);

      estado = 0;

      motoresDesativados = true;
      ModoAtual = MODO_GHOST_SERVOS;

      Serial.println(F(""));
      Serial.println(F("+================================================+"));
      Serial.println(F("|    >>> GHOST MODE (MEMORIA DE MOVIMENTO) <<<   |"));
      Serial.println(F("+================================================+"));
      Serial.println(F("| STATUS:  Modulo de Gravacao INICIADO           |"));
      Serial.println(F("| ALVO:    MOTORES DE PASSO (STEPPERS) [ ]       |"));
      Serial.println(F("|          SERVO MOTORES               [X]       |"));
      Serial.println(F("|------------------------------------------------|"));
      Serial.println(F("| INSTRUCOES:                                    |"));
      Serial.println(F("| 1. Use o BOTAO para alternar os estados.       |"));
      Serial.println(F("|    [0] PRONTO -> [1] GRAVAR -> [2] PLAY        |"));
      Serial.println(F("| 2. Mova os Pots para ensinar o movimento.      |"));
      Serial.println(F("+================================================+"));
      Serial.println(F(""));

      lcd.clear();
      lcd2.clear();

      resetarInercia();

      if (estado == 0) {
        lcd2.clear();
        lcd2.setCursor(0, 0);
        lcd2.print("Modo GHOST ativo");
        lcd2.setCursor(0, 1);
        lcd2.print("nos servos");
        lcd2.setCursor(0, 2);
        lcd2.print("Pronto p/ gravar");
      }

      return;

    } else if (comando.equals("MODO MOTOR DE VELOCIDADE") || comando.equals("MOTOR CONSTANTE") || comando.equals("MVC")) {

      meuServo.detach();
      meuServo2.detach();

      motoresDesativados = true;
      ModoAtual = MOTOR_VELOCIDADE;

      Serial.println(F(""));
      Serial.println(F("+================================================+"));
      Serial.println(F("|    >>> MODO MOTOR DE VELOCIDADE (MVC) <<<      |"));
      Serial.println(F("+================================================+"));
      Serial.println(F("| STATUS:  Ponte H Energizada e Pronta.          |"));
      Serial.println(F("| ALVO:    MOTOR DC AUXILIAR (BASE)              |"));
      Serial.println(F("|------------------------------------------------|"));
      Serial.println(F("| COMANDOS DE OPERACAO:                          |"));
      Serial.println(F("| > BOTAO 1 : PARAR MOTOR (EMERGENCIA)           |"));
      Serial.println(F("| > BOTAO 2 : GIRAR PARA ESQUERDA (L)            |"));
      Serial.println(F("| > BOTAO 3 : GIRAR PARA DIREITA (R)             |"));
      Serial.println(F("|------------------------------------------------|"));
      Serial.println(F("| DICA: Use os botoes para trocar a polaridade   |"));
      Serial.println(F("|       da Ponte H instantaneamente.             |"));
      Serial.println(F("+================================================+"));
      Serial.println(F(""));

      resetarInercia();

      lcd.clear();
      lcd2.clear();

      lcd.setCursor(0, 0);
      lcd.print("Motor: PARADO");

      lcd2.setCursor(0, 0);
      lcd2.print("Modo motor de");
      lcd2.setCursor(0, 1);
      lcd2.print("velocidade ");
      lcd2.setCursor(0, 2);
      lcd2.print("constante ativo");

      return;

    } else {
      if (ModoAtual == MODO_SERVO_SERIAL) {
        execSerialServo(input);

      } else if (ModoAtual == MODO_PASSO_SERIAL) {
        execPassosSerial(input);

      } else if (ModoAtual == TRACADO) {
        execPassosTracado(input);

      } else if (ModoAtual == IA_F1) {
        execIAF1(input);

      } else {
        Serial.println();
        Serial.println(F("==== Comando ou solicitação errada! ===="));
      }
      return;
    }
  }

  digitalWrite(8, !motoresDesativados ? LOW : HIGH);

  if (ModoAtual == MODO_SERVO_POT) {
    execServoPot();

  } else if (ModoAtual == MODO_PASSO_POT) {
    execPassoPot();

  } else if (ModoAtual == MODO_VELOCIDADE_PASSOS) {
    execVelocidade();

  } else if (ModoAtual == MODO_MEDIDOR) {
    execMedidor();

  } else if (ModoAtual == MODO_DISTANCIA_SERVOS) {
    execModoServosUltrassonico();

  } else if (ModoAtual == SEMAFORO_DE_F1) {
    execSemaforoF1();

  } else if (ModoAtual == IA_F1) {
    execIAF1(input);

  } else if (ModoAtual == MODO_GHOST_PASSOS) {
    ghostPassos();

  } else if (ModoAtual == MODO_GHOST_SERVOS) {
    ghostServos();

  } else if (ModoAtual == AJUDA) {
    motoresDesativados = true;

    meuServo.detach();
    meuServo2.detach();


  } else if (ModoAtual == MANUAL) {
    motoresDesativados = true;

    meuServo.detach();
    meuServo2.detach();

  } else if (ModoAtual == OFF) {
    execInativo();

  } else if (ModoAtual == ON) {
    execAtivo();

  } else if (ModoAtual == MODO_MARCHAS) {
    execMarchas();

  } else if (ModoAtual == TRACADO) {
    execPassosTracado(input);

  } else if (ModoAtual == MOTOR_VELOCIDADE) {
    execMotorVel();
  }
}

void TrocaDeModoBotao() {
  static byte estadoOffline = 0;
  static byte ultimoEstado;
  static unsigned int tempoAp = 0;
  const byte delayTemp = 80;

  byte estadoBotao = digitalRead(botao4);
  if (estadoBotao == LOW && ultimoEstado == HIGH && (millis() - tempoAp > delayTemp)) {
    tempoAp = millis();
    indiceModosOffline++;

    desligarMotorVel();
    resetarInercia();

    lcd.clear();
    lcd2.clear();

    ModoAtual = listaModosOffline[indiceModosOffline];

    switch (ModoAtual) {

      case MODO_SERVO_POT:
        meuServo.attach(34);
        meuServo2.attach(36);
        motoresDesativados = true;

        lcd2.print("Modo: SERVO POT");
        lcd2.setCursor(0, 1);
        lcd2.print("Manual (Pots)");

        break;

      case MODO_PASSO_POT:
        meuServo.detach();
        meuServo2.detach();
        motoresDesativados = false;

        lcd2.print("Modo: PASSO POT");
        lcd2.setCursor(0, 1);
        lcd2.print("Posicao (Pots)");

        break;

      case MODO_VELOCIDADE_PASSOS:
        meuServo.detach();
        meuServo2.detach();
        motoresDesativados = false;

        lcd2.print("Modo: VELOCIDADE");
        lcd2.setCursor(0, 1);
        lcd2.print("Continuo (RPM)");
        break;

      case MODO_MEDIDOR:
        meuServo.detach();
        meuServo2.detach();
        motoresDesativados = true;

        lcd2.print("Modo: MEDIDOR");

        break;

      case MODO_MARCHAS:
        meuServo.detach();
        meuServo2.detach();
        motoresDesativados = false;

        lcd2.print("Modo: MARCHAS");
        lcd2.setCursor(0, 1);
        lcd2.print("Simulador H");
        lcd.print("Marcha: 1");
        break;

      case SEMAFORO_DE_F1:
        meuServo.detach();
        meuServo2.detach();
        motoresDesativados = true;

        state = IDLE;
        buttonPressed = false;

        lcd2.setCursor(0, 0);
        lcd2.print("Semaforo de F1");
        lcd2.setCursor(0, 1);
        lcd2.print("ativado");

        break;

      case MOTOR_VELOCIDADE:
        meuServo.detach();
        meuServo2.detach();
        motoresDesativados = true;

        lcd2.print("Modo: PONTE H");
        lcd2.setCursor(0, 1);
        lcd2.print("Motor DC Aux");
        break;
    }
  }
  ultimoEstado = estadoBotao;
  if (indiceModosOffline >= totalDeModosOffline) indiceModosOffline = 0;
}

void execSerialServo(String input) {

  int indiceEspaco = input.indexOf(' ');

  if (indiceEspaco == -1) {
    lcd.clear();
    Serial.println(F(""));
    Serial.println(F("ERRO: Comando incompleto [Ex: A 90]"));
    lcd.setCursor(0, 0);
    lcd.print("ERRO: comando  ");
    lcd.setCursor(0, 1);
    lcd.print("incompleto");
    return;
  }


  String servoID = input.substring(0, indiceEspaco);
  String angulo = input.substring(indiceEspaco + 1);

  servoID.trim();
  servoID.toUpperCase();
  angulo.trim();

  byte posicao = angulo.toInt();

  if (posicao < 0 || posicao > 180) {
    lcd.clear();
    Serial.println(F(""));
    Serial.println(F("ERRO: Angulo invalido (use 0-180)."));
    lcd.setCursor(0, 0);
    lcd.print("ERRO: Angulo");
    lcd.setCursor(0, 1);
    lcd.print("invalido");
    return;

  } else if (servoID.equals("A")) {
    lcd.clear();
    Serial.println(F(""));
    Serial.println(F("Movendo motor A para:"));
    Serial.print(posicao);
    Serial.println(F("°"));
    meuServo.write(posicao);
    lcd.setCursor(0, 0);
    lcd.print("Movendo Motor A: ");
    lcd.setCursor(0, 1);
    lcd.print(posicao);
    lcd.print((char)223);
    lcd.print(" ");
    return;

  } else if (servoID.equals("B")) {
    lcd.clear();
    Serial.println(F(""));
    Serial.println(F("Movendo motor B para:"));
    Serial.print(posicao);
    Serial.println(F("°"));
    meuServo2.write(posicao);
    lcd.setCursor(0, 0);
    lcd.print("Movendo Motor B: ");
    lcd.setCursor(0, 1);
    lcd.print(posicao);
    lcd.print((char)223);
    lcd.print(" ");
    return;

  } else {
    lcd.clear();
    Serial.println(F(""));
    Serial.print(F("ERRO: Servo '"));
    Serial.print(servoID);
    Serial.println(F("' nao reconhecido. Use 'A' ou 'B'"));
    lcd.setCursor(0, 0);
    lcd.print("Servo nao");
    lcd.setCursor(0, 1);
    lcd.print("reconhecido");
    return;
  }
}

void execServoPot() {

  static unsigned long tempoAnterior = 0;
  const byte delayTempoServos = 20;

  int val1 = analogRead(pot1);
  int val2 = analogRead(pot2);

  byte potPosition1 = map(val1, 0, 1023, 0, 180);
  byte potPosition2 = map(val2, 0, 1023, 0, 180);

  if ((millis() - tempoAnterior) > delayTempoServos) {
    tempoAnterior = millis();

    meuServo.write(potPosition1);

    if (potPosition1 != ultimaPosServo1) {

      lcd.setCursor(0, 0);
      lcd.print("Motor 1:      ");
      lcd.setCursor(0, 1);
      lcd.print(potPosition1);
      lcd.print((char)223);
      lcd.print(" ");

      ultimaPosServo1 = potPosition1;
    }

    meuServo2.write(potPosition2);

    if (potPosition2 != ultimaPosServo2) {

      lcd.setCursor(0, 2);
      lcd.print("Motor 2:       ");
      lcd.setCursor(0, 3);
      lcd.print(potPosition2);
      lcd.print((char)223);
      lcd.print(" ");
      ultimaPosServo2 = potPosition2;
    }
  }
}

void execPassosSerial(String input) {

  stepper.setAcceleration(800);
  stepper2.setAcceleration(800);
  stepper3.setAcceleration(800);
  stepperA.setAcceleration(800);

  int indiceEspaco = input.indexOf(' ');

  if (indiceEspaco == -1) {
    lcd.clear();
    Serial.println(F(""));
    Serial.println(F("ERRO: Comando incompleto ou errado. (Ex: M1 200)"));
    lcd.setCursor(0, 0);
    lcd.print("ERRO: Comando ");
    lcd.setCursor(0, 1);
    lcd.print("incompleto ou errado");
    return;
  }

  String MotorID = input.substring(0, indiceEspaco);
  String MotorPosition = input.substring(indiceEspaco + 1);

  MotorID.trim();
  MotorID.toUpperCase();
  MotorPosition.trim();

  passos = MotorPosition.toInt();

  if (MotorID.equals("M1")) {
    lcd.clear();
    Serial.println(F(""));
    Serial.println(F("Movendo M1 para:"));
    Serial.print(passos);
    lcd.setCursor(0, 0);
    lcd.print("Movendo M1: ");
    lcd.setCursor(0, 1);
    lcd.print(passos);
    stepper.runToNewPosition(passos);
    return;

  } else if (MotorID.equals("M2")) {
    lcd.clear();
    Serial.println(F(""));
    Serial.println(F("Movendo M2 para:"));
    Serial.print(passos);
    lcd.setCursor(0, 0);
    lcd.print("Movendo M2: ");
    lcd.setCursor(0, 1);
    lcd.print(passos);
    stepper2.runToNewPosition(passos);
    return;

  } else if (MotorID.equals("M3")) {
    lcd.clear();
    Serial.println(F(""));
    Serial.println(F("Movendo M3 para:"));
    Serial.print(passos);
    lcd.setCursor(0, 0);
    lcd.print("Movendo M3: ");
    lcd.setCursor(0, 1);
    lcd.print(passos);
    stepper3.runToNewPosition(passos);
    return;

  } else if (MotorID.equals("M4")) {
    lcd.clear();
    Serial.println(F(""));
    Serial.println(F("Movendo M4 para:"));
    Serial.print(passos);
    lcd.setCursor(0, 0);
    lcd.print("Movendo M4: ");
    lcd.setCursor(0, 1);
    lcd.print(passos);
    stepperA.runToNewPosition(passos);
    return;

  } else {
    lcd.clear();
    Serial.println(F(""));
    Serial.println(F("Motor invalido ou não reconhecido"));
    lcd.setCursor(0, 0);
    lcd.print("ERRO: Motor nao ");
    lcd.setCursor(0, 1);
    lcd.print("reconhecido ");
    return;
  }
}

void execPassoPot() {

  stepper.setAcceleration(4000);
  stepper2.setAcceleration(4000);
  stepper3.setAcceleration(4000);
  stepperA.setAcceleration(4000);

  int potValue1 = analogRead(pot1);
  int potValue2 = analogRead(pot2);
  int potValue3 = analogRead(pot3);
  int potValue4 = analogRead(pot4);

  int position1 = map(potValue1, 0, 1023, posAtual1, posAtual1 + 200);
  int position2 = map(potValue2, 0, 1023, posAtual2, posAtual2 + 200);
  int position3 = map(potValue3, 0, 1023, posAtual3, posAtual3 + 200);
  int position4 = map(potValue4, 0, 1023, posAtual4, posAtual4 + 200);

  byte deadband1 = 2;
  byte deadband2 = 2;
  byte deadband3 = 2;
  byte deadband4 = 2;

  if (abs(position1 - lastPosition1) > deadband1) {
    stepper.moveTo(position1);
    lastPosition1 = position1;
  }
  if (abs(position2 - lastPosition2) > deadband2) {
    stepper2.moveTo(position2);
    lastPosition2 = position2;
  }
  if (abs(position3 - lastPosition3) > deadband3) {
    stepper3.moveTo(position3);
    lastPosition3 = position3;
  }
  if (abs(position4 - lastPosition4) > deadband4) {
    stepperA.moveTo(position4);
    lastPosition4 = position4;
  }
}

void execPassosTracado(String input) {
  if (Aceleracao == 0) {
    if (input.length() > 0) {
      int indiceEspaco = input.indexOf(' ');

      String acel = input.substring(0, indiceEspaco);
      acel.trim();
      Aceleracao = acel.toInt();

      stepper.setAcceleration(Aceleracao);
      stepper2.setAcceleration(Aceleracao);
      stepper3.setAcceleration(Aceleracao);
      stepperA.setAcceleration(Aceleracao);

      if (Aceleracao != 0) {
        Serial.println(F("\n+--------------------------------------------------+"));
        Serial.print(F("| CONFIGURADO: Aceleração definida para: "));
        Serial.print(Aceleracao);

        if (Aceleracao < 100) Serial.println("        |");
        else if (Aceleracao < 1000) Serial.println("       |");
        else if (Aceleracao < 10000) Serial.println("      |");
        else if (Aceleracao < 100000) Serial.println("     |");
        else if (Aceleracao < 1000000) Serial.println("    |");

        Serial.println(F("| > O sistema está pronto. Use o BOTAO 1 (REC/PLAY)|"));
        Serial.println(F("+--------------------------------------------------+"));
        return;

      } else {
        Serial.println(F("\n[ERRO] Valor invalido! Digite um numero maior que 0."));
      }
    }

  } else {

    byte estadoBotaoReset = digitalRead(botao2);
    if (estadoBotaoReset == LOW && Aceleracao != 0) {
      Aceleracao = 0;

      lcd.clear();
      lcd.print(">> ACEL RESET <<");
      lcd.setCursor(0, 1);
      lcd.print("Digite nova acel");

      Serial.println(F("\n+------------------------------------------------+"));
      Serial.println(F("| [SYSTEM] Aceleração RESETADA pelo usuário!     |"));
      Serial.println(F("| > Digite o novo valor no Serial (Ex: 1000)...  |"));
      Serial.println(F("+------------------------------------------------+"));

      delay(500);
      return;
    }

    if (Aceleracao == 0) {
      if (input.length() > 0) {

        long novaAcel = input.toInt();

        if (novaAcel > 0) {
          Aceleracao = novaAcel;

          stepper.setAcceleration(Aceleracao);
          stepper2.setAcceleration(Aceleracao);
          stepper3.setAcceleration(Aceleracao);
          stepperA.setAcceleration(Aceleracao);

          Serial.println(F("\n+--------------------------------------------------+"));
          Serial.print(F("| CONFIGURADO: Aceleração definida para: "));
          Serial.println(Aceleracao);
          Serial.println(F("| > O sistema está pronto. Use o BOTAO 1 (REC/PLAY)|"));
          Serial.println(F("+--------------------------------------------------+"));


          lcd.clear();
          lcd.print("Acel: ");
          lcd.print(Aceleracao);
          lcd.setCursor(0, 1);
          lcd.print("Pronto p/ gravar");

        } else {
          Serial.println(F("\n[ERRO] Valor invalido! Digite um numero maior que 0."));
        }
      }
      return;
    }
    static unsigned long tempoApTracado = 0;
    static byte ultimoEstado;
    const byte debounceDelay = 80;

    byte estadoBotao = digitalRead(botao1);
    if (estadoBotao == LOW && ultimoEstado == HIGH && (millis() - tempoApTracado > debounceDelay)) {
      estado++;
      tempoApTracado = millis();
      if (estado == 3) estado = 0;

      if (estado == 1) {
        totalGravado = 0;
        pegarvalor1 = 0;
        lcd.clear();
        lcd.print(">> MODO REC <<");
        lcd.setCursor(0, 1);
        lcd.print("Envie Comandos...");

        Serial.println(F("\n+------------------------------------------------+"));
        Serial.println(F("| [REC] INICIANDO COLETA DE DADOS...             |"));
        Serial.println(F("| > Digite: 'M1 500', 'M2 1000', 'M3 -200'...    |"));
        Serial.println(F("+------------------------------------------------+"));
      }

      if (estado == 2) {
        pegarvalor1 = 0;

        lcd.clear();
        lcd.print(">> EXECUTANDO <<");
        lcd.setCursor(0, 1);
        lcd.print("Motores se movendo");
        lcd.setCursor(0, 2);
        lcd.print("para as posicoes");

        Serial.println(F("\n+------------------------------------------------+"));
        Serial.println(F("| [PLAY] INICIANDO SEQUENCIA AUTOMATICA...       |"));
        Serial.println(F("| > Status: Motores em movimento. AFASTE-SE!     |"));
        Serial.println(F("+------------------------------------------------+"));
        Serial.print(F(">> Total de Passos na Fila: "));
        Serial.println(totalGravado);
      }

      if (estado == 0) {
        totalGravado = 0;
        pegarvalor1 = 0;
        if (ghostMP1[0] != 0 || ghostMP2[0] != 0 || ghostMP3[0] != 0 || ghostMP4[0] != 0) {
          for (int i = 0; i < MAX_FRAMES2; i++) {
            ghostMP1[i] = 0;
            ghostMP2[i] = 0;
            ghostMP3[i] = 0;
            ghostMP4[i] = 0;
          }
        }
        lcd.clear();
        resetarInercia();
        lcd.print(">> STANDBY <<");
        lcd.setCursor(0, 1);
        lcd.print("Pronto p/ Gravar");

        Serial.println(F("\n."));
        Serial.println(F(">> [STANDBY] MODO TRAÇADO EM PAUSA (Aguardando...)"));
        Serial.print(F(">> [SUGESTÃO] Digite 'CARREGAR 1' para carregar a primeira pasta salva na memoria"));
      }
    }

    ultimoEstado = estadoBotao;

    if (estado == 0) {
      if (input.length() > 0) {
        int indiceEspaco = input.indexOf(' ');
        String comandoCarregar = input.substring(0, indiceEspaco);

        if (indiceEspaco == -1) {
          lcd.clear();
          Serial.println(F(""));
          Serial.println(F("ERRO: Comando incompleto ou errado. (Ex: CARREGAR 1)"));
          lcd.setCursor(0, 0);
          lcd.print("ERRO: Comando ");
          lcd.setCursor(0, 1);
          lcd.print("incompleto ou errado");
          return;
        }

        comandoCarregar.trim();
        comandoCarregar.toUpperCase();

        if (comandoCarregar.equals("CARREGAR")) {

          String pasta = input.substring(indiceEspaco + 1);
          int numeroPasta = pasta.toInt();
          carregarDaPasta(numeroPasta);
          estado = 2;
          return;
        }
      }
    }

    if (estado == 1) {
      if (input.length() > 0) {
        if (totalGravado < MAX_FRAMES2) {

          int indiceEspaco = input.indexOf(' ');

          if (indiceEspaco == -1) {
            lcd.clear();
            Serial.println(F(""));
            Serial.println(F("ERRO: Comando incompleto ou errado. (Ex: M1 200)"));
            lcd.setCursor(0, 0);
            lcd.print("ERRO: Comando ");
            lcd.setCursor(0, 1);
            lcd.print("incompleto ou errado");
            return;
          }

          String MotorID = input.substring(0, indiceEspaco);
          String comandoSalvar = input.substring(0, indiceEspaco);
          String comandoCarregar = input.substring(0, indiceEspaco);
          String MotorPosition = input.substring(indiceEspaco + 1);

          MotorID.trim();
          MotorID.toUpperCase();
          comandoSalvar.trim();
          comandoSalvar.toUpperCase();
          MotorPosition.trim();

          long passosLidos = MotorPosition.toInt();

          if (passosLidos == 0) passosLidos = 1;

          bool comandoValido = false;

          if (MotorID.equals("M1")) {
            lcd.clear();
            ghostMP1[totalGravado] = passosLidos;
            Serial.println(F(""));
            Serial.println(F("Posição salva para M1: "));
            Serial.print(passosLidos);
            comandoValido = true;

          } else if (MotorID.equals("M2")) {
            lcd.clear();
            ghostMP2[totalGravado] = passosLidos;
            Serial.println(F(""));
            Serial.println(F("Posição salva para M2: "));
            Serial.print(passosLidos);
            comandoValido = true;

          } else if (MotorID.equals("M3")) {
            lcd.clear();
            ghostMP3[totalGravado] = passosLidos;
            Serial.println(F(""));
            Serial.println(F("Posição salva para M3: "));
            Serial.print(passosLidos);
            comandoValido = true;

          } else if (MotorID.equals("M4")) {
            lcd.clear();
            ghostMP4[totalGravado] = passosLidos;
            Serial.println(F(""));
            Serial.println(F("Posição salva para M4: "));
            Serial.print(passosLidos);
            comandoValido = true;

          } else if (comandoSalvar.equals("SALVAR")) {

            String pasta = input.substring(indiceEspaco + 1);
            int numeroPasta = pasta.toInt();
            salvarNaPasta(numeroPasta);
            return;

          } else if (comandoCarregar.equals("CARREGAR")) {

            String pasta = input.substring(indiceEspaco + 1);
            int numeroPasta = pasta.toInt();
            carregarDaPasta(numeroPasta);
            estado = 2;
            return;

          } else {
            lcd.clear();
            Serial.println(F(""));
            Serial.println(F("Motor invalido ou não reconhecido"));
            lcd.setCursor(0, 0);
            lcd.print("ERRO: Motor nao ");
            lcd.setCursor(0, 1);
            lcd.print("reconhecido ");
          }

          if (comandoValido) {

            lcd.setCursor(0, 0);
            lcd.print("Ultimo: ");
            lcd.print(MotorID);
            lcd.print(" ");
            lcd.print(passosLidos);
            lcd.print("   ");

            totalGravado++;
            Serial.println("");
            Serial.print(F("Passos Gravados: "));
            Serial.println(totalGravado);
          }

        } else {
          lcd.setCursor(0, 1);
          lcd.print("MEMORIA CHEIA!  ");
        }
      }
    }

    else if (estado == 2) {
      if (pegarvalor1 < totalGravado) {

        long alvo1 = ghostMP1[pegarvalor1];
        long alvo2 = ghostMP2[pegarvalor1];
        long alvo3 = ghostMP3[pegarvalor1];
        long alvo4 = ghostMP4[pegarvalor1];

        if (alvo1 != 0) stepper.runToNewPosition(alvo1);
        if (alvo2 != 0) stepper2.runToNewPosition(alvo2);
        if (alvo3 != 0) stepper3.runToNewPosition(alvo3);
        if (alvo4 != 0) stepperA.runToNewPosition(alvo4);

        pegarvalor1++;


      } else {

        lcd.setCursor(0, 0);
        lcd.print("FIM DA SEQUENCIA");
        lcd.setCursor(0, 1);
        lcd.print("                    ");
        lcd.setCursor(0, 2);
        lcd.print("                    ");
        lcd.setCursor(0, 3);
        lcd.print("                    ");
      }
    }
  }
}

void salvarNaPasta(int numeroPasta) {
  if (numeroPasta < 1 || numeroPasta > 12) {
    Serial.println(F("\n[ERRO] Pasta invalida! Use de 1 a 12."));
    return;
  }

  int enderecoBase = (numeroPasta - 1) * TAMANHO_PASTA;

  Serial.print(F(">> Salvando na Pasta "));
  Serial.print(numeroPasta);
  Serial.println(F("..."));

  EEPROM.put(enderecoBase, totalGravado);
  EEPROM.put(enderecoBase + 2, Aceleracao);

  int enderecoAtual = enderecoBase + 6;

  for (int i = 0; i < MAX_FRAMES2; i++) {
    EEPROM.put(enderecoAtual, ghostMP1[i]);
    enderecoAtual += 4;

    EEPROM.put(enderecoAtual, ghostMP2[i]);
    enderecoAtual += 4;

    EEPROM.put(enderecoAtual, ghostMP3[i]);
    enderecoAtual += 4;

    EEPROM.put(enderecoAtual, ghostMP4[i]);
    enderecoAtual += 4;
  }

  Serial.println(F("\n>> [SUCESSO] Traçado salvo na EEPROM!"));
}

void carregarDaPasta(int numeroPasta) {
  if (numeroPasta < 1 || numeroPasta > 12) {
    Serial.println(F("\n[ERRO] Pasta invalida! Use de 1 a 12."));
    return;
  }

  int enderecoBase = (numeroPasta - 1) * TAMANHO_PASTA;

  Serial.print(F(">> Carregando Pasta "));
  Serial.print(numeroPasta);
  Serial.println(F("..."));

  EEPROM.get(enderecoBase, totalGravado);
  EEPROM.get(enderecoBase + 2, Aceleracao);

  if (Aceleracao > 0) {
    stepper.setAcceleration(Aceleracao);
    stepper2.setAcceleration(Aceleracao);
    stepper3.setAcceleration(Aceleracao);
    stepperA.setAcceleration(Aceleracao);
  }

  int enderecoAtual = enderecoBase + 6;

  for (int i = 0; i < MAX_FRAMES2; i++) {
    EEPROM.get(enderecoAtual, ghostMP1[i]);
    enderecoAtual += 4;

    EEPROM.get(enderecoAtual, ghostMP2[i]);
    enderecoAtual += 4;

    EEPROM.get(enderecoAtual, ghostMP3[i]);
    enderecoAtual += 4;

    EEPROM.get(enderecoAtual, ghostMP4[i]);
    enderecoAtual += 4;
  }

  Serial.print(F("\n>> [SUCESSO] Carregados "));
  Serial.print(totalGravado);
  Serial.println(F(" passos da memória."));
  Serial.print(F(">> Aceleração restaurada: "));
  Serial.println(Aceleracao);

  lcd.clear();
  lcd.print(">> EXECUTANDO <<");
  lcd.setCursor(0, 1);
  lcd.print("Motores se movendo");
  lcd.setCursor(0, 2);
  lcd.print("para as posicoes");

  Serial.println(F("\n+------------------------------------------------+"));
  Serial.println(F("| [PLAY] INICIANDO SEQUENCIA AUTOMATICA...       |"));
  Serial.println(F("| > Status: Motores em movimento. AFASTE-SE!     |"));
  Serial.println(F("+------------------------------------------------+"));
}

void listarPastas() {
  int contadorOcupadas = 0;

  Serial.println(F("\n+================================================+"));
  Serial.println(F("|        >>> GERENCIADOR DE ARQUIVOS <<<         |"));
  Serial.println(F("+================================================+"));
  Serial.println(F("| PASTA |  STATUS   |  PASSOS  |   ACELERACAO    |"));
  Serial.println(F("|-------|-----------|----------|-----------------|"));

  for (int i = 1; i <= 12; i++) {

    int enderecoBase = (i - 1) * TAMANHO_PASTA;

    int passosLidos;
    long acelLida;

    EEPROM.get(enderecoBase, passosLidos);
    EEPROM.get(enderecoBase + 2, acelLida);

    if (passosLidos > 0 && passosLidos <= MAX_FRAMES2) {

      Serial.print(F("|  "));
      if (i < 10) Serial.print("0");
      Serial.print(i);
      Serial.print(F("   |  OCUPADA  |    "));

      if (passosLidos < 10) Serial.print("0");
      Serial.print(passosLidos);
      Serial.print(F("    |      "));

      Serial.print(acelLida);
      Serial.println(F("       |"));

      contadorOcupadas++;

    } else {
      Serial.print(F("|  "));
      if (i < 10) Serial.print("0");
      Serial.print(i);
      Serial.println(F("   |   LIVRE   |    --    |        --       |"));
    }
  }

  Serial.println(F("+================================================+"));
  Serial.print(F("| ESPACO USADO: "));
  Serial.print(contadorOcupadas);
  Serial.println(F("/12 Slots.                        |"));
  Serial.println(F("+================================================+"));

  if (contadorOcupadas == 0) {
    Serial.println(F("\n>> [AVISO] Nenhuma gravação encontrada na memoria."));
  }
}

void execMarchas() {

  static byte marchas = 1;
  const byte marchasMax = 6;
  static byte ultimaLeituraMais = HIGH;
  static byte ultimaLeituraMenos = HIGH;
  static int marchaAnterior = -1;
  static unsigned long ultimoCorte = 0;
  static bool emCorte = false;

  byte leituraMais = digitalRead(botao2);
  byte leituraMenos = digitalRead(botao3);

  if (leituraMais == LOW && ultimaLeituraMais == HIGH) {
    if (marchas < marchasMax) marchas++;
  }

  if (leituraMenos == LOW && ultimaLeituraMenos == HIGH) {
    if (marchas > 0) marchas--;
  }

  ultimaLeituraMais = leituraMais;
  ultimaLeituraMenos = leituraMenos;

  int valor = analogRead(pot1);
  int acel = map(valor, 0, 1023, 0, limites[marchas]);

  if (marchas == 0) {
    if (acel >= limites[0] - 100) {
      if (millis() - ultimoCorte > 90) {
        ultimoCorte = millis();
        int corte = random(limites[0], limites[0] - 350);
        stepper.setSpeed(corte);
        stepper2.setSpeed(corte);
        stepper3.setSpeed(corte);
        stepperA.setSpeed(corte);
      }
    } else {
      stepper.setSpeed(acel);
      stepper2.setSpeed(acel);
      stepper3.setSpeed(acel);
      stepperA.setSpeed(acel);
    }
  } else {
    stepper.setSpeed(acel);
    stepper2.setSpeed(acel);
    stepper3.setSpeed(acel);
    stepperA.setSpeed(acel);
  }

  stepper.setAcceleration(1);
  stepper2.setAcceleration(1);
  stepper3.setAcceleration(1);
  stepperA.setAcceleration(1);

  stepper.runSpeed();
  stepper2.runSpeed();
  stepper3.runSpeed();
  stepperA.runSpeed();

  if (marchas != marchaAnterior) {
    lcd.setCursor(0, 0);
    lcd.print("Marcha: ");

    if (marchas == 0) {
      lcd.print("N ");
    } else {
      lcd.print(marchas);
    }
    marchaAnterior = marchas;
  }
}

void ghostPassos() {

  static byte ultimoEstado;

  byte estadoBotao = digitalRead(botao1);
  if (estadoBotao == LOW && ultimoEstado == HIGH && (millis() - tempoApGhost > delayGhost)) {
    estado++;
    tempoApGhost = millis();
    if (estado == 3) estado = 0;

    if (estado == 1) {
      totalGravado = 0;
      pegarvalor1 = 0;
      lcd.clear();
      lcd.print("GRAVANDO...");
      Serial.println(F(""));
      Serial.println(F(">> [REC] INICIANDO GRAVACAO..."));
      Serial.println(F(">> Mova os potenciometros agora!"));
    }

    if (estado == 2) {
      pegarvalor1 = 0;
      lcd.clear();
      lcd.print("REPRODUZINDO...");
      lcd.setCursor(0, 1);
      lcd.print("pressione outra vez");
      lcd.setCursor(0, 2);
      lcd.print("para anular essa");
      lcd.setCursor(0, 3);
      lcd.print("gravacao");
      Serial.println(F(""));
      Serial.println(F(">> [PLAY] REPRODUZINDO MOVIMENTOS..."));
      Serial.println(F(">> Pressione outra vez para anular essa gravação"));
      Serial.print(F(">> Frames Totais: "));
      Serial.println(totalGravado);
    }

    if (estado == 0) {
      totalGravado = 0;
      pegarvalor1 = 0;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Pronto p/ gravar");
      Serial.println(F(""));
      Serial.println(F(">> [STOP] MODO FANTASMA EM ESPERA."));
      Serial.println(F(">> Pressione o botao para gravar."));
    }
  }
  ultimoEstado = estadoBotao;

  if (estado == 0) {
    execPassoPot();
  }

  if (estado == 1) {
    if (millis() - tempoRecord > 30) {

      if (totalGravado < MAX_FRAMES) {
        stepper.setAcceleration(4000);
        stepper2.setAcceleration(4000);
        stepper3.setAcceleration(4000);
        stepperA.setAcceleration(4000);

        int potValue1 = analogRead(pot1);
        int potValue2 = analogRead(pot2);
        int potValue3 = analogRead(pot3);
        int potValue4 = analogRead(pot4);

        int position1 = map(potValue1, 0, 1023, posAtual1, posAtual1 + 200);
        int position2 = map(potValue2, 0, 1023, posAtual2, posAtual2 + 200);
        int position3 = map(potValue3, 0, 1023, posAtual3, posAtual3 + 200);
        int position4 = map(potValue4, 0, 1023, posAtual4, posAtual4 + 200);

        byte deadband1 = 2;
        byte deadband2 = 2;
        byte deadband3 = 2;
        byte deadband4 = 2;

        if (abs(position1 - lastPosition1) > deadband1) {
          stepper.moveTo(position1);
          lastPosition1 = position1;
        }
        if (abs(position2 - lastPosition2) > deadband2) {
          stepper2.moveTo(position2);
          lastPosition2 = position2;
        }
        if (abs(position3 - lastPosition3) > deadband3) {
          stepper3.moveTo(position3);
          lastPosition3 = position3;
        }
        if (abs(position4 - lastPosition4) > deadband4) {
          stepperA.moveTo(position4);
          lastPosition4 = position4;
        }

        ghostM1[totalGravado] = position1;
        ghostM2[totalGravado] = position2;
        ghostM3[totalGravado] = position3;
        ghostM4[totalGravado] = position4;

        totalGravado++;

      } else {
        lcd.setCursor(0, 1);
        lcd.print("MEMORIA CHEIA!  ");
      }
      tempoRecord = millis();
    }
  }

  else if (estado == 2) {
    if (millis() - frameRate > 30) {

      if (pegarvalor1 < totalGravado) {

        byte alvo1 = ghostM1[pegarvalor1];
        byte alvo2 = ghostM2[pegarvalor1];
        byte alvo3 = ghostM3[pegarvalor1];
        byte alvo4 = ghostM4[pegarvalor1];

        if (alvo1 <= 5) alvo1 = 0;
        if (alvo2 <= 5) alvo2 = 0;
        if (alvo3 <= 5) alvo3 = 0;
        if (alvo4 <= 5) alvo4 = 0;

        stepper.moveTo(alvo1);
        stepper2.moveTo(alvo2);
        stepper3.moveTo(alvo3);
        stepperA.moveTo(alvo4);

        pegarvalor1++;

      } else {

        lcd.setCursor(0, 0);
        lcd.print("FIM DO PLAYBACK  ");
        lcd.setCursor(0, 1);
        lcd.print("                    ");
        lcd.setCursor(0, 2);
        lcd.print("                    ");
        lcd.setCursor(0, 3);
        lcd.print("                    ");
      }
      frameRate = millis();
    }
  }
}

void ghostServos() {

  static unsigned long tempoAnterior = 0;
  const byte delayTempoServos = 20;
  static byte ultimoEstado;

  byte estadoBotao = digitalRead(botao1);
  if (estadoBotao == LOW && ultimoEstado == HIGH && (millis() - tempoApGhost > delayGhost)) {
    estado++;
    tempoApGhost = millis();
    if (estado == 3) estado = 0;

    if (estado == 1) {
      totalGravado = 0;
      pegarvalor1 = 0;
      lcd2.clear();
      lcd2.print("GRAVANDO...");
      Serial.println(F(""));
      Serial.println(F(">> [REC] INICIANDO GRAVACAO..."));
      Serial.println(F(">> Mova os potenciometros agora!"));
    }

    if (estado == 2) {
      pegarvalor1 = 0;
      lcd2.clear();
      lcd2.print("REPRODUZINDO...");
      lcd2.setCursor(0, 1);
      lcd2.print("pressione outra");
      lcd2.setCursor(0, 2);
      lcd2.print("vez para anular");
      lcd2.setCursor(0, 3);
      lcd2.print("essa gravacao");
      Serial.println(F(""));
      Serial.println(F(">> [PLAY] REPRODUZINDO MOVIMENTOS..."));
      Serial.println(F(">> Pressione outra vez para anular essa gravação"));
      Serial.print(F(">> Frames Totais: "));
      Serial.println(totalGravado);
    }

    if (estado == 0) {
      totalGravado = 0;
      pegarvalor1 = 0;
      lcd2.clear();
      lcd2.print("GHOST MODE ativo");
      lcd2.setCursor(0, 1);
      lcd2.print("nos servos");
      lcd2.setCursor(0, 2);
      lcd2.print("Pronto p/ gravar");
      Serial.println(F(""));
      Serial.println(F(">> [STOP] MODO FANTASMA EM ESPERA."));
      Serial.println(F(">> Pressione o botao para gravar."));
    }
  }
  ultimoEstado = estadoBotao;

  if (estado == 0) {
    execServoPot();
  }
  if (estado == 1) {
    if (millis() - tempoRecord > 30) {

      if (totalGravado < MAX_FRAMES) {
        int val1 = analogRead(pot1);
        int val2 = analogRead(pot2);

        byte potPosition1 = map(val1, 0, 1023, 0, 180);
        byte potPosition2 = map(val2, 0, 1023, 0, 180);

        if (millis() - tempoAnterior > delayTempoServos) {
          tempoAnterior = millis();

          meuServo.write(potPosition1);

          if (potPosition1 != ultimaPosServo1) {

            lcd.setCursor(0, 0);
            lcd.print("Motor 1:      ");
            lcd.setCursor(0, 1);
            lcd.print(potPosition1);
            lcd.print((char)223);
            lcd.print(" ");

            ultimaPosServo1 = potPosition1;
          }

          meuServo2.write(potPosition2);

          if (potPosition2 != ultimaPosServo2) {

            lcd.setCursor(0, 2);
            lcd.print("Motor 2:       ");
            lcd.setCursor(0, 3);
            lcd.print(potPosition2);
            lcd.print((char)223);
            lcd.print(" ");
            ultimaPosServo2 = potPosition2;
          }
        }
        ghostM1[totalGravado] = potPosition1;
        ghostM2[totalGravado] = potPosition2;
        totalGravado++;

      } else {
        lcd2.setCursor(0, 1);
        lcd2.print("MEMORIA CHEIA!  ");
        lcd2.setCursor(0, 2);
        lcd2.print("             ");
      }
      tempoRecord = millis();
    }
  }

  else if (estado == 2) {
    if (millis() - frameRate > 30) {

      if (pegarvalor1 < totalGravado) {

        byte alvo1 = ghostM1[pegarvalor1];
        byte alvo2 = ghostM2[pegarvalor1];

        meuServo.write(alvo1);
        meuServo2.write(alvo2);

        pegarvalor1++;
      } else {
        lcd2.setCursor(0, 0);
        lcd2.print("FIM DO PLAYBACK  ");
        lcd2.setCursor(0, 1);
        lcd2.print("                ");
        lcd2.setCursor(0, 2);
        lcd2.print("                ");
        lcd2.setCursor(0, 3);
        lcd2.print("                ");
      }
      frameRate = millis();
    }
  }
}

void execMedidor() {
  static unsigned long tempoApMedidas = 0;
  const byte medidasDelay = 80;

  static unsigned long tempoPassado = 0;
  const byte delayTemp = 150;
  static byte ultimoEstado;
  static byte ultimoEstado2;

  if (!mostrandoMensagem && millis() - tempoPassado > delayTemp) {
    tempoPassado = millis();

    unsigned int distancia_cm = sonar.ping_cm();
    unsigned int distancia_cm2 = sonar2.ping_cm();

    lcd.setCursor(0, 0);
    lcd.print("Distancia em CM:");
    lcd.setCursor(0, 1);
    lcd.print(distancia_cm);
    lcd.print("cm  ");

    lcd.setCursor(0, 2);
    lcd.print("Distancia em M:");
    lcd.setCursor(0, 3);
    lcd.print(distancia_cm / 100.0);
    lcd.print("m   ");


    lcd2.setCursor(0, 0);
    lcd2.print("Distancia em CM:");
    lcd2.setCursor(0, 1);
    lcd2.print(distancia_cm2);
    lcd2.print("cm  ");

    lcd2.setCursor(0, 2);
    lcd2.print("Distancia em M:");
    lcd2.setCursor(0, 3);
    lcd2.print(distancia_cm2 / 100.0);
    lcd2.print("m   ");
  }

  byte estadoBotao = digitalRead(botao3);
  if (estadoBotao == LOW && ultimoEstado == HIGH && (millis() - tempoApMedidas > medidasDelay)) {
    tempoApMedidas = millis();
    mostrandoMensagem = true;
    salvarMedidas();
  }
  ultimoEstado = estadoBotao;

  byte estadoBotao2 = digitalRead(botao2);
  if (estadoBotao2 == LOW && ultimoEstado2 == HIGH && (millis() - tempoApMedidas > medidasDelay)) {
    tempoApMedidas = millis();
    mostrandoMensagem = true;
    salvarMedidas2();
  }
  ultimoEstado2 = estadoBotao2;

  if (mostrandoMensagem && millis() - tempoMensagemSalvo > 1500) {
    mostrandoMensagem = false;
    lcd.clear();
    lcd2.clear();
  }
}

void salvarMedidas() {
  int medidaAtual = sonar.ping_cm();

  listaDeMedidas.add(medidaAtual);
  Serial.print(F("Medida Salva S1: "));
  Serial.print(medidaAtual);
  Serial.print(F("cm | Total na memoria S1: "));
  Serial.println(listaDeMedidas.size());

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(">> REGISTRADO <<");
  lcd.setCursor(0, 1);
  lcd.print("Valor: ");
  lcd.print(medidaAtual);
  lcd.print(" cm");

  mostrandoMensagem = true;
  tempoMensagemSalvo = millis();
}

void salvarMedidas2() {
  int medidaAtual = sonar2.ping_cm();

  listaDeMedidas2.add(medidaAtual);
  Serial.print(F("Medida Salva S2: "));
  Serial.print(medidaAtual);
  Serial.print(F("cm | Total na memoria S2: "));
  Serial.println(listaDeMedidas2.size());

  lcd2.clear();
  lcd2.setCursor(0, 0);
  lcd2.print(">> REGISTRADO <<");
  lcd2.setCursor(0, 1);
  lcd2.print("Valor: ");
  lcd2.print(medidaAtual);
  lcd2.print(" cm");

  mostrandoMensagem = true;
  tempoMensagemSalvo = millis();
}

void mostrarResultados() {

  Serial.println(F("\n+============================================+"));
  Serial.println(F("|         >>> RELATORIO DE MEDIDAS <<<       |"));
  Serial.println(F("|               >>> SENSOR 1 <<<             |"));
  Serial.println(F("+============================================+"));
  Serial.println(F("|  #  |  DISTANCIA (cm)   |  DISTANCIA (m)   |"));
  Serial.println(F("|-----|-------------------|------------------|"));

  if (listaDeMedidas.size() == 0) {
    Serial.println(F("|    [!] NENHUM DADO REGISTRADO NA MEMORIA   |"));

  } else {

    for (int i = 0; i < listaDeMedidas.size(); i++) {
      int valor = listaDeMedidas.get(i);
      float valorMetros = valor / 100.0;

      Serial.print(F("| "));
      if (i < 9) Serial.print("0");
      Serial.print(i + 1);
      Serial.print(F("  |      "));

      if (valor < 10) Serial.print(F("   "));
      else if (valor < 100) Serial.print(F("  "));
      else if (valor < 1000) Serial.print(F(" "));

      Serial.print(valor);
      Serial.print(F(" cm      |      "));
      Serial.print(valorMetros, 2);
      Serial.println(F(" m      |"));
    }
  }
  Serial.println(F("+============================================+"));
  Serial.print(F("| TOTAL DE LEITURAS: "));
  Serial.println(listaDeMedidas.size());
  Serial.println(F("+--------------------------------------------+\n"));
}

void mostrarResultados2() {

  Serial.println(F("\n+============================================+"));
  Serial.println(F("|         >>> RELATORIO DE MEDIDAS <<<       |"));
  Serial.println(F("|               >>> SENSOR 2 <<<             |"));
  Serial.println(F("+============================================+"));
  Serial.println(F("|  #  |  DISTANCIA (cm)   |  DISTANCIA (m)   |"));
  Serial.println(F("|-----|-------------------|------------------|"));

  if (listaDeMedidas2.size() == 0) {
    Serial.println(F("|    [!] NENHUM DADO REGISTRADO NA MEMORIA   |"));

  } else {

    for (int i = 0; i < listaDeMedidas2.size(); i++) {
      int valor = listaDeMedidas2.get(i);
      float valorMetros = valor / 100.0;

      Serial.print(F("| "));
      if (i < 9) Serial.print("0");
      Serial.print(i + 1);
      Serial.print(F("  |      "));

      if (valor < 10) Serial.print(F("   "));
      else if (valor < 100) Serial.print(F("  "));
      else if (valor < 1000) Serial.print(F(" "));

      Serial.print(valor);
      Serial.print(F(" cm      |      "));
      Serial.print(valorMetros, 2);
      Serial.println(F(" m      |"));
    }
  }
  Serial.println(F("+============================================+"));
  Serial.print(F("| TOTAL DE LEITURAS: "));
  Serial.println(listaDeMedidas2.size());
  Serial.println(F("+--------------------------------------------+\n"));
}

void apagarResultados() {
  listaDeMedidas.clear();
  Serial.println(F("\n\n##################################################"));
  Serial.println(F("#   SISTEMA DE GESTAO DE MEMORIA PARA MEDIDAS    #"));
  Serial.println(F("##################################################"));
  Serial.println(F("# > INICIANDO PROTOCOLO DE LIMPEZA...            #"));
  delay(300);
  Serial.print(F("# > ACESSANDO ENDERECOS DE MEMORIA...      "));
  delay(300);
  Serial.println(F("[OK]  #"));
  Serial.print(F("# > DELETANDO REGISTROS...                 "));
  listaDeMedidas.clear();
  listaDeMedidas2.clear();
  delay(300);
  Serial.println(F("[OK]  #"));
  Serial.println(F("#------------------------------------------------#"));
  Serial.println(F("#      >>> MEMORIA RESETADA COM SUCESSO <<<      #"));
  Serial.println(F("##################################################\n"));
}

void execVelocidade() {

  stepper.setAcceleration(1);
  stepper2.setAcceleration(1);
  stepper3.setAcceleration(1);
  stepperA.setAcceleration(1);

  int potValue1 = analogRead(pot1);
  int potValue2 = analogRead(pot2);
  int potValue3 = analogRead(pot3);
  int potValue4 = analogRead(pot4);

  speed = map(abs(potValue1 - 512), 0, 512, 0, 2300);
  if (potValue1 < 512) {
    stepper.setSpeed(-speed);
  } else {
    stepper.setSpeed(speed);
  }

  speed2 = map(abs(potValue2 - 512), 0, 512, 0, 2300);
  if (potValue2 < 512) {
    stepper2.setSpeed(-speed2);
  } else {
    stepper2.setSpeed(speed2);
  }

  speed3 = map(abs(potValue3 - 512), 0, 512, 0, 2300);
  if (potValue3 < 512) {
    stepper3.setSpeed(-speed3);
  } else {
    stepper3.setSpeed(speed3);
  }

  speed4 = map(abs(potValue4 - 512), 0, 512, 0, 2300);
  if (potValue4 < 512) {
    stepperA.setSpeed(-speed4);
  } else {
    stepperA.setSpeed(speed4);
  }
  stepper.runSpeed();
  stepper2.runSpeed();
  stepper3.runSpeed();
  stepperA.runSpeed();
}

void execModoServosUltrassonico() {
  static unsigned long tempoDelayUltrassonico = 0;
  const byte tempoCarregamento = 80;

  if (millis() - tempoDelayUltrassonico > tempoCarregamento) {
    tempoDelayUltrassonico = millis();

    unsigned int dist = sonar.ping_cm();

    if (dist == 0) dist = 30;

    byte angulo = map(dist, 0, 30, 0, 180);

    if (angulo < 0) {
      angulo = 0;
    }
    if (angulo > 180) {
      angulo = 180;
    }

    byte deadBand = 5;
    static byte anguloAnterior = 0;

    if (abs(angulo - anguloAnterior) > deadBand) {
      meuServo.write(angulo);
      meuServo2.write(180 - angulo);

      lcd.setCursor(0, 0);
      lcd.print("Dist: ");
      lcd.print(dist);
      lcd.print("cm   ");

      lcd.setCursor(0, 1);
      lcd.print("Angulo: ");
      lcd.print(angulo);
      lcd.print((char)223);
      lcd.print("  ");
    }
    anguloAnterior = angulo;
    angulo = deadBand;
  }
}

void execSemaforoF1() {

  digitalWrite(LED_PIN1, LOW);
  digitalWrite(LED_PIN2, LOW);
  digitalWrite(LED_PIN3, LOW);
  digitalWrite(LED_PIN4, LOW);
  digitalWrite(LED_PIN5, LOW);

  byte estadoBotao1 = digitalRead(botao1);
  if (state == IDLE && estadoBotao1 == LOW) {
    if (estadoBotao1 == LOW && !buttonPressed) {
      buttonPressed = true;
      lcd.clear();
      lcd.print("Preparando....");
      delay(1000);

      state = WAITING;

      digitalWrite(LED_PIN1, HIGH);
      digitalWrite(LED_PIN2, HIGH);
      digitalWrite(LED_PIN3, HIGH);
      digitalWrite(LED_PIN4, HIGH);
      digitalWrite(LED_PIN5, HIGH);

      unsigned int lightTime = random(200, 3000);
      unsigned long waitStart = millis();

      while (millis() - waitStart < lightTime) {
        if (digitalRead(botao1) == HIGH) {
          lcd.clear();
          lcd.print("Queimou largada!!!");
          delay(2000);
          lcd.clear();
          lcd.print("Tente novamente");
          delay(1500);
          lcd.clear();
          delay(100);
          largadasInvalidas++;
          state = IDLE;

          return;
        }
      }

      digitalWrite(LED_PIN1, LOW);
      digitalWrite(LED_PIN2, LOW);
      digitalWrite(LED_PIN3, LOW);
      digitalWrite(LED_PIN4, LOW);
      digitalWrite(LED_PIN5, LOW);


      startTime = millis();
      state = TIMING;
      lcd.clear();
      lcd.print("GO! Pressione!");
    }
  }

  if (state == TIMING) {
    if (estadoBotao1 == HIGH && !buttonPressed) {
      buttonPressed = true;
      reactionTime = millis() - startTime;
      float resultTime = reactionTime / 1000.0;
      lcd.clear();
      lcd.print("Tempo: ");
      lcd.print(reactionTime / 1000.0, 3);
      lcd.print("s");
      listaTemposF1.add(resultTime);

      float melhorTempo = 100.0;
      for (int k = 0; k < listaTemposF1.size(); k++) {
        if (listaTemposF1.get(k) < melhorTempo) {
          melhorTempo = listaTemposF1.get(k);

          lcd2.setCursor(0, 2);
          lcd2.print("Melhor tempo: ");
          lcd2.setCursor(0, 3);
          lcd2.print(melhorTempo, 3);
          lcd2.print("s");
        }
      }
      state = IDLE;

    } else {
      unsigned int elapsedTime = millis() - startTime;
      lcd.setCursor(0, 1);
      lcd.print("Crono: ");
      lcd.print(elapsedTime / 1000.0, 3);
      lcd.print("s");
    }
  }
  if (estadoBotao1 == HIGH) {
    buttonPressed = false;
  }
}

void mostrarRelatorioF1() {
  Serial.println(F("\n+================================================+"));
  Serial.println(F("|       >>> TELEMETRIA DE CORRIDA F1 <<<         |"));
  Serial.println(F("+================================================+"));
  Serial.println(F("|  VOLTA  |   TEMPO (s)    |      STATUS         |"));
  Serial.println(F("|---------|----------------|---------------------|"));

  if (listaTemposF1.size() == 0) {
    Serial.println(F("|      [!] NENHUMA VOLTA REGISTRADA              |"));
  } else {

    float melhorTempo = 100.0;
    for (int k = 0; k < listaTemposF1.size(); k++) {
      if (listaTemposF1.get(k) < melhorTempo) {
        melhorTempo = listaTemposF1.get(k);
      }
    }

    for (int i = 0; i < listaTemposF1.size(); i++) {
      float tempo = listaTemposF1.get(i);

      Serial.print(F("|   "));
      if (i < 9) Serial.print("0");
      Serial.print(i + 1);
      Serial.print(F("    |    "));

      Serial.print(tempo, 3);
      Serial.print(F("s      |   "));

      if (tempo == melhorTempo) {
        Serial.println(F("POLE POSITION! 🏆 |"));
      } else if (tempo < 0.200) {
        Serial.println(F("REFLEXO ALIEN! 👽 |"));
      } else if (tempo < 0.300) {
        Serial.println(F("REFLEXO NATURAL 🧑‍🦱|"));
      } else if (tempo > 0.300) {
        Serial.println(F("LENTO... 🐢       |"));
      } else {
        Serial.println(F("                  |"));
      }
    }
  }
  Serial.println(F("+================================================+"));
  Serial.print(F("| LARGADAS VALIDAS: "));
  Serial.println(listaTemposF1.size());
  Serial.println(F("+------------------------------------------------+"));
  Serial.print(F("| LARGADAS INVALIDAS: "));
  Serial.println(largadasInvalidas);
  Serial.println(F("+------------------------------------------------+"));
  Serial.print(F("| TOTAL DE LARGADAS: "));
  Serial.print(largadasInvalidas + listaTemposF1.size());
  Serial.println(F("\n+================================================+"));
}

void limparDadosF1() {
  listaTemposF1.clear();

  Serial.println(F("\n\n"));
  Serial.println(F("##################################################"));
  Serial.println(F("#      RACE CONTROL - SISTEMA DE TELEMETRIA      #"));
  Serial.println(F("##################################################"));
  Serial.println(F("# > SOLICITACAO DE RESET RECEBIDA...             #"));
  delay(200);
  Serial.print(F("# > EXCLUINDO VOLTAS ANTIGAS...            "));
  delay(300);
  Serial.println(F("[OK]  #"));
  Serial.print(F("# > ZERANDO CONTADOR DE LARGADAS...        "));
  delay(300);
  Serial.println(F("[OK]  #"));
  Serial.println(F("#------------------------------------------------#"));
  Serial.println(F("#     >>> NOVA SESSAO DE TREINO INICIADA <<<     #"));
  Serial.println(F("##################################################\n"));

  largadasInvalidas = 0;

  lcd.clear();
  lcd.print("Historico F1");
  lcd.setCursor(0, 1);
  lcd.print("Apagado!");
  lcd2.setCursor(0, 3);
  lcd2.print("       ");
  delay(1000);
}

void execIAF1(String input) {

  static float configCombustivel = 0.0;
  static float configReserva = 0.0;
  static float configDesgaste = 0.0;
  static float configLaps = 0.0;

  static bool combustivelConfigurado = false;
  static bool pneuConfigurado = false;

  int indiceVirgula = input.indexOf(',');
  int doisPontos = input.indexOf(':');

  if (input.length() > 0) {
    if (indiceVirgula == -1 || doisPontos == -1) {
      Serial.println(F("[ERRO] Formato invalido! Use: NOME, VALOR1 : VALOR2"));
      return;
    }

    String ID = input.substring(0, indiceVirgula);
    String n1 = input.substring(indiceVirgula + 1, doisPontos);
    String n2 = input.substring(doisPontos + 1);

    ID.trim();
    ID.toUpperCase();
    n1.trim();
    n2.trim();

    float valor1 = n1.toFloat();
    float valor2 = n2.toFloat();


    if (ID.equals("COMBUSTIVEL")) {
      configCombustivel = valor1;
      configReserva = valor2;
      combustivelConfigurado = true;
      Serial.print(F("[OK] Combustivel definido: "));
      Serial.println(configCombustivel);
      Serial.print(F("[OK] Reserva definida: "));
      Serial.println(configReserva);
      Serial.println(F("Agora configure o PNEU..."));

    } else if (ID.equals("PNEU") || ID.equals("DESGASTE")) {
      configDesgaste = valor1;
      configLaps = valor2;
      pneuConfigurado = true;
      Serial.print(F("[OK] Pneu definido: "));
      Serial.println(configDesgaste);
    }
  }


  if (combustivelConfigurado && pneuConfigurado) {
    Serial.println(F("\n+--------------------------------+"));
    Serial.println(F("| INICIANDO TREINAMENTO DA IA... |"));
    Serial.println(F("+--------------------------------+"));

    CerebroIA pilotoCombustivel("COMBUSTIVEL", configCombustivel, configReserva);
    CerebroIA pilotoPneu("PNEU", configDesgaste, configLaps);

    int tentativas = 0;
    const int maxTentativas = 7000;

    bool iaCombustivelPronta = false;
    bool iaPneuPronta = false;

    while ((!iaCombustivelPronta || !iaPneuPronta) && tentativas < maxTentativas) {

      if (!iaCombustivelPronta) {
        pilotoCombustivel.treinar();
        if (abs(pilotoCombustivel.alvoIA1 - configCombustivel) < 0.00001) iaCombustivelPronta = true;
      }

      if (!iaPneuPronta) {
        pilotoPneu.treinar();
        if (abs(pilotoPneu.alvoIA1 - configDesgaste) < 0.0001) iaPneuPronta = true;
      }

      tentativas++;
    }

    if (tentativas >= maxTentativas) {
      Serial.println(F("\n[AVISO] A IA parou por limite de tentativas (Segurança)."));

    } else {

      Serial.println(F("\n"));
      Serial.println(F("+==================================================+"));
      Serial.println(F("|      >>> RELATORIO FINAL DE TREINO (IA) <<<      |"));
      Serial.println(F("+==================================================+"));
      Serial.println(F("| O modelo convergiu com sucesso! Analise abaixo:  |"));
      Serial.println(F("+--------------------------------------------------+"));
      Serial.println(F("| PARAMETRO       | ALVO (REAL)  | IA (APRENDIDO)  |"));
      Serial.println(F("|-----------------|--------------|-----------------|"));

      Serial.print(F("| Consumo (L/V)   | "));
      Serial.print(configCombustivel);
      Serial.print(F("         | "));
      Serial.print(pilotoCombustivel.alvoIA1, 3);  // 4 casas pra ver a precisão
      Serial.println(F("           |"));

      Serial.print(F("| Reserva (L)     | "));
      Serial.print(configReserva);
      Serial.print(F("         | "));
      Serial.print(pilotoCombustivel.alvoIA2, 3);
      Serial.println(F("           |"));

      Serial.println(F("|-----------------|--------------|-----------------|"));

      Serial.print(F("| Desgaste Pneu   | "));
      Serial.print(configDesgaste);
      Serial.print(F("         | "));
      Serial.print(pilotoPneu.alvoIA1, 3);
      Serial.println(F("           |"));

      Serial.println(F("+==================================================+"));
      Serial.print(F("| TENTATIVAS USADAS: "));
      Serial.print(tentativas);
      Serial.print(F(" / "));
      Serial.print(maxTentativas);
      Serial.println(F("                   |"));
      Serial.println(F("+==================================================+"));
      Serial.println(F("| DICA: Valores prontos para uso em corrida real.  |"));
      Serial.println(F("+==================================================+"));
    }

    combustivelConfigurado = false;
    pneuConfigurado = false;
  }
}

void execMotorVel() {

  int estadoBotao = (digitalRead(botao3));
  static int ultimoEstado;
  static unsigned int tempoTroca = 0;
  const byte debounceDelay = 80;
  if (estadoBotao == LOW && ultimoEstado == HIGH && (millis() - tempoTroca > debounceDelay)) {
    tempoTroca = millis();

    digitalWrite(BASE1, HIGH);
    digitalWrite(BASE2, HIGH);
    digitalWrite(BASE3, LOW);
    digitalWrite(BASE4, LOW);

    lcd.setCursor(0, 0);
    lcd.print("Motor: GIRANDO 'R'");
  }
  ultimoEstado = estadoBotao;

  int estadoBotao2 = (digitalRead(botao2));
  static int ultimoEstado2;
  static unsigned int tempoTroca2 = 0;
  if (estadoBotao2 == LOW && ultimoEstado2 == HIGH && (millis() - tempoTroca2 > debounceDelay)) {
    tempoTroca2 = millis();

    digitalWrite(BASE1, LOW);
    digitalWrite(BASE2, LOW);
    digitalWrite(BASE3, HIGH);
    digitalWrite(BASE4, HIGH);

    lcd.setCursor(0, 0);
    lcd.print("Motor: GIRANDO 'L'");
  }
  ultimoEstado2 = estadoBotao2;

  int estadoBotao3 = (digitalRead(botao1));
  static int ultimoEstado3;
  static unsigned int tempoTroca3 = 0;
  if (estadoBotao3 == LOW && ultimoEstado3 == HIGH && (millis() - tempoTroca3 > debounceDelay)) {
    tempoTroca3 = millis();

    digitalWrite(BASE1, LOW);
    digitalWrite(BASE2, LOW);
    digitalWrite(BASE3, LOW);
    digitalWrite(BASE4, LOW);

    lcd.setCursor(0, 0);
    lcd.print("Motor: PARADO      ");
  }
  ultimoEstado3 = estadoBotao3;

  if (estadoBotao == LOW && estadoBotao2 == LOW) {

    digitalWrite(BASE1, LOW);
    digitalWrite(BASE2, LOW);
    digitalWrite(BASE3, LOW);
    digitalWrite(BASE4, LOW);

    lcd.setCursor(0, 0);
    lcd.print("Motor: PARADO      ");
  }
}

void desligarMotorVel() {
  digitalWrite(BASE1, LOW);
  digitalWrite(BASE2, LOW);
  digitalWrite(BASE3, LOW);
  digitalWrite(BASE4, LOW);
}

void resetarInercia() {
  stepper.setSpeed(0);
  stepper2.setSpeed(0);
  stepper3.setSpeed(0);
  stepperA.setSpeed(0);

  stepper.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  stepperA.setCurrentPosition(0);
}

void execInativo() {

  lcd.setCursor(0, 0);
  lcd.print("Sistemas DESATIVADOS");
  lcd2.setCursor(0, 0);
  lcd2.print("Motores ");
  lcd2.setCursor(0, 1);
  lcd2.print("DESATIVADOS ");

  meuServo.detach();
  meuServo2.detach();

  motoresDesativados = true;
}

void execAtivo() {

  lcd.setCursor(0, 0);
  lcd.print("Sistemas ATIVADOS");
  lcd2.setCursor(0, 0);
  lcd2.print("Motores ");
  lcd2.setCursor(0, 1);
  lcd2.print("ATIVADOS ");

  meuServo.attach(34);
  meuServo2.attach(36);

  motoresDesativados = false;
}

void mostrarManualCompleto() {
  Serial.println(F("\n\n\n\n"));
  Serial.println(F("################################################################################"));
  Serial.println(F("#        MANUAL DO PROPRIETARIO (MASTER) - SISTEMA DE CONTROLE INTEGRADO       #"));
  Serial.println(F("#                                                                              #"));
  Serial.println(F("################################################################################"));
  Serial.println(F(""));

  Serial.println(F("=== [1] SISTEMA GERAL E SEGURANCA =============================================="));
  Serial.println(F("Comandos para controle de energia e documentacao global."));
  Serial.println(F("--------------------------------------------------------------------------------"));
  Serial.println(F(">>> LIGAR MOTORES (Energizar / Travar eixos)"));
  Serial.println(F("    Comandos Aceitos: 'ON', 'MOTOR ON', 'LIGAR MOTORES', 'ATIVAR MOTORES'"));
  Serial.println(F(""));
  Serial.println(F(">>> DESLIGAR MOTORES (Corte de energia / Eixos livres)"));
  Serial.println(F("    Comandos Aceitos: 'OFF', 'MOTOR OFF', 'DESLIGAR MOTORES', 'DESATIVAR MOTORES'"));
  Serial.println(F(""));
  Serial.println(F(">>> AJUDA E DOCUMENTACAO"));
  Serial.println(F("    Menu Rapido:      'H', 'AJUDA'"));
  Serial.println(F("    Manual Completo:  'MANUAL', 'MEU MANUAL', 'MANUAL DA MAQUINA'"));
  Serial.println(F(""));

  Serial.println(F("=== [2] SERVO MOTORES (BRACOS ROBOTICOS) ======================================="));
  Serial.println(F("Controle dos Servos conectados aos pinos 34 e 36."));
  Serial.println(F("--------------------------------------------------------------------------------"));
  Serial.println(F(">>> MODO 1: CONTROLE VIA SERIAL (Digite o angulo)"));
  Serial.println(F("    Ativar Modo: 'CMSS', 'SERVO SERIAL', 'SERIAL SERVO'"));
  Serial.println(F("    Como usar:   Digite a letra do motor (A ou B) + espaco + angulo (0 a 180)."));
  Serial.println(F("    Exemplos:    'A 90', 'B 45', 'A 180'"));
  Serial.println(F(""));
  Serial.println(F(">>> MODO 2: CONTROLE VIA POTENCIOMETRO (Manual)"));
  Serial.println(F("    Ativar Modo: 'CSP', 'SERVO POT', 'CONTROLE DE SERVO POR POTENCIOMETRO'"));
  Serial.println(F("    Como usar:   Gire o POT 1 para o Servo A e o POT 2 para o Servo B."));
  Serial.println(F(""));
  Serial.println(F(">>> MODO 3: CONTROLE VIA SENSOR (Theremin / Gestos)"));
  Serial.println(F("    Ativar Modo: 'CSPS', 'MODO SERVO POR DISTANCIA', 'CONTROLE DO SERVO POR DISTANCIA'"));
  Serial.println(F("    Como usar:   Aproxime a mao do sensor ultrassonico."));
  Serial.println(F("                 Perto (0-5cm) = 0 graus | Longe (30cm) = 180 graus."));
  Serial.println(F(""));

  Serial.println(F("=== [3] MOTORES DE PASSO (STEPPERS) ============================================"));
  Serial.println(F("Controle de 4 motores de passo independentes."));
  Serial.println(F("--------------------------------------------------------------------------------"));
  Serial.println(F(">>> MODO 1: CONTROLE DE POSICAO ABSOLUTA (Serial)"));
  Serial.println(F("    Ativar Modo: 'CMPS', 'MOTOR DE PASSO POR SERIAL', 'CONTROLE DE MOTORES DE PASSO POR SERIAL'"));
  Serial.println(F("    Como usar:   Digite 'M' + Numero do Motor (1-4) + espaco + Passos."));
  Serial.println(F("    Exemplos:    'M1 200' (Avanca), 'M2 -500' (Recua), 'M3 0' (Volta pro zero)."));
  Serial.println(F(""));
  Serial.println(F(">>> MODO 2: CONTROLE DE POSICAO POR POTENCIOMETRO (Follow Focus)"));
  Serial.println(F("    Ativar Modo: 'MPP', 'MOTOR DE PASSO POR POT'"));
  Serial.println(F("    Como usar:   O motor copia o movimento do seu respectivo potenciometro."));
  Serial.println(F("                 Pot no min = Passo 0 | Pot no max = Passo 200."));
  Serial.println(F(""));
  Serial.println(F(">>> MODO 3: CONTROLE DE VELOCIDADE CONTINUA (RPM)"));
  Serial.println(F("    Ativar Modo: 'MPV', 'MODO VELOCIDADE', 'ATIVAR MODO CONTINUO',"));
  Serial.println(F("                 'ATIVAR O MODO VELOCIDADE DOS MOTORES DE PASSO'"));
  Serial.println(F("    Como usar:   O Potenciometro vira um acelerador."));
  Serial.println(F("                 Centro (meio dia) = Parado."));
  Serial.println(F("                 Esquerda = Gira Anti-Horario | Direita = Gira Horario."));
  Serial.println(F(""));

  Serial.println(F(">>> MODO 4: CONTROLE CNC / TRAÇADO (Teach & Repeat Serial)"));
  Serial.println(F("    Ativar Modo: 'TRACADO', 'TRAÇADO', 'MOTORES NO MODO TRAÇADO'"));
  Serial.println(F("    Como usar:   Sistema de 3 Estados controlado pelo BOTAO PRINCIPAL."));
  Serial.println(F("    [1] REC :    Digite coordenadas no Serial (Ex: 'M1 2000', 'M2 500')."));
  Serial.println(F("    [2] PLAY:    Executa a sequencia gravada (CUIDADO: MOVIMENTO AUTOMATICO)."));
  Serial.println(F("    [0] STOP:    Pausa o sistema."));
  Serial.println(F("    *NOTA:       Use o botao de EMERGENCIA (Reset) para cancelar movimentos!"));
  Serial.println(F(""));
  Serial.println(F("    > SISTEMA DE ARQUIVOS (MEMORIA EEPROM):"));
  Serial.println(F("      1. LISTAR ARQUIVOS: Digite 'LISTAR' para ver os slots ocupados."));
  Serial.println(F("      2. SALVAR TRAJETO:  Digite 'SALVAR X' (Onde X e uma pasta de 1 a 12)."));
  Serial.println(F("                          Ex: 'SALVAR 1' guarda o trajeto na pasta 1."));
  Serial.println(F("      3. CARREGAR TRAJETO: Digite 'CARREGAR X' para recuperar da memoria."));
  Serial.println(F("                           Ex: 'CARREGAR 1' restaura os dados da pasta 1."));
  Serial.println(F(""));

  Serial.println(F("=== [4] GHOST MODE (GRAVADOR DE MOVIMENTOS) ===================================="));
  Serial.println(F("Sistema de aprendizado e repeticao automatica (Automacao)."));
  Serial.println(F("--------------------------------------------------------------------------------"));
  Serial.println(F(">>> CENTRAL DE COMANDO (HUB)"));
  Serial.println(F("    Ativar: 'GHOST', 'GHOST MODE'    "));
  Serial.println(F("    Funcao: Abre o menu para escolher entre gravar Servos ou Motores de Passo."));
  Serial.println(F(""));
  Serial.println(F(">>> GRAVADOR DE MOTORES DE PASSO"));
  Serial.println(F("    Ativar Direto: 'PASSO', 'MOTORES DE PASSO'"));
  Serial.println(F("    ATENCAO: Ao entrar neste modo, a posicao atual e definida como ZERO."));
  Serial.println(F(""));
  Serial.println(F(">>> GRAVADOR DE SERVO MOTORES"));
  Serial.println(F("    Ativar Direto: 'SERVO', 'SERVO MOTORES'"));
  Serial.println(F(""));
  Serial.println(F(">>> INSTRUCOES DE USO (BOTAO UNICO):"));
  Serial.println(F("    [ESTADO 0] Standby: Apenas move os motores manualmente (Live)."));
  Serial.println(F("       |-> Aperte o Botao..."));
  Serial.println(F("    [ESTADO 1] REC (Gravando): O sistema salva seus movimentos por ~25seg."));
  Serial.println(F("       |-> Aperte o Botao..."));
  Serial.println(F("    [ESTADO 2] PLAY (Reproduzindo): O sistema repete o movimento em loop."));
  Serial.println(F("       |-> Aperte o Botao para voltar ao Zero."));
  Serial.println(F(""));

  Serial.println(F("=== [5] UTILITARIOS E FERRAMENTAS =============================================="));
  Serial.println(F("--------------------------------------------------------------------------------"));
  Serial.println(F(">>> REGUA DIGITAL (MEDIDOR ULTRASSONICO)"));
  Serial.println(F("    Ativar Modo: 'MUS', 'MODO MEDIDOR', 'MODO ULTRASSONICO'"));
  Serial.println(F("    Acao Principal: Pressione o BOTAO FISICO para gravar uma medida na RAM."));
  Serial.println(F("    > Ver Historico:    'VER MEDIDAS', 'MEDIDAS', 'MOSTRAR RELATORIO DE MEDIAS'"));
  Serial.println(F("    > Apagar Historico: 'LIMPAR MEDIDAS', 'DEL MED', 'DELETE MEDIDAS'"));
  Serial.println(F(""));

  Serial.println(F("=== [6] JOGOS E SIMULADORES ===================================================="));
  Serial.println(F("--------------------------------------------------------------------------------"));
  Serial.println(F(">>> SIMULADOR DE LARGADA F1 (TREINO DE REFLEXO)"));
  Serial.println(F("    Ativar Modo: 'SDF1', 'SEMAFORO DE F1', 'ATIVAR SEMAFORO DE F1'"));
  Serial.println(F("    Como Jogar:  Aguarde as 5 luzes vermelhas. Quando APAGAREM, aperte o botao!"));
  Serial.println(F("    > Ver Ranking:      'VER TEMPOS F1', 'TEMPOS', 'TELEMETRIA DE TEMPO'"));
  Serial.println(F("    > Resetar Sessao:   'LIMPAR TEMPOS', 'DEL F1', 'DELETE TEMPOS', 'EXCLUIR TEMPOS'"));
  Serial.println(F(""));
  Serial.println(F(">>> SIMULADOR DE CAMBIO (H-SHIFTER)"));
  Serial.println(F("    Ativar Modo: 'MARCHA', 'MODO MARCHA', 'H-SHIFTER'"));
  Serial.println(F("    Como Jogar:  Simula a fisica de um motor de carro e trocas de marcha."));
  Serial.println(F("    > Controles: BOTAO 2 (Sobe Marcha) | BOTAO 3 (Desce Marcha)"));
  Serial.println(F("    > Acelerador: POTENCIOMETRO 1"));
  Serial.println(F("    > Dica Pro:  Acelere tudo no NEUTRO para cortar giro (Vibracao)."));
  Serial.println(F(""));

  Serial.println(F("=== [7] MOTORES AUXILIARES (PONTE H) ==========================================="));
  Serial.println(F("--------------------------------------------------------------------------------"));
  Serial.println(F(">>> MODO MOTOR DE VELOCIDADE (MVC)"));
  Serial.println(F("    Ativar Modo: 'MVC', 'MODO MOTOR DE VELOCIDADE', 'MOTOR CONSTANTE'"));
  Serial.println(F("    Como usar:   Controla o sentido de giro de um motor DC via Ponte H."));
  Serial.println(F("    > BOTAO 1:   PARAR MOTOR (Comando de parada imediata)"));
  Serial.println(F("    > BOTAO 2:   GIRAR PARA ESQUERDA (Sentido Anti-Horario)"));
  Serial.println(F("    > BOTAO 3:   GIRAR PARA DIREITA (Sentido Horario)"));
  Serial.println(F(""));

  Serial.println(F("################################################################################"));
  Serial.println(F("#                FIM DO MANUAL - SISTEMA PRONTO PARA COMANDO                   #"));
  Serial.println(F("################################################################################"));
  Serial.println(F("\n\n\n\n"));
}

void mostrarAjuda() {
  Serial.println(F("\n\n\n\n"));
  Serial.println(F("================================================================"));
  Serial.println(F("|                SISTEMA DE CONTROLE INTEGRADO                 |"));
  Serial.println(F("================================================================"));
  Serial.println(F("| CODIGO | FUNCAO                                              |"));
  Serial.println(F("|--------|-----------------------------------------------------|"));
  Serial.println(F("|   >> SERVO MOTORES                                           |"));
  Serial.println(F("| CMSS   | Controle Serial (Digite: 'A 90' ou 'B 180')         |"));
  Serial.println(F("| CSP    | Controle Manual via Potenciometros (Pot 1 e 2)      |"));
  Serial.println(F("| CSPS   | Modo 'Theremin' (Controle por Distancia da Mao)     |"));
  Serial.println(F("|--------|-----------------------------------------------------|"));
  Serial.println(F("|   >> MOTORES DE PASSO                                        |"));
  Serial.println(F("| CMPS   | Controle Serial (Digite: 'M1 2000' ou 'M2 -500')    |"));
  Serial.println(F("| MPP    | Controle de Posicao via Potenciometros (Pot 1 a 4)  |"));
  Serial.println(F("| MPV    | Controle de Velocidade Continua (Pot define RPM)    |"));
  Serial.println(F("|--------|-----------------------------------------------------|"));
  Serial.println(F("|   >> MODO TRAÇADO (CNC & EEPROM)                             |"));
  Serial.println(F("| TRACADO| Ativa o Modo CNC. (Use 'REC' / 'PLAY' no Botao)     |"));
  Serial.println(F("| LISTAR | [No Modo] Lista os arquivos salvos na memoria       |"));
  Serial.println(F("| SALVAR | [No Modo] Digite 'SALVAR X' (Onde X = Pasta 1-12)   |"));
  Serial.println(F("| CARREGAR| [No Modo] Digite 'CARREGAR X' (Recupera trajeto)   |"));
  Serial.println(F("|--------|-----------------------------------------------------|"));
  Serial.println(F("|   >> GHOST MODE (GRAVADOR DE MOVIMENTOS)                     |"));
  Serial.println(F("| GHOST  | Abre o Menu Central de Gravacao (Hub)               |"));
  Serial.println(F("| PASSO  | Gravar/Reproduzir Motores de Passo (Direto)         |"));
  Serial.println(F("| SERVO  | Gravar/Reproduzir Servo Motores (Direto)            |"));
  Serial.println(F("|--------|-----------------------------------------------------|"));
  Serial.println(F("|   >> MOTORES AUXILIARES (PONTE H)                            |"));
  Serial.println(F("| MVC    | Modo Velocidade Constante (Botoes p/ Direcao)       |"));
  Serial.println(F("|--------|-----------------------------------------------------|"));
  Serial.println(F("|   >> SIMULADORES E JOGOS (F1 & CARROS)                       |"));
  Serial.println(F("| SDF1   | Iniciar Jogo de Largada (Luzes + Reflexo)           |"));
  Serial.println(F("| TEMPOS | Exibe Ranking de Tempos e Pole Position             |"));
  Serial.println(F("| MARCHA | Ativa Simulador de Cambio H-Shifter (6 Marchas)     |"));
  Serial.println(F("|--------|-----------------------------------------------------|"));
  Serial.println(F("|   >> MEDIDOR ULTRASSONICO (MODO MUS)                         |"));
  Serial.println(F("| MUS    | Ativa Modo Medidor (Use o BOTAO p/ Salvar)          |"));
  Serial.println(F("| MEDIDAS| Exibe Relatorio de Medidas (Distancia)              |"));
  Serial.println(F("| DEL MED| Apaga registros de Distancia                        |"));
  Serial.println(F("|--------|-----------------------------------------------------|"));
  Serial.println(F("|   >> SISTEMA GERAL                                           |"));
  Serial.println(F("| OFF    | MOTOR OFF - Corta energia (Eixo livre)              |"));
  Serial.println(F("| ON     | MOTOR ON  - Ativa energia (Eixo travado)            |"));
  Serial.println(F("| H      | Exibe este menu de ajuda novamente                  |"));
  Serial.println(F("| MANUAL | Abre o Manual Completo do Proprietario              |"));
  Serial.println(F("================================================================"));
  Serial.println(F("\n\n\n\n"));
}