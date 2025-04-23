#include <Arduino.h>
#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

// Піни для драйвера моторів
#define LEFT_MOTOR_PIN1 2
#define LEFT_MOTOR_PIN2 3
#define RIGHT_MOTOR_PIN1 4
#define RIGHT_MOTOR_PIN2 5
#define LEFT_MOTOR_PWM 9
#define RIGHT_MOTOR_PWM 10
// Кнопка старту
#define START_BUTTON 7
// Змінні для роботи
bool isRunning = false;
int lastError = 0;
int integral = 0;
const int BASE_SPEED = 150; // Базова швидкість (0-255)

// Константи для PID-регулятора
float Kp = 0.5;  // Пропорційний коефіцієнт
float Ki = 0.01; // Інтегральний коефіцієнт
float Kd = 0.2;  // Диференціальний коефіцієнт

void calibrateSensors()
{
  Serial.println("Калібрування сенсорів...");
  Serial.println("Рухайте болідом вздовж лінії в обидві сторони");
  // Блимання світлодіодом для індикації калібрування
  pinMode(LED_BUILTIN, OUTPUT);
  // Калібрування протягом 10 секунд
  for (uint16_t i = 0; i < 400; i++)
  {
    // Блимання світлодіодом
    if (i % 40 == 0)
    {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
    qtr.calibrate();
    delay(25);
  }
  digitalWrite(LED_BUILTIN, LOW);
  // Вивід мінімальних значень калібрування
  Serial.println("Мінімальні значення:");
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(" ");
  }
  Serial.println();
  // Вивід максимальних значень калібрування
  Serial.println("Максимальні значення:");
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println("Калібрування завершено!");
  Serial.println("Натисніть кнопку для запуску боліда...");
  // Чекаємо відпускання кнопки
  while (digitalRead(START_BUTTON) == LOW)
  {
    delay(10);
  }
  // Чекаємо повторного натискання кнопки для старту
  while (digitalRead(START_BUTTON) == HIGH)
  {
    delay(10);
  }
  // Невелика затримка перед стартом
  delay(1000);
  isRunning = true;
  Serial.println("Старт!");
}

void setup()
{
  // Конфігурація сенсора QTR-8A
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4}, SensorCount);
  // Налаштування пінів моторів як виходів
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  // Налаштування кнопки старту з внутрішнім підтягуючим резистором
  pinMode(START_BUTTON, INPUT_PULLUP);
  // Початок серійного зв'язку для налагодження
  Serial.begin(9600);
  Serial.println("Натисніть кнопку для калібрування сенсорів...");
  // Чекаємо натискання кнопки для калібрування
  while (digitalRead(START_BUTTON) == HIGH)
  {
    delay(10);
  }
  // Калібрування сенсорів
  calibrateSensors();
}

void driveMotors(int leftSpeed, int rightSpeed)
{
  // Керування лівим мотором
  if (leftSpeed >= 0)
  {
    digitalWrite(LEFT_MOTOR_PIN1, HIGH);
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
  }
  else
  {
    digitalWrite(LEFT_MOTOR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_PIN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  // Керування правим мотором
  if (rightSpeed >= 0)
  {
    digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
  }
  else
  {
    digitalWrite(RIGHT_MOTOR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
    rightSpeed = -rightSpeed;
  }
  // Встановлення швидкості через ШІМ
  analogWrite(LEFT_MOTOR_PWM, leftSpeed);
  analogWrite(RIGHT_MOTOR_PWM, rightSpeed);
}

void loop()
{
  // Керування стартом і зупинкою
  if (digitalRead(START_BUTTON) == LOW && !isRunning)
  {
    delay(50); // Для усунення дребезгу контактів
    if (digitalRead(START_BUTTON) == LOW)
    {
      isRunning = true;
      Serial.println("Старт!");
      delay(1000); // Затримка перед початком руху
    }
  }
  else if (digitalRead(START_BUTTON) == LOW && isRunning)
  {
    delay(50);
    if (digitalRead(START_BUTTON) == LOW)
    {
      isRunning = false;
      Serial.println("Зупинка!");
      driveMotors(0, 0); // Зупинка моторів
      delay(1000);
    }
  }
  // Якщо болід запущено, виконуємо алгоритм руху
  if (isRunning)
  {
    // Зчитування позиції (0-4000)
    uint16_t position = qtr.readLineBlack(sensorValues);
    // Обчислення помилки
    int error = position - 2000; // 2000 - це позиція центру
    // Вивід даних для налагодження
    Serial.print("Позиція: ");
    Serial.print(position);
    Serial.print(" Помилка: ");
    Serial.println(error);
    // Обчислення PID-значень
    // Пропорційна складова
    int P = error;
    // Інтегральна складова
    integral = integral + error;
    // Обмеження інтегралу для запобігання перенасичення
    if (integral > 1000)
      integral = 1000;
    if (integral < -1000)
      integral = -1000;
    int I = integral;
    // Диференціальна складова
    int D = error - lastError;
    lastError = error;
    // Обчислення PID-значення
    int pidValue = Kp * P + Ki * I + Kd * D;
    // Застосування PID до швидкості моторів
    int leftMotorSpeed = BASE_SPEED - pidValue;
    int rightMotorSpeed = BASE_SPEED + pidValue;
    // Обмеження швидкості
    if (leftMotorSpeed > 255)
      leftMotorSpeed = 255;
    if (leftMotorSpeed < 0)
      leftMotorSpeed = 0;
    if (rightMotorSpeed > 255)
      rightMotorSpeed = 255;
    if (rightMotorSpeed < 0)
      rightMotorSpeed = 0;
    // Керування моторами
    driveMotors(leftMotorSpeed, rightMotorSpeed);
  }
  else
  {
    // Зупинка моторів
    driveMotors(0, 0);
  }
  delay(10); // Коротка затримка для стабільності
}