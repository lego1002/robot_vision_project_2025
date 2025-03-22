/* 
  TB6612FNG 馬達測試範例
  測試方式：依序執行正轉、停止、反轉、停止
  前後馬達皆使用 50% PWM 輸出，以降低電流衝擊
*/

// 前兩顆馬達接腳設定
#define FRONT_PWM_A 3
#define FRONT_PWM_B 5
#define FRONT_AIN1 4
#define FRONT_AIN2 2
#define FRONT_BIN1 7
#define FRONT_BIN2 8

// 後兩顆馬達接腳設定
#define REAR_PWM_A 6
#define REAR_PWM_B 9
#define REAR_AIN1 A6
#define REAR_AIN2 A7
#define REAR_BIN1 A4
#define REAR_BIN2 A5

// 設定 PWM 佔空比 (0 ~ 255)，此處使用 128 (大約 50%)
const int pwmSpeed = 128;

void setup() {
  // 設定所有相關腳位為輸出
  pinMode(FRONT_PWM_A, OUTPUT);
  pinMode(FRONT_PWM_B, OUTPUT);
  pinMode(FRONT_AIN1, OUTPUT);
  pinMode(FRONT_AIN2, OUTPUT);
  pinMode(FRONT_BIN1, OUTPUT);
  pinMode(FRONT_BIN2, OUTPUT);
  
  pinMode(REAR_PWM_A, OUTPUT);
  pinMode(REAR_PWM_B, OUTPUT);
  pinMode(REAR_AIN1, OUTPUT);
  pinMode(REAR_AIN2, OUTPUT);
  pinMode(REAR_BIN1, OUTPUT);
  pinMode(REAR_BIN2, OUTPUT);
  
  Serial.begin(9600);
}

void loop() {
  // 1. 正轉測試：設定正轉方向，並以50% PWM 驅動
  // 前馬達：將 AIN1 設為 HIGH, AIN2 設為 LOW；BIN1 設為 HIGH, BIN2 設為 LOW
  digitalWrite(FRONT_AIN1, HIGH);
  digitalWrite(FRONT_AIN2, LOW);
  digitalWrite(FRONT_BIN1, HIGH);
  digitalWrite(FRONT_BIN2, LOW);
  
  // 後馬達同理
  digitalWrite(REAR_AIN1, HIGH);
  digitalWrite(REAR_AIN2, LOW);
  digitalWrite(REAR_BIN1, HIGH);
  digitalWrite(REAR_BIN2, LOW);
  
  // 使用 analogWrite 控制 PWM 輸出
  analogWrite(FRONT_PWM_A, pwmSpeed);
  analogWrite(FRONT_PWM_B, pwmSpeed);
  analogWrite(REAR_PWM_A, pwmSpeed);
  analogWrite(REAR_PWM_B, pwmSpeed);
  
  Serial.println("正轉中，以50% PWM輸出");
  delay(2000);  // 運行2秒
  
  // 2. 停止：將所有 PWM 設為0
  analogWrite(FRONT_PWM_A, 0);
  analogWrite(FRONT_PWM_B, 0);
  analogWrite(REAR_PWM_A, 0);
  analogWrite(REAR_PWM_B, 0);
  
  Serial.println("停止");
  delay(1000);
  
  // 3. 反轉測試：反轉時將方向腳位反向
  digitalWrite(FRONT_AIN1, LOW);
  digitalWrite(FRONT_AIN2, HIGH);
  digitalWrite(FRONT_BIN1, LOW);
  digitalWrite(FRONT_BIN2, HIGH);
  
  digitalWrite(REAR_AIN1, LOW);
  digitalWrite(REAR_AIN2, HIGH);
  digitalWrite(REAR_BIN1, LOW);
  digitalWrite(REAR_BIN2, HIGH);
  
  analogWrite(FRONT_PWM_A, pwmSpeed);
  analogWrite(FRONT_PWM_B, pwmSpeed);
  analogWrite(REAR_PWM_A, pwmSpeed);
  analogWrite(REAR_PWM_B, pwmSpeed);
  
  Serial.println("反轉中，以50% PWM輸出");
  delay(2000);
  
  // 4. 再次停止
  analogWrite(FRONT_PWM_A, 0);
  analogWrite(FRONT_PWM_B, 0);
  analogWrite(REAR_PWM_A, 0);
  analogWrite(REAR_PWM_B, 0);
  
  Serial.println("停止");
  delay(1000);
}
