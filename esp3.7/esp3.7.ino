#include <Adafruit_MAX31865.h>
#include <SHT3x.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include "FirebaseESP32.h"
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <SimpleKalmanFilter.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// cấu hình chân
#define CS 5
#define SDI 23
#define SDO 19
#define CLK 18

#define SCL 22
#define SDA 21

#define TRIAC_PIN 4  
#define zero_cross 25

#define led_wf  13 
#define led_fb  2
#define led_tt  15
#define quat 26

// Hằng số cho cảm biến
#define RREF  430.0
#define RNOMINAL  100.0

// thông tin WiFi và fb 
#define WIFI_SSID "DoAnTN2024"
#define WIFI_PASSWORD "98761234"
#define FIREBASE_HOST "projecttn-68455-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define FIREBASE_AUTH "CHPO2dEFllUzTC01rSH3r7PB1hRdDsfcVEZLV94e"

#define commentForDebug  // chạy đc dòng code bên trong

// Biến toàn cục

SHT3x Sensor;
Adafruit_MAX31865 thermo = Adafruit_MAX31865(CS, SDI, SDO, CLK);
FirebaseData firebaseData;
String path = "/";
FirebaseJson json;
HardwareSerial hmiSerial(2); // RX, TX
SimpleKalmanFilter locnhieu(0.3, 0.3, 0.01);
const long utcOffsetInSeconds = 25200;  // UTC+7
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);


//biến triac
esp_timer_handle_t tTriacHandle;
TaskHandle_t taskTriacHandle;
uint64_t ThoiGianKichTriac = 0;
bool TrangThaiGiaNhiet = false;

//thông số pid 
float NhietCaiDat;
static float kp, ki, kd;
float SampleTime = 1000; 
float WindupMax = 9600; 
float WindupMin = 0; 
float OutMax = 9600; 
float OutMin = 0;
float P_Tern, I_Tern, D_Tern;
float err;  
float feedBackWindup; //giá trị tự tính toán
double alpha = 0.2; // Hệ số lọc thông thấp (0 < alpha < 1)
float LastErr = 0; 
float LastD_Tern = 0; 
float kw = 1; 
float Output; 

float NhietDoHienTai, gtnd;
int H;
int sp_hmi, sp_esp, sp_fb;
int p_quat, p_den;
int tocdoquat = 255;
String formattedDate, formattedTime;

unsigned long currentMillis;
unsigned long lastMillis = 0;
const long interval = 500;                      // Thời gian chờ giữa các lần lặp (1 giây)
unsigned long lastFirebaseUpdateMillis = 0;     // Thêm biến này để kiểm soát thời gian gửi dữ liệu lên Firebase
const long firebaseUpdateInterval = 10000;      // Thời gian chờ giữa các lần gửi dữ liệu lên Firebase (10 giây)

float debugPID = 0;
int BatTatDieuKhienNhietDo = 0;
String tt_losay;

#pragma region Phần Điều Khiển Nhiệt Độ
void BatDieuKhienNhietDo(void)  // Hàm bật điều khiển nhiệt độ
{
  TrangThaiGiaNhiet = true; // Chuyển trạng thái điều khiển nhiệt sang ON
}

void TatDieuKhienNhietDo(void)
{
  TrangThaiGiaNhiet = false; // Chuyển trạng thái điều khiển nhiệt sang OFF
}

void KichTriac(uint8_t State) {
  digitalWrite(TRIAC_PIN, State);
}

void IRAM_ATTR PhatHienDiemKhong() {
  KichTriac(0);
  esp_timer_start_once(tTriacHandle, ThoiGianKichTriac);
}

void ZeroPointDelay_Callback(void* pvParameter) {
  if (ThoiGianKichTriac < 9600 && ThoiGianKichTriac >= 4200) {
    xTaskNotify(taskTriacHandle, 0x01, eSetValueWithoutOverwrite);  // Gọi đến TaskTriacTrigger
  }
}


void TaskTriacTrigger(void* pvParameter) {
  uint32_t notifyNum = 0;
  while (1) {
    if (xTaskNotifyWait(pdFALSE, pdTRUE, &notifyNum, portMAX_DELAY)) {
      if (TrangThaiGiaNhiet == true) {
        for (uint8_t i = 0; i < 50; i++) KichTriac(1);
        KichTriac(0);
      } else if (TrangThaiGiaNhiet == false) {
        KichTriac(0);
      }
    }
  }
}
#pragma endregion

#pragma region Hàm PID
float PIDControl(float GiaTriSetPoint, float GiaTriNhietDo){
  //tính sai số
  err =  GiaTriSetPoint - GiaTriNhietDo;

  //khâu P
  P_Tern = kp*err;

  //khâu I và antiwundup
  I_Tern += (ki * err + feedBackWindup * kw) * (SampleTime /1000.0);
  if (I_Tern > WindupMax){
    I_Tern = WindupMax;
  } else if (I_Tern < WindupMin){
    I_Tern = WindupMin;
  }
  
  //khâu D và lọc thông thấp
  D_Tern = (err - LastErr)/(SampleTime/1000); //tinh đạo hàm
  // Áp dụng bộ lọc thông thấp để làm mượt D_Tern
  D_Tern = alpha * D_Tern + (1 - alpha) * LastD_Tern; // thấy cần thêm hệ số alpha như công thức PID
  LastErr = err; // Cập nhật LastErr cho lần tính toán tiếp theo
  LastD_Tern = D_Tern; // Cập nhật LastD_Tern cho lần tính toán tiếp theo

  //Output
  Output = P_Tern + I_Tern + D_Tern * kd;
  if (Output > OutMax){
    feedBackWindup = OutMax - Output;
    Output = OutMax;
  } else if(Output < OutMin ){
    feedBackWindup = OutMax - Output;
    Output = OutMin;
  } else {
    feedBackWindup = 0;
  }
  return Output;
}

//chia pid trong các khoảng khác nhau
void setPIDParam(float k_p, float k_i, float k_d) {
  kp = k_p;
  ki = k_i;
  kd = k_d;
}
#pragma endregion

void setup() {
  Serial.begin(115200);
  Sensor.Begin();
  if (!thermo.begin(MAX31865_4WIRE)) {
    Serial.println("Failed to initialize MAX31865. Check wiring and sensor.");
    while (10);
  } else {
    Serial.println("MAX31865 initialized successfully.");
  }

  pinMode(TRIAC_PIN, OUTPUT);
  pinMode(zero_cross, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(zero_cross), PhatHienDiemKhong, FALLING);

  esp_timer_create_args_t timer_arg;
  timer_arg.callback = ZeroPointDelay_Callback;
  timer_arg.dispatch_method = ESP_TIMER_TASK;
  timer_arg.name = "TimerTriac";
  timer_arg.arg = NULL;

  esp_timer_create(&timer_arg, &tTriacHandle);
  esp_timer_stop(tTriacHandle);
  xTaskCreate(TaskTriacTrigger, "TaskTriacTrigger", 2048, NULL, 3, &taskTriacHandle);

  tt_losay = "off";
  hmiSerial.begin(115200, SERIAL_8N1, 16, 17);

  pinMode(led_wf, OUTPUT);
  pinMode(led_fb, OUTPUT);
  pinMode(led_tt, OUTPUT);
  pinMode(quat, OUTPUT);
  digitalWrite(led_fb, LOW);
  ledcAttachPin(quat, 0);
  ledcSetup(0, 1000, 8);
  ledcWrite(0, tocdoquat);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  if(WiFi.status() != WL_CONNECTED) {
    delay(3000);
    Serial.print(".");
  }
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  Firebase.setwriteSizeLimit(firebaseData, "tiny");

  if (!Firebase.beginStream(firebaseData, path)) {
    Serial.println("REASON: " + firebaseData.errorReason());
    Serial.println();
  } else {
    Firebase.setString(firebaseData, "/App/setpoint", NULL);
    Firebase.setString(firebaseData, "/App/ttlosay", "off");
    Firebase.setInt(firebaseData, "/App/tocdoquat", 255);
  }

  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  timeClient.begin();
}


void DocDoAm() {
  Sensor.UpdateData();
  float humi = Sensor.GetRelHumidity();
  if (humi >= 0) {
    H = humi;
  }
}
void setPIDParamsBasedOnTemp() {
  if ((NhietCaiDat >= 35.0) && (NhietCaiDat < 50.0)) { 
    setPIDParam(600.0, 5.0, 20.0);  
  } else if ((NhietCaiDat >= 50.0) && (NhietCaiDat < 60.0)) {
    setPIDParam(480.0, 3.85, 20.0);
  } else if ((NhietCaiDat >= 60.0) && (NhietCaiDat < 70.0)) {
    setPIDParam(425, 3.55, 15);
  } else if ((NhietCaiDat >= 70.0) && (NhietCaiDat < 80.0)) {
    setPIDParam(470.0, 3.3, 15);
  }
}

String getFormattedDate(NTPClient& timeClient) {
  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime((time_t *)&epochTime);
  int day = ptm->tm_mday;
  int month = ptm->tm_mon + 1;
  int year = ptm->tm_year + 1900;
  return String(day) + "/" + String(month) + "/" + String(year);
}
String getFormattedTime(NTPClient& timeClient) {
  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime((time_t *)&epochTime);
  int hours = ptm->tm_hour;
  int minutes = ptm->tm_min;
  int seconds = ptm->tm_sec;
  return String(hours) + ":" + String(minutes) + ":" + String(seconds);
}

void sendCommandToHMI(String command) {
  hmiSerial.print(command);
  hmiSerial.write(0xff);
  hmiSerial.write(0xff);
  hmiSerial.write(0xff);
}

void SendDataToHMI(String id, String value, String id1, String value1, String id2, String value2, String id3, String value3, String id4, String value4) {
  sendCommandToHMI(id + value + "\"");
  sendCommandToHMI(id1 + value1 + "\"");
  sendCommandToHMI(id2 + value2 + "\"");
  sendCommandToHMI(id3 + value3 + "\"");
  sendCommandToHMI(id4 + value4 + "\"");
}
void SendDataBar(String id, int value, String id1, int value1) {
  sendCommandToHMI(id + String(value));
  sendCommandToHMI(id1 + String(value1));
}

void  SentDataWaveform(int ID_NO, int channel, int amp) {
  char buf[50];
  
  int ns = 15;
  int ndht = NhietDoHienTai ;
  int ndd = NhietCaiDat;
  
  for (int t = 0; t < ns; t++) {
    int len = sprintf(buf, "add %d,%d,%d", ID_NO, channel, ndht);
    hmiSerial.print(buf);
    hmiSerial.write(0xFF);
    hmiSerial.write(0xFF);
    hmiSerial.write(0xFF);
    len = sprintf(buf, "add %d,%d,%d", ID_NO, channel + 1, ndd);
    hmiSerial.print(buf);
    hmiSerial.write(0xFF);
    hmiSerial.write(0xFF);
    hmiSerial.write(0xFF);
  }
}

void SendtoHMI() {
  //gữi dữ liệu đến HMI
  SendDataToHMI("t0.txt=\"", String(NhietDoHienTai), "t8.txt=\"", String(NhietCaiDat), "t2.txt=\"", String(p_quat), "t3.txt=\"", String(p_den), "t1.txt=\"", String(H));
  SendDataBar("j0.val=", NhietDoHienTai, "j1.val=", H);
  SentDataWaveform(1, 0, 0);
}


void SendDataToFirebase() {
  if (Firebase.ready()) {
    Firebase.setString(firebaseData, "/ESP_sent/TempString", String(NhietDoHienTai));
    Firebase.setString(firebaseData, "/ESP_sent/HumidString", String(H));
    Firebase.setString(firebaseData, "/ESP_sent/SetpointString", String(NhietCaiDat));
    Firebase.setString(firebaseData, "/ESP_sent/FanString", String(p_quat));
    Firebase.setString(firebaseData, "/ESP_sent/LedString", String(p_den));
    Firebase.setString(firebaseData, "/ESP_sent/DateString", formattedDate);
    Firebase.setString(firebaseData, "/ESP_sent/TimeString", formattedTime);
  }
}
void GetFirebaseData() {
  if (Firebase.ready()) {
    Firebase.getString(firebaseData, path + "/App/setpoint");
    String sp_fb_str = firebaseData.stringData();
    sp_fb = sp_fb_str.toInt();
    sp_esp = sp_fb;

    Firebase.getInt(firebaseData, "/App/tocdoquat");
    tocdoquat = firebaseData.intData();

    Firebase.getString(firebaseData, "/App/ttlosay");
    tt_losay = firebaseData.stringData();
    if(strstr(tt_losay.c_str(), "off")) {
      sp_esp = 0;
    }
  }
}

void checkWiFiStatus() {
  //kiểm tra kết nối wifi
  if(WiFi.status() != WL_CONNECTED) {
    digitalWrite(led_wf, LOW);
  }else{ 
    digitalWrite(led_wf, HIGH);
  }
}

void updateFromHMI(String chuoi_hmi_str) {
  const char* input = chuoi_hmi_str.c_str();
  
  // Đọc setpoint từ HMI
  const char* key_sp = "set";
  const char* p = strstr(input, key_sp);
  if (p != NULL) {
    p += strlen(key_sp);
    sp_hmi = atof(p);
    sp_esp = sp_hmi;
    tt_losay = "on";
  }

  // Đọc trạng thái lò từ HMI
  if (strstr(input, "off")) {
    tt_losay = "off";
    sp_esp = 0;
  }

  // Đọc tốc độ quạt từ HMI
  const char* key_fan = "fan";
  const char* p1 = strstr(input, key_fan);
  if (p1 != NULL) {
    p1 += strlen(key_fan);
    tocdoquat = atof(p1);
  }
}

void sendDataToFirebase(String chuoi_hmi_str) {
  const char* input = chuoi_hmi_str.c_str();
  
  // Gửi setpoint và trạng thái lò lên Firebase
  const char* key_sp = "set";
  const char* p = strstr(input, key_sp);
  if (p != NULL) {
    p += strlen(key_sp);
    sp_hmi = atof(p);
    Firebase.setString(firebaseData, "/App/setpoint", String(sp_hmi));
    Firebase.setString(firebaseData, "/App/ttlosay", "on");
  }

  if (strstr(input, "off")) {
    Firebase.setString(firebaseData, "/App/ttlosay", "off");
    Firebase.setString(firebaseData, "/App/setpoint", NULL);
  }

  // Gửi tốc độ quạt lên Firebase
  const char* key_fan = "fan";
  const char* p1 = strstr(input, key_fan);
  if (p1 != NULL) {
    p1 += strlen(key_fan);
    int fan = atof(p1);
    Firebase.setInt(firebaseData, "/App/tocdoquat", fan);
  }
}

void loop() {

  currentMillis = millis();
  // Kiểm tra thời gian để thực hiện các hành động
  if ((currentMillis - lastMillis) >= interval) {
    lastMillis = currentMillis;
    DocDoAm();  
    gtnd = thermo.temperature(RNOMINAL, RREF);
    NhietDoHienTai= locnhieu.updateEstimate(gtnd);
    
    ledcWrite(0, tocdoquat);
    p_quat = (tocdoquat * 100) / 255;                           // công suất quạt
    p_den = ((9600 - ThoiGianKichTriac) * 100) / (9600 - 4200); // công suất gia nhiệt
    
    //kiểm tra kết nối fb và xử lý 
    if (!Firebase.beginStream(firebaseData, path)) {
      digitalWrite(led_fb, LOW);
      //Đọc từ data HMI
      if (hmiSerial.available()) {
        String chuoi_hmi_str = hmiSerial.readString(); 
        updateFromHMI(chuoi_hmi_str);
      }
    } else {
      digitalWrite(led_fb, HIGH); 
      
      if ((currentMillis - lastFirebaseUpdateMillis) >= firebaseUpdateInterval) {
        lastFirebaseUpdateMillis = currentMillis;
        timeClient.update();
        formattedDate = getFormattedDate(timeClient);
        formattedTime = getFormattedTime(timeClient);
        if (hmiSerial.available()) {
          String chuoi_hmi_str = hmiSerial.readString();
          sendDataToFirebase(chuoi_hmi_str);
        }
        //Đọc dữ liệu từ fb
        GetFirebaseData();
        //cập nhật dữ liệu lên firebase
        SendDataToFirebase(); 
        
      }    
    }

    // Kiểm soát nhiệt độ
    if (sp_esp > 34 && sp_esp < 80) {
      digitalWrite(led_tt, HIGH);
      NhietCaiDat = sp_esp;
      BatDieuKhienNhietDo();
      if ((NhietDoHienTai > NhietCaiDat)) {
        ThoiGianKichTriac = 9600;
      } else {
        setPIDParamsBasedOnTemp();
        debugPID = PIDControl(NhietCaiDat, NhietDoHienTai);
        ThoiGianKichTriac = map(debugPID, 0, 9600, 9600, 4200);
      }
    } else if (strstr(tt_losay.c_str(), "off")) {
      digitalWrite(led_tt, LOW);
      TatDieuKhienNhietDo();
      ThoiGianKichTriac = 9600;
      sp_esp = 0;
      NhietCaiDat = 0;
    }
    
    //gữi dữ liệu đến HMI
    SendtoHMI();
  
    //kiểm tra kết nối wifi
    checkWiFiStatus();

    #ifdef commentForDebug
      Serial.print("Nhiet Do Cai Dat: "); 
      Serial.print(NhietCaiDat);
      Serial.print("|");
      Serial.print("Nhiet Do Hien Tai: "); 
      Serial.print(NhietDoHienTai);
      Serial.print("|");
      Serial.print("Do am hien tai: ");
      Serial.print(H);
      Serial.print("%");
      Serial.print("\n");
    #endif
  }
}
