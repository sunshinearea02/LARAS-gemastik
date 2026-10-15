#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Semua struct & enum ditaruh paling atas, supaya Arduino IDE tidak error
// "does not name a type" (IDE membuat prototipe fungsi otomatis di bagian atas file).
struct Key     { uint8_t src; uint32_t seq; uint8_t tr; uint32_t t; };
struct Relayed { uint8_t src; uint32_t seq; uint32_t t; };
enum TxType    { TX_DATA, TX_ACK, TX_BEACON };
struct Tx {
  bool active; TxType type; String pkt; uint32_t due;
  uint8_t src; uint32_t seq; uint8_t tr;
  uint32_t bNo; uint8_t bTtl; uint32_t bOff; uint32_t bRxAt;   // khusus beacon
};

// ================= UBAH PER NODE =================
#define NODE_ID   2     // FILE INI KHUSUS NODE 1 (jangan diubah)
#define HAS_RAIN  0 
#define SIM_RAIN 1 // Node 1 punya rain gauge
// =================================================

// ---------- Radio (HARUS sama di semua node dan gateway) ----------
#define FREQ        920E6
#define SYNC_WORD   0xF3
#define TX_POWER    10      // uji di meja: 2-10. Naikkan ke 17-20 saat dipasang di lapangan

// ---------- Jaringan ----------
#define MAX_TTL          2       // maksimal 2 lompatan: node -> relay -> gateway
#define MAX_RETRY        3       // total percobaan kirim per paket
#define ACK_TIMEOUT_MS   2000    // tunggu ACK sebelum dianggap gagal
#define GW_FRESH_MS      60000   // relay hanya kalau dengar gateway langsung < 60 dtk terakhir
#define DEDUP_WINDOW_MS  60000   // catatan paket ganda berlaku 60 dtk
#define BEACON_AIR_MS    40      // perkiraan lama beacon di udara (SF7)

// ---------- Jadwal (HARUS sama dengan gateway) ----------
#define CYCLE_MS         10000   // 1 putaran
#define FIRST_SLOT_MS    1000    // slot pertama mulai detik ke-1
#define SLOT_MS          4000    // panjang 1 slot
#define RETRY_MARGIN_MS  2300    // sisa slot minimal supaya boleh kirim ulang di slot yang sama
#define BEACON_LOST_MS   30000   // 30 dtk tanpa beacon -> mode cadangan

// ---------- Mode cadangan (tanpa beacon) ----------
#define SEND_INTERVAL_MS 10000
#define JITTER_MS        1500

// ---------- Pin LoRa Dragino ----------
#define LORA_CS    10
#define LORA_MOSI  11
#define LORA_SCK   12
#define LORA_MISO  13
#define LORA_RST   15
#define LORA_DIO0  7
// ---------- Pin I2C MPU6050 ----------
#define I2C_SDA    8
#define I2C_SCL    9
// ---------- Pin sensor lingkungan ----------
#define PIN_SOIL_SHALLOW  4   // 25 cm
#define PIN_SOIL_DEEP     5   // 75 cm
#define PIN_RAIN_PULSE    6   // tipping bucket

// ---------- Kalibrasi ----------
#define SOIL_DRY    4095      // nilai mentah kering (udara)
#define SOIL_WET    1500      // nilai mentah basah (air)
#define MM_PER_TIP  0.2

// =================================================================
//                              SENSOR
// =================================================================
Adafruit_MPU6050 mpu;
bool mpuOk = false;
uint32_t lastMpuTry = 0;

// Nyalakan / bangunkan MPU6050. Dipanggil saat start dan saat MPU terbaca 0
// (MPU yang sempat kehilangan daya kembali ke mode tidur dan membaca semua 0).
bool initMpu() {
  lastMpuTry = millis();
  if (!mpu.begin()) return false;
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  return true;
}
 
volatile uint32_t rainTips = 0;
volatile uint32_t lastTipMs = 0;
uint32_t sentTips = 0;

void IRAM_ATTR countRain() {
  uint32_t now = millis();
if (now - lastTipMs > 500) { rainTips = rainTips + 1; lastTipMs = now; }   // anti-pantul (maks 2 tip/detik) 
}
int avgRead(int pin) {
  long s = 0;
  for (int i = 0; i < 10; i++) { s += analogRead(pin); delay(3); }
  return s / 10;
}

float soilPct(int raw) {
  float pct = (float)(SOIL_DRY - raw) * 100.0f / (float)(SOIL_DRY - SOIL_WET);
  return constrain(pct, 0.0f, 100.0f);
}

// 15 kolom (urutan = kolom sensor_data):
// moisture,moisture_percent,moisture_percent_deep,rainfall_mm,rain_tips_total,
// ax,ay,az,gx,gy,gz,pitch,roll,battery_percent,battery_voltage
String readSensorsPayload() {
  int raw     = avgRead(PIN_SOIL_SHALLOW);
  int rawDeep = avgRead(PIN_SOIL_DEEP);

  String rainStr = "", tipsStr = "";
#if HAS_RAIN
  noInterrupts(); uint32_t tips = rainTips; interrupts();
  rainStr = String((tips - sentTips) * MM_PER_TIP, 2);
  tipsStr = String(tips);
  sentTips = tips;   // catatan: kalau paket gagal, server tetap bisa menghitung dari rain_tips_total
#elif SIM_RAIN
  // Hujan SIMULASI: pola mirip hujan asli, bukan angka acak murni.
  // Rata-rata hujan mulai tiap ~1,5 jam, berlangsung ~10 menit, intensitas ~15 mm/jam
  // (total ~35 mm/hari, mirip hari hujan di musim penghujan).
  static bool     simRaining = false;
  static uint32_t simTips    = 0;          // counter kumulatif, sama seperti rain gauge asli
  if (!simRaining && random(0, 540) == 0)     simRaining = true;
  else if (simRaining && random(0, 60) == 0)  simRaining = false;
  uint32_t newTips = (simRaining && random(0, 100) < 20) ? 1 : 0;
  simTips += newTips;
  rainStr = String(newTips * MM_PER_TIP, 2);
  tipsStr = String(simTips);
#endif

String imu = ",,,,,,,";   // 8 kolom kosong kalau MPU tidak terbaca
  if (!mpuOk && millis() - lastMpuTry > 10000) {
    mpuOk = initMpu();                      // coba sambung ulang tiap 10 detik
    if (mpuOk) Serial.println("MPU6050 tersambung lagi.");
  }
  if (mpuOk) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
    if (sqrt(ax * ax + ay * ay + az * az) < 2.0) {
      // Gravitasi selalu ~9,8 m/s2. Total < 2 berarti MPU reset/tertidur atau kabel I2C putus:
      // jangan kirim angka 0 (AI akan mengira guncangan besar), kirim kosong lalu bangunkan ulang.
      Serial.println("!! MPU6050 membaca 0 (daya drop / kabel SDA-SCL). Inisialisasi ulang...");
      mpuOk = initMpu();
      return String(raw) + "," + String(soilPct(raw), 2) + "," + String(soilPct(rawDeep), 2) + "," +
             rainStr + "," + tipsStr + "," + imu + ",,";
    }
    float gx = g.gyro.x,         gy = g.gyro.y,         gz = g.gyro.z;
    float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    float roll  = atan2(ay, az) * 180.0 / PI;
    imu = String(ax, 3) + "," + String(ay, 3) + "," + String(az, 3) + "," +
          String(gx, 3) + "," + String(gy, 3) + "," + String(gz, 3) + "," +
          String(pitch, 2) + "," + String(roll, 2);
  }


  return String(raw) + "," + String(soilPct(raw), 2) + "," + String(soilPct(rawDeep), 2) + "," +
         rainStr + "," + tipsStr + "," + imu + ",,";   // baterai % dan V belum dipasang
}

// =================================================================
//                         CATATAN PAKET GANDA
// =================================================================
Key seenData[20]; uint8_t seenDataIdx = 0;
Key seenAck[20];  uint8_t seenAckIdx = 0;

// true = paket ini sudah pernah didengar (dalam 60 dtk terakhir)
bool seenBefore(Key *buf, uint8_t &idx, uint8_t src, uint32_t sq, uint8_t tr) {
  uint32_t now = millis();
  for (int i = 0; i < 20; i++)
    if (buf[i].t && buf[i].src == src && buf[i].seq == sq && buf[i].tr == tr &&
        now - buf[i].t < DEDUP_WINDOW_MS) return true;
  buf[idx] = {src, sq, tr, now | 1};
  idx = (idx + 1) % 20;
  return false;
}

// Data siapa saja yang barusan kita relay (supaya ACK-nya kita teruskan balik)
Relayed relayed[6]; uint8_t relayedIdx = 0;

void rememberRelay(uint8_t src, uint32_t sq) {
  relayed[relayedIdx] = {src, sq, millis() | 1};
  relayedIdx = (relayedIdx + 1) % 6;
}

bool didRelay(uint8_t src, uint32_t sq) {
  for (int i = 0; i < 6; i++)
    if (relayed[i].t && relayed[i].src == src && relayed[i].seq == sq &&
        millis() - relayed[i].t < DEDUP_WINDOW_MS) return true;
  return false;
}

// =================================================================
//                       ANTREAN KIRIM (RELAY)
// =================================================================
Tx q[4];

bool enqueue(const Tx &t) {
  for (int i = 0; i < 4; i++)
    if (!q[i].active) { q[i] = t; q[i].active = true; return true; }
  Serial.println("Antrean penuh, paket relay dibuang");
  return false;
}

void cancelRelay(uint8_t src, uint32_t sq) {
  for (int i = 0; i < 4; i++)
    if (q[i].active && q[i].type == TX_DATA && q[i].src == src && q[i].seq == sq) {
      q[i].active = false;
      Serial.printf("Relay N%u #%lu dibatalkan (gateway sudah terima)\n", src, (unsigned long)sq);
    }
}

void setupLoRa() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setSPI(SPI);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  while (!LoRa.begin(FREQ)) {
    Serial.println("LoRa gagal inisialisasi! Cek kabel SPI & daya modul. Coba lagi 2 detik...");
    delay(2000);
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(SYNC_WORD);
  LoRa.setTxPower(TX_POWER);
  LoRa.enableCrc();
}

// Kirim paket dengan batas waktu. Library aslinya menunggu selamanya kalau modul
// tidak pernah melapor "selesai kirim" (biasanya karena daya modul drop saat memancar).
bool radioSend(const String &pkt) {
  LoRa.beginPacket();
  LoRa.print(pkt);
  LoRa.endPacket(true);                  // mulai kirim, jangan menunggu di dalam library
  uint32_t t0 = millis();
  delay(5);
  while (!LoRa.beginPacket()) {          // beginPacket() = 0 selama modul masih memancar
    if (millis() - t0 > 1500) {
      Serial.println("!! TX MACET > 1,5 detik. Kemungkinan daya modul LoRa kurang saat memancar.");
      Serial.println("!! Cek kabel 5V/3.3V/GND modul, atau turunkan TX_POWER. Modul di-reset ulang...");
      LoRa.end();
      setupLoRa();
      return false;
    }
    delay(2);
  }
  LoRa.idle();                           // batalkan paket kosong dari beginPacket() di atas
  return true;
}

String dataPkt(uint8_t src, uint32_t sq, uint8_t tr, uint8_t ttl, uint8_t via, const String &payload) {
  return "D," + String(src) + "," + String(sq) + "," + String(tr) + "," +
         String(ttl) + "," + String(via) + "|" + payload;
}

// =================================================================
//                     STATUS GATEWAY & JADWAL
// =================================================================
bool     gwHeard = false;        // pernah dengar gateway secara langsung?
uint32_t lastGatewayHeard = 0;

bool     beaconOk = false;       // pernah dapat beacon?
uint32_t lastBeaconRx = 0;       // kapan terakhir dapat beacon
uint32_t lastBeaconNo = 0;
uint32_t cycleAnchor = 0;        // waktu (millis) awal putaran beacon terakhir
uint32_t anchorNo = 0;           // nomor beacon terakhir

bool gwFresh()    { return gwHeard  && millis() - lastGatewayHeard < GW_FRESH_MS; }
bool tdmaActive() { return beaconOk && millis() - lastBeaconRx     < BEACON_LOST_MS; }

// =================================================================
//                         KIRIM DATA SENDIRI
// =================================================================
uint32_t seq = 0;
String   ownPayload;
bool     pending = false;        // ada data yang belum ter-ACK
bool     waitingAck = false;     // sedang menunggu ACK
uint8_t  tries = 0;
uint32_t ackDeadline = 0;
uint32_t nextSend = 0;           // khusus mode cadangan
uint32_t lastTxCycle = 0xFFFFFFFF;

void sendOwn() {
  tries++;
  String pkt = dataPkt(NODE_ID, seq, tries, MAX_TTL, NODE_ID, ownPayload);
  bool ok = radioSend(pkt);
  Serial.printf("[%s] TX #%lu (percobaan %d, %d byte)%s\n",
                tdmaActive() ? "JADWAL" : "CADANGAN", (unsigned long)seq, tries, pkt.length(),
                ok ? "" : " -> GAGAL MEMANCAR");
  waitingAck = true;
  ackDeadline = millis() + ACK_TIMEOUT_MS + random(0, 300);
}

void finishOwn(bool ok) {
  if (!ok) Serial.printf("GAGAL: #%lu tidak ter-ACK setelah %d kali\n", (unsigned long)seq, tries);
  pending = false;
  waitingAck = false;
  seq++;
}

// =================================================================
//                         TERIMA PAKET
// =================================================================
void handlePacket(const String &s, int rssi) {
  uint32_t now = millis();

  // ---------------- BEACON ----------------
  if (s.startsWith("B,")) {
    unsigned long n, off; unsigned int ttl;
    if (sscanf(s.c_str(), "B,%lu,%u,%lu", &n, &ttl, &off) != 3) return;
    if (ttl == MAX_TTL) { gwHeard = true; lastGatewayHeard = now; }   // langsung dari gateway
    if (beaconOk && n == lastBeaconNo) return;                        // sudah dapat beacon ini
    lastBeaconNo = n;
    anchorNo     = n;
    cycleAnchor  = now - off;     // koreksi kalau beacon datang lewat relay
    lastBeaconRx = now;
    if (!beaconOk) Serial.println("Beacon diterima -> MODE JADWAL aktif");
    beaconOk = true;

    if (ttl > 1) {                // teruskan beacon untuk node yang jauh
      Tx t = {}; t.type = TX_BEACON; t.due = now + NODE_ID * 120;   // urutan per node, tidak tabrakan
      t.bNo = n; t.bTtl = ttl - 1; t.bOff = off; t.bRxAt = now;
      enqueue(t);
    }
    return;
  }

  // ---------------- ACK ----------------
  if (s.startsWith("A,")) {
    unsigned int src, tr, ttl; unsigned long sq;
    if (sscanf(s.c_str(), "A,%u,%lu,%u,%u", &src, &sq, &tr, &ttl) != 4) return;
    if (ttl == MAX_TTL) { gwHeard = true; lastGatewayHeard = now; }   // ACK asli dari gateway

    if (src == NODE_ID) {                                             // ACK untuk kita
      if (pending && sq == seq) {
        Serial.printf("ACK #%lu diterima (RSSI %d, %s)\n", sq, rssi,
                      ttl == MAX_TTL ? "langsung" : "lewat relay");
        finishOwn(true);
      }
      return;
    }

    cancelRelay(src, sq);                                             // gateway sudah terima
    if (ttl > 1 && didRelay(src, sq) && !seenBefore(seenAck, seenAckIdx, src, sq, tr)) {
      Tx t = {}; t.type = TX_ACK; t.due = now + random(150, 400);
      t.pkt = "A," + String(src) + "," + String(sq) + "," + String(tr) + "," + String(ttl - 1);
      t.src = src; t.seq = sq; t.tr = tr;
      enqueue(t);
    }
    return;
  }

  // ---------------- DATA node lain ----------------
  if (s.startsWith("D,")) {
    int bar = s.indexOf('|');
    if (bar < 0) return;
    unsigned int src, tr, ttl, via; unsigned long sq;
    if (sscanf(s.c_str(), "D,%u,%lu,%u,%u,%u", &src, &sq, &tr, &ttl, &via) != 5) return;
    if (src == NODE_ID || src == 0) return;
    if (seenBefore(seenData, seenDataIdx, src, sq, tr)) return;
    if (ttl <= 1) return;
    if (!gwFresh()) {
      Serial.printf("Dengar N%u #%lu (RSSI %d), TIDAK relay (gateway tidak terdengar)\n", src, sq, rssi);
      return;
    }
    Tx t = {}; t.type = TX_DATA; t.due = now + random(300, 600);
    t.pkt = dataPkt(src, sq, tr, ttl - 1, NODE_ID, s.substring(bar + 1));
    t.src = src; t.seq = sq; t.tr = tr;
    if (enqueue(t)) Serial.printf("Dengar N%u #%lu (RSSI %d) -> siap relay\n", src, sq, rssi);
  }
}

// =================================================================
//                              SETUP
// =================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  randomSeed(esp_random());

  Wire.begin(I2C_SDA, I2C_SCL);
  mpuOk = mpu.begin();
  if (mpuOk) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 siap.");
  } else {
    Serial.println("MPU6050 gagal dideteksi!");
  }

#if HAS_RAIN
  pinMode(PIN_RAIN_PULSE, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_RAIN_PULSE), countRain, FALLING);
#endif
#if !HAS_RAIN && SIM_RAIN
  Serial.println("PERHATIAN: data hujan Node ini SIMULASI (SIM_RAIN 1). Set ke 0 sebelum dipasang di lapangan.");
#endif
  setupLoRa();
  Serial.printf("Node %d siap. Menunggu beacon... (kalau tidak ada, kirim mode cadangan dalam ~10 detik)\n", NODE_ID);

  // Beri waktu ~1 putaran untuk mendengar beacon sebelum mode cadangan mulai kirim
  nextSend = millis() + CYCLE_MS + random(0, JITTER_MS);
}

// =================================================================
//                              LOOP
// =================================================================
void loop() {
  uint32_t now = millis();

  if (tdmaActive()) {
    // ---------- MODE JADWAL ----------
    uint32_t k        = (now - cycleAnchor) / CYCLE_MS;          // putaran ke-k sejak beacon terakhir
    uint32_t cycleNo  = anchorNo + k;
    uint32_t cStart   = cycleAnchor + k * CYCLE_MS;
    uint32_t sStart   = cStart + FIRST_SLOT_MS + (NODE_ID - 1) * SLOT_MS;
    uint32_t sEnd     = sStart + SLOT_MS;
    bool inSlot = (int32_t)(now - sStart) >= 0 && (int32_t)(now - sEnd) < 0;

    if (inSlot && cycleNo != lastTxCycle) {
      // Awal slot kita: kirim data baru, atau lanjutkan data lama yang belum ter-ACK
      lastTxCycle = cycleNo;
      if (pending && tries >= MAX_RETRY) finishOwn(false);
      if (!pending) {
        ownPayload = readSensorsPayload();
        tries = 0;
        pending = true;
      }
      sendOwn();
    } else if (waitingAck && (int32_t)(now - ackDeadline) >= 0) {
      waitingAck = false;
      if (tries >= MAX_RETRY) finishOwn(false);
      else if (inSlot && (int32_t)(sEnd - now) > RETRY_MARGIN_MS) sendOwn();   // masih cukup waktu
      else Serial.println("Slot hampir habis, kirim ulang di putaran berikutnya");
    }
  } else {
    // ---------- MODE CADANGAN (tidak ada beacon) ----------
    if (!pending && (int32_t)(now - nextSend) >= 0) {
      ownPayload = readSensorsPayload();
      tries = 0;
      pending = true;
      sendOwn();
      nextSend = now + SEND_INTERVAL_MS + random(-JITTER_MS, JITTER_MS);
    } else if (pending && !waitingAck) {
      // data tertunda dari mode jadwal
      if (tries >= MAX_RETRY) finishOwn(false); else sendOwn();
    } else if (waitingAck && (int32_t)(now - ackDeadline) >= 0) {
      waitingAck = false;
      if (tries >= MAX_RETRY) finishOwn(false); else sendOwn();
    }
  }

  // ---------- Dengarkan ----------
  int size = LoRa.parsePacket();
  if (size) {
    String s = "";
    while (LoRa.available()) s += (char)LoRa.read();
    handlePacket(s, LoRa.packetRssi());
  }

  // ---------- Antrean relay ----------
  for (int i = 0; i < 4; i++) {
    if (q[i].active && (int32_t)(millis() - q[i].due) >= 0) {
      if (q[i].type == TX_BEACON) {
        uint32_t off = q[i].bOff + (millis() - q[i].bRxAt) + BEACON_AIR_MS;
        String b = "B," + String(q[i].bNo) + "," + String(q[i].bTtl) + "," + String(off);
        radioSend(b);
        Serial.println("RELAY beacon: " + b);
      } else {
        radioSend(q[i].pkt);
        if (q[i].type == TX_DATA) rememberRelay(q[i].src, q[i].seq);
        Serial.println(String(q[i].type == TX_DATA ? "RELAY data: " : "RELAY ACK: ") + q[i].pkt);
      }
      q[i].active = false;
    }
  }
}
