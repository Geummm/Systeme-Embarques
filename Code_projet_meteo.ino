

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <EEPROM.h>
#include "RTClib.h"
#include <SoftwareSerial.h>
#include <ChainableLED.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// ========================== PINS ===============================
#define PIN_LED_CLK    6
#define PIN_LED_DATA   7
#define NUM_LEDS       1
ChainableLED rgbLed(PIN_LED_CLK, PIN_LED_DATA, NUM_LEDS);

#define PIN_BOUTON_ROUGE 3
#define PIN_BOUTON_VERT  2

#define PIN_GPS_RX       8
#define PIN_GPS_TX       9
SoftwareSerial gpsSerial(PIN_GPS_RX, PIN_GPS_TX);

#define PIN_SD_CS        4
#define PIN_LUMINOSITE   A0

// *** BME280 (I2C) ***
Adafruit_BME280 bme;
bool bmeDisponible = false;

// ======================= CONSTANTES =============================
#define TEMPS_APPUI_LONG 5000
#define TIMEOUT_CONFIG   1800000UL
#define DEBOUNCE_TIME    50

// ======================== ENUMS =================================
enum ModeSysteme { MODE_STANDARD, MODE_CONFIGURATION, MODE_MAINTENANCE, MODE_ECONOMIQUE };
enum EtatLED { LED_ETEINTE, LED_VERT_CONTINU, LED_JAUNE_CONTINU, LED_BLEU_CONTINU,
               LED_ORANGE_CONTINU, ERREUR_RTC_RB, ERREUR_GPS_RY, ERREUR_CAPTEUR_RG,
               ERREUR_DATA_RGV, ERREUR_SD_PLEINE_RW, ERREUR_SD_WRITE_RWB };

// ====================== STRUCTURES =============================
typedef struct {
  uint16_t LOG_INTERVAL;
  uint32_t FILE_MAX_SIZE;
  uint16_t TIMEOUT_CAPTEUR;
  uint16_t LUMIN_LOW;
  uint16_t LUMIN_HIGH;
} Config;

typedef struct {
  char timestamp[20];
  uint16_t luminosite;
  float temperature_air;
  float humidite_air;
  float pression_air;     // Champ ajouté pour la pression
  char gpsData[32];
} DataPoint;

// ====================== VARIABLES GLOBALES =====================
ModeSysteme modeActuel;
Config configSysteme;
RTC_DS1307 rtc;

bool rtcDisponible = true;
bool sdDisponible = true;

unsigned long dernierTempsMesure = 0;
unsigned long dernierTempsActiviteConfig = 0;
int compteurMesuresEco = 0;

volatile bool drapeauEvenementBoutonRouge = false;
volatile bool drapeauEvenementBoutonVert = false;

unsigned long tempsDebutAppuiRouge = 0;
unsigned long tempsDebutAppuiVert = 0;
unsigned long tempsDernierChangementRouge = 0;
unsigned long tempsDernierChangementVert = 0;

EtatLED etatLedActuel = LED_ETEINTE;
unsigned long tempsDernierChangementLed = 0;
bool etatBlinkCouleur1 = true;

// ======================== ISR ==================================
void isr_bouton_rouge() { drapeauEvenementBoutonRouge = true; }
void isr_bouton_vert()  { drapeauEvenementBoutonVert = true; }

// =================== PROTOTYPES =================================
void gererLogiqueBoutons();
void verifierAppuiLong();
void mettreAJourEtatLED(ModeSysteme mode);
void setColorLED(byte r, byte g, byte b);
void gererAffichageLED();
void gererModeStandard(Config* cfg);
void gererModeConfiguration(Config* cfg);
void gererModeMaintenance(Config* cfg);
void gererModeEconomie(Config* cfg);
void lireCapteurs(DataPoint* dp, bool lireGPS, bool lireLuminositeHumidite = true);
void parserGPGGA(const char* nmea, char* latLonAlt, size_t len);
void enregistrerDonneesSD(DataPoint* dp);
void gererTailleFichierSD();
void reinitialiserConfiguration(Config* cfg);
void lireConfiguration(Config* cfg);
void sauvegarderConfiguration(Config* cfg);
uint16_t compterLignesFichier();

// ====================== SETUP ==================================
void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  setColorLED(0,0,0);
  pinMode(PIN_BOUTON_ROUGE, INPUT_PULLUP);
  pinMode(PIN_BOUTON_VERT, INPUT_PULLUP);
  pinMode(PIN_LUMINOSITE, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_BOUTON_ROUGE), isr_bouton_rouge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BOUTON_VERT), isr_bouton_vert, CHANGE);

  Wire.begin();

  Serial.print(F("Test RTC DS1307... "));
  if (!rtc.begin()) {
    Serial.println(F("ERREUR"));
    rtcDisponible = false;
    etatLedActuel = ERREUR_RTC_RB;
  } else {
    Serial.println(F("OK"));
    if (!rtc.isrunning() || rtc.now().year() < 2023) {
      Serial.println(F("Ajustement RTC..."));
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  }

   if (!bme.begin(0x76)) {
    Serial.println("BME280 non détecté !");
    while (1);
  }

  Serial.print(F("Test carte SD... "));
  if (!SD.begin(PIN_SD_CS)) {
    Serial.println(F("ERREUR"));
    sdDisponible = false;
    if (etatLedActuel == LED_ETEINTE) etatLedActuel = ERREUR_SD_WRITE_RWB;
  } else {
    Serial.println(F("OK"));
    if (SD.exists("datalog.txt")) {
      SD.remove("datalog.txt");
      Serial.println(F("datalog.txt effacé"));
    }
  }

  lireConfiguration(&configSysteme);

  modeActuel = (digitalRead(PIN_BOUTON_ROUGE) == LOW) ? MODE_CONFIGURATION : MODE_STANDARD;

  if (etatLedActuel == LED_ETEINTE)
    mettreAJourEtatLED(modeActuel);

  dernierTempsActiviteConfig = millis();

  // Forcer une mesure immédiate en mode STANDARD
  if (modeActuel == MODE_STANDARD) {
    dernierTempsMesure = millis() - configSysteme.LOG_INTERVAL;
    Serial.println(F("Mode STANDARD: mesure immediate activee"));
  }
}

// ======================== LOOP ==================================
void loop() {
  gererLogiqueBoutons();
  verifierAppuiLong();

  switch (modeActuel) {
    case MODE_STANDARD:      gererModeStandard(&configSysteme); break;
    case MODE_CONFIGURATION: gererModeConfiguration(&configSysteme); break;
    case MODE_MAINTENANCE:   gererModeMaintenance(&configSysteme); break;
    case MODE_ECONOMIQUE:    gererModeEconomie(&configSysteme); break;
  }

  gererAffichageLED();
}

// =================== FONCTIONS ==================================

void setColorLED(byte r, byte g, byte b) {
  rgbLed.setColorRGB(0, r, g, b);
}

void mettreAJourEtatLED(ModeSysteme mode) {
  if (etatLedActuel >= ERREUR_RTC_RB) return;
  EtatLED nouvelEtat = LED_ETEINTE;

  switch (mode) {
    case MODE_STANDARD:       nouvelEtat = LED_VERT_CONTINU; break;
    case MODE_CONFIGURATION:  nouvelEtat = LED_JAUNE_CONTINU; break;
    case MODE_ECONOMIQUE:     nouvelEtat = LED_BLEU_CONTINU; break;
    case MODE_MAINTENANCE:    nouvelEtat = LED_ORANGE_CONTINU; break;
  }

  if (nouvelEtat != etatLedActuel) {
    etatLedActuel = nouvelEtat;
    tempsDernierChangementLed = millis();
    etatBlinkCouleur1 = true;

    if (rtcDisponible && rtc.isrunning()) {
      DateTime now = rtc.now();
      char timestamp[20];
      snprintf(timestamp, sizeof(timestamp), "%04d/%02d/%02d %02d:%02d:%02d",
               now.year(), now.month(), now.day(),
               now.hour(), now.minute(), now.second());
      Serial.print(F("[")); Serial.print(timestamp); Serial.print(F("] "));
    } else {
      Serial.print(F("["));
      Serial.print(millis()/1000);
      Serial.print(F("s] "));
    }

    Serial.print(F("Mode: "));
    switch(mode) {
      case MODE_STANDARD: Serial.println(F("STANDARD")); break;
      case MODE_CONFIGURATION: Serial.println(F("CONFIG")); break;
      case MODE_MAINTENANCE: Serial.println(F("MAINTENANCE")); break;
      case MODE_ECONOMIQUE: Serial.println(F("ECONOMIQUE")); break;
    }
  }
}

void gererAffichageLED() {
  unsigned long t = millis();
  switch (etatLedActuel) {
    case LED_VERT_CONTINU:   setColorLED(0, 255, 0); break;
    case LED_JAUNE_CONTINU:  setColorLED(255, 255, 0); break;
    case LED_BLEU_CONTINU:   setColorLED(0, 0, 255); break;
    case LED_ORANGE_CONTINU: setColorLED(255, 100, 0); break;
    case ERREUR_RTC_RB:
    case ERREUR_SD_WRITE_RWB:
      if (t - tempsDernierChangementLed >= 500) {
        etatBlinkCouleur1 = !etatBlinkCouleur1;
        tempsDernierChangementLed = t;
      }
      setColorLED(etatBlinkCouleur1?255:0,0,etatBlinkCouleur1?0:255);
      break;
    case ERREUR_CAPTEUR_RG:
      // clignotement rouge/vert pour erreur capteur
      if (t - tempsDernierChangementLed >= 500) {
        etatBlinkCouleur1 = !etatBlinkCouleur1;
        tempsDernierChangementLed = t;
      }
      setColorLED(etatBlinkCouleur1?255:0, etatBlinkCouleur1?0:255, 0);
      break;
    default:
      break;
  }
}

void gererLogiqueBoutons() {
  unsigned long t = millis();

  noInterrupts();
  bool rouge = drapeauEvenementBoutonRouge;
  drapeauEvenementBoutonRouge = false;
  interrupts();

  if (rouge && t - tempsDernierChangementRouge > DEBOUNCE_TIME){
    tempsDebutAppuiRouge = (digitalRead(PIN_BOUTON_ROUGE) == LOW) ? t : 0;
    tempsDernierChangementRouge = t;
  }

  noInterrupts();
  bool vert = drapeauEvenementBoutonVert;
  drapeauEvenementBoutonVert = false;
  interrupts();

  if (vert && t - tempsDernierChangementVert > DEBOUNCE_TIME){
    tempsDebutAppuiVert = (digitalRead(PIN_BOUTON_VERT) == LOW) ? t : 0;
    tempsDernierChangementVert = t;
  }
}

void verifierAppuiLong() {
  unsigned long t = millis();

  if (tempsDebutAppuiRouge && t - tempsDebutAppuiRouge > TEMPS_APPUI_LONG){
    if (modeActuel == MODE_STANDARD || modeActuel == MODE_ECONOMIQUE) modeActuel = MODE_MAINTENANCE;
    else if (modeActuel == MODE_MAINTENANCE) modeActuel = MODE_STANDARD;
    mettreAJourEtatLED(modeActuel);
    tempsDebutAppuiRouge = 0;
  }

  if (tempsDebutAppuiVert && t - tempsDebutAppuiVert > TEMPS_APPUI_LONG){
    if (modeActuel == MODE_STANDARD) modeActuel = MODE_ECONOMIQUE;
    mettreAJourEtatLED(modeActuel);
    tempsDebutAppuiVert = 0;
  }
}

void gererModeStandard(Config* cfg){
  if (!sdDisponible) {
    Serial.println(F("MODE STANDARD: SD absente, capteurs desactives"));
    delay(5000);
    return;
  }
  
  unsigned long t = millis();
  if (t - dernierTempsMesure >= cfg->LOG_INTERVAL){
    dernierTempsMesure = t;
    DataPoint dp;
    lireCapteurs(&dp,true,true);
    
    // Afficher les données
    Serial.print(dp.timestamp); Serial.print(F(" | "));
    Serial.print(F("Lum:")); Serial.print(dp.luminosite); Serial.print(F(" | "));
    Serial.print(F("Temp:")); Serial.print(dp.temperature_air); Serial.print(F("C | "));
    Serial.print(F("Hum:")); Serial.print(dp.humidite_air); Serial.print(F("% | "));
    Serial.print(F("Press:")); Serial.print(dp.pression_air); Serial.print(F("hPa | "));
    Serial.print(F("GPS:")); Serial.print(dp.gpsData);
    
    // Enregistrer sur SD
    enregistrerDonneesSD(&dp);
    Serial.println(F(" -> SD OK"));
  }
}

void gererModeConfiguration(Config* cfg){
  if (Serial.available() > 0){
    char cmd[10];
    int i = 0;
    while(Serial.available() && i < 9) {
      cmd[i++] = Serial.read();
      delay(2);
    }
    cmd[i] = 0;
    
    // Retirer \n et \r
    for(int j = 0; j < i; j++) {
      if(cmd[j] == '\n' || cmd[j] == '\r') cmd[j] = 0;
    }
    
    if (strcmp(cmd, "RESET") == 0) { 
      reinitialiserConfiguration(cfg); 
      sauvegarderConfiguration(cfg);
      Serial.println(F("Config reset!"));
    }
    dernierTempsActiviteConfig = millis();
  }
  if (millis() - dernierTempsActiviteConfig >= TIMEOUT_CONFIG){
    modeActuel = MODE_STANDARD;
    mettreAJourEtatLED(modeActuel);
  }
}

void gererModeMaintenance(Config* cfg){
  unsigned long t = millis();
  if (t - dernierTempsMesure >= cfg->LOG_INTERVAL){
    dernierTempsMesure = t;
    DataPoint dp;
    lireCapteurs(&dp,true,true);
    Serial.print(dp.timestamp); Serial.print(F(" | "));
    Serial.print(F("Lum:")); Serial.print(dp.luminosite); Serial.print(F(" | "));
    Serial.print(F("Temp:")); Serial.print(dp.temperature_air); Serial.print(F("C | "));
    Serial.print(F("Hum:")); Serial.print(dp.humidite_air); Serial.print(F("% | "));
    Serial.print(F("Press:")); Serial.print(dp.pression_air); Serial.print(F("hPa | "));
    Serial.print(F("GPS:")); Serial.println(dp.gpsData);
  }
}

void gererModeEconomie(Config* cfg){
  if (!sdDisponible) {
    Serial.println(F("MODE ECO: SD absente, capteurs desactives"));
    delay(5000);
    return;
  }
  
  unsigned long t = millis();
  if (t - dernierTempsMesure >= (unsigned long)cfg->LOG_INTERVAL * 2UL){
    dernierTempsMesure = t;
    compteurMesuresEco++;
    bool lireGPS = (compteurMesuresEco % 2 != 0);
    DataPoint dp;
    lireCapteurs(&dp,lireGPS,false);
    enregistrerDonneesSD(&dp);
  }
}

void parserGPGGA(const char* nmea, char* latLonAlt, size_t len) {
    char buf[64];
    strncpy(buf, nmea, sizeof(buf));
    buf[sizeof(buf)-1] = 0;

    char* token;
    int field = 0;
    char lat[16] = "", ns = ' ';
    char lon[16] = "", ew = ' ';
    char alt[16] = "";

    token = strtok(buf, ",");
    while(token) {
        field++;
        switch(field) {
            case 3: strncpy(lat, token, sizeof(lat)); lat[sizeof(lat)-1]=0; break;
            case 4: ns = token[0]; break;
            case 5: strncpy(lon, token, sizeof(lon)); lon[sizeof(lon)-1]=0; break;
            case 6: ew = token[0]; break;
            case 10: strncpy(alt, token, sizeof(alt)); alt[sizeof(alt)-1]=0; break;
        }
        token = strtok(NULL, ",");
    }
    snprintf(latLonAlt, len, "%s%c,%s%c,%s", lat, ns, lon, ew, alt);
}

// === LECTURE CAPTEURS (BME280) ===
void lireCapteurs(DataPoint* dp, bool lireGPS, bool lireLuminositeHumidite){
    if (rtcDisponible && rtc.isrunning()) {
      DateTime now = rtc.now();
      snprintf(dp->timestamp, sizeof(dp->timestamp), "%04d/%02d/%02d %02d:%02d:%02d",
               now.year(), now.month(), now.day(),
               now.hour(), now.minute(), now.second());
    } else {
      snprintf(dp->timestamp, sizeof(dp->timestamp), "UP:%lu", millis()/1000);
    }

    if (lireLuminositeHumidite){
        dp->luminosite = analogRead(PIN_LUMINOSITE);
    } else {
        dp->luminosite = 0;
    }

    dp->temperature_air = bme.readTemperature();
    dp->humidite_air = bme.readHumidity();
    dp->pression_air = bme.readPressure() / 100.0F;

    if(lireGPS){
        unsigned long t0 = millis();
        dp->gpsData[0] = 0;
        
        while(millis() - t0 < 1000){
            if(gpsSerial.available()){
                char line[70];
                int idx = 0;
                while(gpsSerial.available() && idx < 69) {
                  char c = gpsSerial.read();
                  if(c == '\n') break;
                  line[idx++] = c;
                }
                line[idx] = 0;
                
                if(strncmp(line, "$GPGGA", 6) == 0){
                    parserGPGGA(line, dp->gpsData, sizeof(dp->gpsData));
                    break;
                }
            }
        }
        if(dp->gpsData[0] == 0) strcpy(dp->gpsData,"GPS NA");
    } else strcpy(dp->gpsData,"NA");
}

// *** NOUVEAU : Compte le nombre de lignes dans le fichier ***
uint16_t compterLignesFichier() {
  if (!sdDisponible) return 0;
  
  File f = SD.open("datalog.txt", FILE_READ);
  if (!f) return 0;
  
  uint16_t lignes = 0;
  while (f.available()) {
    if (f.read() == '\n') lignes++;
  }
  f.close();
  
  return lignes;
}

// *** AMÉLIORATION : FIFO ligne par ligne précis ***
void gererTailleFichierSD() {
  if (!sdDisponible) return;
  
  File f = SD.open("datalog.txt", FILE_READ);
  if (!f) return;
  
  unsigned long tailleFichier = f.size();
  f.close();
  
  if (tailleFichier > configSysteme.FILE_MAX_SIZE) {
    Serial.println(F("Nettoyage SD (FIFO)..."));
    
    uint16_t totalLignes = compterLignesFichier();
    uint16_t lignesASupprimer = max(1, totalLignes / 5);
    
    Serial.print(F("Total lignes: ")); Serial.println(totalLignes);
    Serial.print(F("Suppression: ")); Serial.print(lignesASupprimer);
    Serial.println(F(" lignes anciennes"));
    
    File source = SD.open("datalog.txt", FILE_READ);
    File temp = SD.open("temp.txt", FILE_WRITE);
    
    if (source && temp) {
      uint16_t lignesIgnorees = 0;
      while (source.available() && lignesIgnorees < lignesASupprimer) {
        char c = source.read();
        if (c == '\n') lignesIgnorees++;
      }
      
      while (source.available()) {
        temp.write(source.read());
      }
      
      source.close();
      temp.close();
      
      SD.remove("datalog.txt");
      
      File tempRead = SD.open("temp.txt", FILE_READ);
      File newLog = SD.open("datalog.txt", FILE_WRITE);
      
      if (tempRead && newLog) {
        while (tempRead.available()) {
          newLog.write(tempRead.read());
        }
        tempRead.close();
        newLog.close();
      }
      
      SD.remove("temp.txt");
      
      uint16_t nouvelleTaille = compterLignesFichier();
      Serial.print(F("Nouvelles lignes: ")); Serial.println(nouvelleTaille);
      Serial.println(F("Nettoyage termine!"));
    } else {
      Serial.println(F("Erreur ouverture fichiers"));
    }
  }
}

void enregistrerDonneesSD(DataPoint* dp){
  if (!sdDisponible) return;
  
  gererTailleFichierSD();
  
  File f = SD.open("datalog.txt", FILE_WRITE);
  if(f){
    f.print(dp->timestamp); f.print(';');
    f.print(dp->luminosite); f.print(';');
    f.print(dp->temperature_air); f.print(';');
    f.print(dp->humidite_air); f.print(';');
    f.print(dp->pression_air); f.print(';');   // champ pression ajouté
    f.println(dp->gpsData);
    f.close();
  } else {
    Serial.println(F("Erreur SD"));
    sdDisponible = false;
    etatLedActuel = ERREUR_SD_WRITE_RWB;
  }
}

void reinitialiserConfiguration(Config* cfg){
  cfg->LOG_INTERVAL = 10000;
  cfg->FILE_MAX_SIZE = 2048;
  cfg->TIMEOUT_CAPTEUR = 30;
  cfg->LUMIN_LOW = 255;
  cfg->LUMIN_HIGH = 768;
}

void lireConfiguration(Config* cfg){
  EEPROM.get(0, *cfg);
  if(cfg->LOG_INTERVAL == 0 || cfg->LOG_INTERVAL > 3600000UL){
    reinitialiserConfiguration(cfg);
    sauvegarderConfiguration(cfg);
  }
}

void sauvegarderConfiguration(Config* cfg){
  EEPROM.put(0, *cfg);
}
