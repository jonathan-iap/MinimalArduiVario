#include <Arduino.h>
#include "define.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BMP085_U.h" // fonctionne pour le BMP180
#include "Wire.h"
#include "EEPROM.h"
// -Débug LCD i2C-
//#include <LiquidCrystal_I2C.h> // http://arduino-info.wikispaces.com/LCD-Blue-I2C
//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // adresse pour expender chinois type "LCM1602 IIC"



bool g_falling;

int g_volume, g_sensibility;

// Capteur de pression.
Adafruit_BMP085_Unified BMP180;



void initialisation();
void initEeprom();
void readEeprom();
void Led(int ledPin, bool state);
void check(bool valid, int delayTime, int freqTone);
void CtrlError(bool state);
float filterSlowValue(float value);
void vario(bool init);
void bipSound(float p_toneFreq, int p_ddsAcc);
void potar_value(byte value);
void switchBouton();


/*****************************************************************************/
/*----( SETUP )----*/
/*****************************************************************************/
void setup(void)
{
  /* Lecture des anciens réglages, déclaration des entrée/sortie et des
  fonctions "begin()" */
  initialisation();

  // Vérifie que le capteur de pression est bien reconnue.
  CtrlError(BMP180.begin());

  // Première lecture de la valeur de pression qui raccourci la mise en service.
  vario(true);
}

/*****************************************************************************/
/*----( LOOP )----*/
/*****************************************************************************/
void loop(void)
{
  // Fonction principal.
  vario(false);

  // Traite l'infortion relative à l'appui des boutons.
  switchBouton();

  // Delai necessaire à la mesure de la pression.
  delay(20);
}






/*****************************************************************************/
/*----( FONCTIONS )----*/
/*****************************************************************************/

void initialisation()
{
  // Lecture des anciens réglages.
  initEeprom();

  // Ecriture de la valeur g_volume pour le potentiometre i2c.
  potar_value(g_volume);

  // Activation de la résistance interne de pull-up pour l'appuie bouton.
  pinMode(SELECT, INPUT_PULLUP);
  pinMode(UP, INPUT_PULLUP);
  pinMode(DOWN, INPUT_PULLUP);

  pinMode(LED_ERROR, OUTPUT);
  pinMode(LED, OUTPUT);

  // -Débug port Serie-
  //Serial.begin(115200);

  // -Débug LCD i2C-
  //lcd.begin(16, 2);

  // Initialisation du capteur de pression.
  BMP180.begin(BMP085_MODE_STANDARD);

}




void initEeprom()
{
  /*
  Si l'on téléverse le code pour la première fois il ce peut que la mémoire
  eeprom soit rempli avec des valeurs aléatoires supérieurs à celle que l'on utilise,
  si c'est le cas on initialise avec des valeurs par défaut
  */

  //Lecture des anciens réglages
  readEeprom();

  //Initialisation de la memoire eeprom si les valeurs sont corrompus.
  if (g_volume > MAX_VOLUME || g_sensibility > MAX_SENS) {
    EEPROM.write(MEM_VOLUME, MIN_VOLUME );
    EEPROM.write(MEM_SENS, MIN_SENS);
    EEPROM.write(MEM_FALL, false);
    // Et on relie la mémoire.
    readEeprom();
  }
}




void readEeprom()
{
  g_volume = EEPROM.read(MEM_VOLUME);
  g_sensibility = EEPROM.read(MEM_SENS);
  g_falling = EEPROM.read(MEM_FALL);
}





void Led(int ledPin, bool state)
{
  // Fonction visant à simplifier l'utilisation des leds dans le code.
  if (state == HIGH) {
    digitalWrite(ledPin, HIGH);
  }
  else {
    digitalWrite(ledPin, LOW);
  }
}





void check(bool valid, int delayTime, int freqTone)
{
  /*
  Fonction pour valider des actions :
  - 1 bip de frequence réglable
  - led rouge ou led verte s'allume puis s'éteint.
  */

  // Si erreur:
  if (valid == false) {
    tone(BUZZER, freqTone);
    Led(LED_ERROR, HIGH);
    delay(delayTime);
    noTone(BUZZER);
    Led(LED_ERROR, LOW);
    delay(delayTime);
  }
  // Si pas d'erreur:
  else {
    tone(BUZZER, freqTone);
    Led(LED, HIGH);
    delay(delayTime);
    noTone(BUZZER);
    Led(LED, LOW);
    delay(delayTime);
  }
}





void CtrlError(bool state)
{
  // Erreur
  if (!state) {
    while (1) check(false, 500, 1000);
  }
  // OK
  else {
    check(true, 100, 800);
    check(true, 100, 800);
  }
}





float filterSlowValue(float value)
{
  /* Filtre de Kalman */

  const float Q = 1e-7;       // Bruit du filtre.
  const float R = 0.00001;    // Bruit du capteur.

  static float P = 1.0;       // permet une initialisation rapide du filtre
  static float Pc, K, Xp, Xe, Zp;

  Pc = P + Q;
  K = Pc / (Pc + R);
  P = (1 - K) * Pc;
  Xp = Xe;
  Zp = Xp;
  Xe = K * (value - Zp) + Xp;

  return Xe;
}





void vario(bool init)
{
  /* Lecture et calcul des variations de pression/température */

  static int ddsAcc;
  static float toneFreq, toneFreqLowpass, lowPassFast, lowPassSlow, pressure ;

  // Premiere utilisation.
  if (init) {
    Led(LED_ERROR, HIGH);
    // On effectue plusieurs filtrage premier filtrage pour étalonner les filtres.
    for (int i = 0; i < 10; i++) {
      BMP180.getPressure(&pressure);
      lowPassFast = lowPassFast + (pressure - lowPassFast) * 0.1;
      lowPassSlow = filterSlowValue(pressure);
    }
    Led(LED_ERROR, LOW);
  }
  else {
    // Lecture de la valeur de pression du capteur BMP180.
    BMP180.getPressure(&pressure);

    // Filtrage de la valeur sur deux niveaux.
    lowPassFast = lowPassFast + (pressure - lowPassFast) * 0.1; // Faible niveau de filtrage, reponse rapide.
    lowPassSlow = filterSlowValue(pressure); // Niveau de filtrage plus élevé, valeur en décalage de lowPassFast.

    // On fait la différence des deux valeurs et on lui donne plus de poid.
    toneFreq = (lowPassSlow - lowPassFast) * 500;

    // Filtrage de la nouvelle valeur.
    toneFreqLowpass = toneFreqLowpass + (toneFreq - toneFreqLowpass) * 0.1;

    // La valeur est contrainte sur une plage en cas de forte variation.
    toneFreq = constrain(toneFreqLowpass, -500, 500);

    // "ddsAcc" donne une valeur qui permetra d'interrompre le son quelque instant pour faire un bip-bip-bip....
    ddsAcc += toneFreq * 100 + 2000;

    bipSound(toneFreq, ddsAcc);

  }

  // -Débug port Serie-
  /*
  Serial.print("Sensibilite : ");
  Serial.println(g_sensibility);
  Serial.print("Descente : ");
  Serial.println(g_falling);
  Serial.print("g_volume : ");
  Serial.println(g_volume);
  Serial.print("variable toneFreq : ");
  Serial.println(toneFreq);
  Serial.println("");
  Serial.println("");
  */

  // -Débug LCD i2C-
  /*
  lcd.setCursor(0, 0);
  lcd.print("S ");
  lcd.print(g_sensibility);
  lcd.print(" ");
  lcd.setCursor(5, 0);
  lcd.print("D ");
  lcd.print(g_falling);
  lcd.print(" ");
  lcd.setCursor(10, 0);
  lcd.print("V ");
  lcd.print(g_volume);
  lcd.print(" ");
  lcd.setCursor(0, 1);
  lcd.print(toneFreq);
  lcd.print(" ");
  lcd.setCursor(8, 1);
  lcd.print(ddsAcc);
  lcd.print(" ");
  */
}





void bipSound(float p_toneFreq, int p_ddsAcc)
{
  /* Fonction qui gère le son de monté ou de descente. */

  // Si on descend ou si on reste à une même altitude.
  if (p_toneFreq < g_sensibility || p_ddsAcc > 0)
  {
    // Lorsque "g_falling" et activé Bip de descente si on dépasse le seuil de descente "MIN_FALL".
    if (g_falling == true && p_toneFreq < MIN_FALL) {
      tone(BUZZER, p_toneFreq + SOUND_FALL);
      Led(LED, LOW);
      Led(LED_ERROR, HIGH);
    }
    // Sinon Bip de monté
    else if (g_falling == true && p_toneFreq > g_sensibility) {
      tone(BUZZER, p_toneFreq + SOUND_RISE);
      Led(LED, HIGH);
      Led(LED_ERROR, LOW);
    }
    // et si "g_g_falling" est désactivée ou si on ne bouge pas, pas de son.
    else {
      noTone(BUZZER);
      Led(LED, LOW);
      Led(LED_ERROR, LOW);
    }
  }
  #define test
  // Sinon si l'on monte.
  else
  {
    // Descente activé.
    if (g_falling == true) {
      noTone(BUZZER);
      Led(LED, LOW);
    }
    // Descente désactivé bip de monté.
    else {
      tone(BUZZER, p_toneFreq + SOUND_RISE);
      Led(LED, HIGH);
    }
  }
}





void potar_value(byte value)
{
  // Transmet la valeur pour de resistance de potentiomètre.
  Wire.beginTransmission(MCP4017);
  Wire.write(0);
  Wire.write(value);
  Wire.endTransmission();
}





void switchBouton()
{
  /*Fonction pour la Gestion des boutons. */

  // Bouton "Select".
  if (digitalRead(SELECT) == LOW) {
    delay(500);
    // Si le bouton "Select" reste appuyé on règle la sensibilité avec l'aide d'un deuxieme bouton.
    if (digitalRead(SELECT) == LOW) {
      while (digitalRead(SELECT) == LOW ) {
        // Bouton "Select" + "Up".
        if (digitalRead(UP) == LOW) {
          if (g_sensibility < MAX_SENS) {
            g_sensibility += STEP_SENS;
            check(true, DELAY_TRUE, FREQ_TRUE);
            EEPROM.write(MEM_SENS, g_sensibility);
            delay(200);
          }
          else {
            check(false, DELAY_FALSE, FREQ_FALSE); // Indique le réglage maximun.
          }
        }
        // Bouton "Select" + "Down" -> sensibilité -.
        if (digitalRead(DOWN) == LOW) {
          if (g_sensibility > MIN_SENS) {
            g_sensibility -= STEP_SENS;
            check(true, DELAY_TRUE, FREQ_TRUE);
            EEPROM.write(MEM_SENS, g_sensibility);
            delay(200);
          }
          else {
            check(false, DELAY_FALSE, FREQ_FALSE);
          }
        }
      }
    }
    // Bouton "Select" 1 fois -> Activation ou déscativation de la déscente.
    else {
      g_falling = ! g_falling;
      check(true, DELAY_TRUE, FREQ_TRUE);
      EEPROM.write(MEM_FALL, g_falling);
      delay(200);
    }
  }

  // Bouton "Up" -> g_volume +.
  if (digitalRead(UP) == LOW) {
    if (g_volume < MAX_VOLUME) {
      g_volume += STEP_VOL;
      potar_value(g_volume);
      check(true, DELAY_TRUE, FREQ_TRUE);
      EEPROM.write(MEM_VOLUME, g_volume);
      delay(200);
    }
    else {
      check(false, DELAY_FALSE, FREQ_FALSE);
    }
  }

  // Bouton "Down" -> g_volume -.
  if (digitalRead(DOWN) == LOW) {
    if (g_volume > MIN_VOLUME) {
      g_volume -= STEP_VOL;
      potar_value(g_volume);
      check(true, DELAY_TRUE, FREQ_TRUE);
      EEPROM.write(MEM_VOLUME, g_volume);
      delay(200);
    }
    else {
      check(false, DELAY_FALSE, FREQ_FALSE);
    }
  }
}
