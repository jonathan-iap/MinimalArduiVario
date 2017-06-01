#ifndef Define_h
#define Define_h

// Mémoire EEprom.
#define MEM_VOLUME  10
#define MEM_SENS    11
#define MEM_FALL    12

// Sensibilité - valeurs.
#define MAX_SENS    40   // Seuil maxi déclenchement en monté
#define MIN_SENS    10   // Seuil mini déclenchement en monté
#define MIN_FALL    -30  // Seuil déclenchement en descente

// Constantes pour la fonction "check" - valeurs.
#define DELAY_TRUE  100    // Durée d'activation du bip et led "check".
#define FREQ_TRUE   1500    // Fréquence pour le son du bip "check".
#define DELAY_FALSE 500
#define FREQ_FALSE  800

// Constantes pour la fonction "vario" - valeurs.
#define SOUND_RISE 500  // Bip de monté.
#define SOUND_FALL 400  // Bip de descente.

// Constantes pour la fonction "switchBouton" - valeurs.
#define STEP_SENS 10  //augmentation par pas de x...
#define DEBOUNCE  200

// Tone frequency
#define TONE_LIMIT    1400 // approximately resonance frequency of buzzer
#define TONE_CONFIRM  1100

// Buttons - pins.
#define BTN_UP        A1
#define BTN_DOWN      A2
#define BTN_SELECT    A3

// Leds - pins.
#define LED_GOOD  A0 // Green
#define LED_ERROR 13 // Red

#endif
