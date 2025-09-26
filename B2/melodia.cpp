#include "Melodia.h"

Melodia::Melodia(int pin) {
  outputPin = pin;
}

void Melodia::playSound(int* melody, int* durations, int size) {
  for (int i = 0; i < size; i++) {
    int noteDuration = 1000 / durations[i];
    tone(outputPin, melody[i], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.20; // Pausa del 20% per renderlo più fluido
    delay(pauseBetweenNotes);
    noTone(outputPin);
  }
}

// Suoni brevi
void Melodia::playHappySound() {
  playSound(melodyHappy, noteDurationsHappy, happySize);
}

void Melodia::playAngerSound() {
  playSound(melodyAnger, noteDurationsAnger, angerSize);
}

void Melodia::playSadSound() {
  playSound(melodySad, noteDurationsSad, sadSize);
}

// Versi lunghi "Gibberlink"
void Melodia::playHappyVerse() {
  playSound(verseHappy, verseDurationsHappy, verseHappySize);
}

void Melodia::playAngerVerse() {
  playSound(verseAnger, verseDurationsAnger, verseAngerSize);
}

void Melodia::playSadVerse() {
  playSound(verseSad, verseDurationsSad, verseSadSize);
}
