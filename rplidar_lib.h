/*
  rplidar_lib.h
  -----------------------------------------------------------------------------
  Déclaration de la classe utilisée pour piloter un RPLIDAR depuis un ESP32/Arduino.

  Ce fichier ne contient pas la logique détaillée : il décrit seulement les fonctions
  disponibles pour le programme principal. L'implémentation se trouve dans
  rplidar_lib.cpp.

  Principe général :
  - un port série Arduino Stream est utilisé pour échanger les octets avec le LIDAR ;
  - une broche PWM commande le moteur du capteur ;
  - les fonctions publiques permettent d'initialiser, commander, lire et décoder le LIDAR.
*/

#ifndef rplidar_lib_h
#define rplidar_lib_h

#include "Arduino.h"

// La classe utilise Stream afin de rester indépendante du port série choisi.
// Dans ce projet, le programme principal lui passe Serial2.

// Définition de la classe rplidar_lib pour interagir avec un capteur RPLIDAR
class rplidar_lib {
public:
  // Constructeur : initialise avec un port série et une broche moteur
  rplidar_lib(Stream& serial, int motor_pin);

  // --- Commandes principales ---
  bool getheal();                     // Vérifie l'état de santé du LIDAR
  void init();                        // Initialise le LIDAR (broche moteur + reset)
  void motor(int power);             // Contrôle la puissance (PWM) du moteur
  void start_scan();                 // Lance un scan continu
  void stop();                       // Arrête le scan
  void reset();                      // Réinitialise le LIDAR

  // --- Communication série / lecture ---
  void flush();                      // Vide les donnéss en stock (supprime les anciennes données)
  void read_descriptor(int& dsize, int& is_single, int& dtype); // Lit le descripteur de réponse
  size_t read_response(byte* payload, size_t dsize); // Lit les données de réponse dans un tableau fourni

  // --- Fonctions supplémentaires / avancées ---
  void getsamplerate();            // Récupère le taux d'échantillonnage du LIDAR
  void getconf();                  // Récupère la configuration du LIDAR
  void getinfo();                  // Récupère les infos générales (version, modèle...)
  
// Une mesure standard de scan RPLIDAR est codée sur 5 octets.
// Cette méthode transforme la trame brute en valeurs physiques exploitables.

  // Traite une mesure : extrait qualité, angle et distance depuis une trame brute de 5 octets
  int process_scan(const byte hit[5], int& new_scan, int& quality, float& angle, float& distance);

private:
  int Mpin;            // Broche connectée au moteur du LIDAR
  Stream& serial_;     // Référence au port série utilisé pour la communication
};

#endif
