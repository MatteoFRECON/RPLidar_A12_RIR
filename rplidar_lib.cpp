/*
  rplidar_lib.cpp
  -----------------------------------------------------------------------------
  Implémentation des échanges bas niveau avec le RPLIDAR.

  Le fichier regroupe :
  - l'envoi des commandes du protocole Slamtec ;
  - la lecture des descripteurs de réponse ;
  - la lecture des payloads série ;
  - le décodage des mesures standard de 5 octets en angle, distance et qualité.

  Les ajouts sont uniquement des commentaires destinés à rendre le dépôt GitHub
  compréhensible par une personne extérieure au projet.
*/

#include "esp32-hal.h"

#include "Arduino.h"
#include "rplidar_lib.h"
#include "math.h"



// Constructeur : mémorise le port série du LIDAR et la broche de commande moteur.

rplidar_lib::rplidar_lib(Stream &serial, int motor_pin)

  : serial_(serial) {  // Initialisation du membre 'serial_' avec le flux série passé en argument
  Mpin = motor_pin;    // Initialisation du membre 'Mpin' avec le numéro de broche du moteur
}



// Initialise la partie matérielle du LIDAR puis le remet dans un état connu avec reset().

void rplidar_lib::init() {

  pinMode(Mpin, OUTPUT);  // Configure la broche Mpin (connectée au moteur) comme une sortie

  reset();  // Appelle la fonction 'reset' pour initialiser ou réinitialiser le LIDAR

  delay(50);  // Attend 50 millisecondes pour permettre au système de se stabiliser
              /// attention modiffier de 500 à 50
}



// Applique une commande PWM au moteur du LIDAR.

void rplidar_lib::motor(int power) {
    analogWrite(Mpin, power);  // Applique une valeur de puissance (entre 0 et 255) à la broche Mpin pour contrôler la vitesse du moteur
}



// Décode une trame de mesure standard RPLIDAR.
// Format simplifié de la trame :
// - hit[0] : flags + qualité de retour ;
// - hit[1] et hit[2] : angle en format fixe ;
// - hit[3] et hit[4] : distance en format fixe.

int rplidar_lib::process_scan(const byte hit[5], int& new_scan, int& quality, float& angle, float& distance){

  ///Serial.printf("values : %x %x %x %x %x", hit[0],hit[1],hit[2],hit[3],hit[4]);
  //new_scan = hit[0] & 0b1;  // a regarder avec david ca ne sert a rien
  //int inverse = (hit[0] >> 1) & 0b1;  // a regarder avec david ca ne sert a rien

  quality = hit[0] >> 2; // revoit la quantite de lumiere retourner au capteur du lidar, il correspond a octect 0 moins les 2 bit de flag

  if ((bitRead(hit[0], 0) == bitRead(hit[0], 1))){ // On regarde si le task flag et le flag inverse sont egaux ce qui est une erreur
    Serial.print("Les flag ne sont pas inverce \n");
    return 0;
  }

  if (bitRead(hit[1], 0) != 1){  //regarde si le bit de check n'est pas  egal a 1
    Serial.print("le bit de check est 0 \n");
  }

  angle = (hit[2] << 7) | (hit[1] >> 1); /// Angle brut en bit correspond a l'otect 2 plus l'octect 1 mois le bit de flag
  distance = (hit[4] << 8) | hit[3]; // distance brut en bit correspond a l'octect 4 plus l'octect 3
  distance = distance / 4.0 ;  // la distance reel correspont distance brut en bit diviser par 4. cf.rplidar_interface_protocol_en
  angle = angle / 64.0;   // l'angle reel correspont l'angle brut en bit diviser par 64. cf.rplidar_interface_protocol_en

  return 1;
}



// Lit le descripteur de 7 octets envoyé par le LIDAR avant certaines réponses.

void rplidar_lib::read_descriptor(int& dsize, int& is_single, int& dtype) {

  Serial.printf("available %d \n", serial_.available());  // Affiche le nombre d'octets dispo sur le port série

  byte data[7];  // Buffer pour stocker les 7 octets du descripteur

  auto nb_read = serial_.readBytes(data, 7);  // Lit exactement 7 octets

  if (nb_read != 7) {  // Vérifie qu'on a bien reçu les 7 octets attendus
    Serial.println("Erreur : descripteur incomplet !");
    return;
  }

  // Debug : affiche les valeurs du descripteur en hex (désactivé ici)
  // Serial.printf("values : %x %x %x %x %x %x %x\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6]);

  is_single = data[5];  // Indique si la réponse est unique (1) ou continue (0)
  dsize     = data[2];  // Taille des données qui suivront la réponse
  dtype     = data[6];  // Type de données (permet de savoir comment interpréter la suite)

  // Debug : affiche la taille de la réponse attendue
  // Serial.printf("dsize %d \n", dsize);
}



// Lit dsize octets depuis le port série dans un buffer fourni par l'appelant.

size_t rplidar_lib::read_response(byte* payload, size_t dsize) {
  // Lit dsize octets depuis le port série dans un tableau fourni par l'appelant.
  // Cette version évite les allocations dynamiques répétées.
  if (payload == nullptr || dsize == 0) {
    return 0;
  }

  size_t nb_read = serial_.readBytes(payload, dsize);

  // Debug ponctuel si besoin :
  // Serial.printf("how many read : %zu / %zu\n", nb_read, dsize);

  return nb_read;
}



// Interroge l'état de santé du capteur avant de lancer le scan.

bool rplidar_lib::getheal() {
  // Envoie la commande pour obtenir l'état de santé du LIDAR (commande standard : 0xA5 0x52)
  byte health_cmd[2] = {0xA5, 0x52};
  serial_.write(health_cmd, 2);  // Envoie les 2 octets de la commande via le port série

  // Lit le descripteur de réponse (obtenu après l'envoi de la commande)
  int dsize, is_single, dtype;
  read_descriptor(dsize, is_single, dtype);  // Récupère la taille, le type de réponse et si c'est une réponse unique

  // Lit la réponse complète selon la taille du descripteur.
  // La réponse health standard fait 3 octets, mais on garde un buffer un peu plus large par sécurité.
  byte data[16] = {0};

  if (dsize <= 0 || dsize > (int)sizeof(data)) {
    Serial.println("Erreur : taille de reponse health invalide");
    return false;
  }

  size_t nb_read = read_response(data, (size_t)dsize);
  if (nb_read != (size_t)dsize) {
    Serial.println("Erreur : reponse health incomplete");
    return false;
  }

  // Le premier octet de la réponse correspond à l'état de santé du LIDAR
  int status = data[0];  

  // Affiche l'état de santé du LIDAR en fonction du code reçu
  if (status == 0)
    Serial.print("Good \n");   // Si le code est 0, l'état est "Bon"
  else if (status == 1)
    Serial.print("Warning\n"); // Si le code est 1, l'état est "Avertissement"
  else if (status == 2)
    Serial.print("Error\n");   // Si le code est 2, l'état est "Erreur"

  // Retourne 'true' si l'état est "Good" (0), sinon 'false' pour les autres statuts
  return !status;  // Retourne 'true' si l'état est "Good" (0), sinon 'false'
}



// Lance le scan standard : après cette commande, le LIDAR envoie des mesures en continu.

void rplidar_lib::start_scan() {
  
  int status = getheal();  // Vérifie l'état de santé du LIDAR

  if (status) {  // Si le LIDAR est en bon état (status == 1)
    Serial.print("Here! \n");  // Affiche un message de debug

    // Envoie de la commande de démarrage du scan (0xA5 0x20)
    byte cmd[2] = {0xA5, 0x20};
    serial_.write(cmd, 2);  // Envoie la commande via le port série

    // Lit le descripteur de la réponse (taille, type, mode)
    int dsize, is_single, dtype;
    read_descriptor(dsize, is_single, dtype);
  }
}



// Commande optionnelle de diagnostic : demande du taux d'échantillonnage.

void rplidar_lib::getsamplerate() {
// get sample rate cf. doc rplidar

  serial_.write(0xA5); 
  serial_.write(0x59);
}



// Commande optionnelle de diagnostic : demande de configuration.

void rplidar_lib::getconf() {
  // get confing rate cf. doc rplidar

  serial_.write(0xA5);
  serial_.write(0x84);
  serial_.write(0x04);
  serial_.write(0x70);
  serial_.write((byte)0x00);
  serial_.write((byte)0x00);
  serial_.write((byte)0x00);
}



// Commande optionnelle de diagnostic : informations générales du capteur.

void rplidar_lib::getinfo() {
// get info cf. doc rplidar

  serial_.write(0xA5);
  serial_.write(0x50);
}



// Stoppe le flux de scan continu.

void rplidar_lib::stop() {
// stop le scan cf. doc rplidar

  serial_.write(0xA5);
  serial_.write(0x25);
}



// Envoie un reset logiciel au capteur.

void rplidar_lib::reset() {
// reset cf. doc rplidar

  serial_.write(0xA5);
  serial_.write(0x40);
}



// Vide les octets encore présents dans le buffer série du LIDAR.

void rplidar_lib::flush() {
  
  delay(1000);                           // Attend 1 seconde. Laisse le temps à des données restantes d'arriver sur le port série.
  while (serial_.available() > 0) {     // Tant qu'il y a des données disponibles dans le buffer série...
    serial_.read();                     // ... on les lit (et donc on les vide du buffer).
  }
}

