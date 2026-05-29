// doc rplidar https://bucket-download.slamtec.com/6fad02c42af6da33f89fbc043c5f165e2b222e0d/rplidar_interface_protocol_en.pdf

#include <Wire.h>
#include "esp_task_wdt.h"
#include "rplidar_lib.h"

// Définition des pins UART2 utilisées pour communiquer avec le LIDAR
#define RX 16 // etant le pin TX du lidar
#define TX 17 // étant le pin RX du lidar

// Définition des pins UART1 utilisées pour communiquer avec l'ESP32 principal
#define COM_RX 25 // etant le pin TX 17 de l'esp32 principale
#define COM_TX 26 // étant le pin RX 16 de l'esp32 principale

// Vitesse de communication avec le PC via USB / UART0
#define PC_BAUD 115200

// Vitesse de communication entre l'ESP32 LIDAR et l'ESP32 principal
#define LINK_BAUD 115200

// Choix des cœurs ESP32
// Core 1 : lecture rapide du LIDAR
// Core 0 : communication Serial1 + traitement/envoi vers l'ESP32 principal
// Attention : le WiFi/Bluetooth utilise souvent le core 0.
// Donc la tâche du core 0 contient toujours un vTaskDelay().
#define CORE_LIDAR 1
#define CORE_COMMUNICATION 0

// Désactivation du watchdog pour éviter les redémarrages pendant les boucles longues.
// Remets à false quand ton programme sera stable.
#define DESACTIVER_WATCHDOG true

// Période d'envoi vers l'ESP32 principal
#define ENVOI_BOOL_MS 200

// Queue de points LIDAR : évite d'écraser les mesures quand elles arrivent vite
#define LIDAR_QUEUE_LEN 1024

// Nombre max de points que printTask traite à chaque passage.
// Augmente cette valeur si le compteur de queue reste élevé.
#define MAX_POINTS_PAR_CYCLE 120

// Filtre de distance utile
#define DISTANCE_MIN_MM 5
#define DISTANCE_MAX_MM 500

// Zones de détection d'obstacle :
// - Sur 360° : détection si distance <= 30 mm
// - Sur le secteur ±60° centré sur 0° (120° au total) : détection si distance <= 50 mm
#define DETECTION_DISTANCE_360_MM   350.0f
#define DETECTION_DISTANCE_FRONT_MM 500.0f
#define DETECTION_ANGLE_FRONT_DEG   60.0f   // demi-angle du secteur frontal (±60° => 120° total)

// Fiabilisation de la detection :
// - passe a 1 seulement a partir de 5 points valides sur l intervalle
// - repasse a 0 seulement si 0 point valide sur l intervalle
// - entre 1 et 4 points, on garde l ancien etat pour eviter les oscillations
#define MIN_POINTS_PASSAGE_A_1 5

// Buffer utilisé pour stocker les caractères reçus depuis Serial1
String bufferRecu = "";

// Sauvegarde du temps du dernier caractère reçu
unsigned long dernierCaractereRecu = 0;

// Temps maximum sans nouveau caractère avant de considérer que le message est terminé
const unsigned long TIMEOUT_MESSAGE = 100; // ms

// Création de l'objet lidar
// Il utilise Serial2 pour communiquer avec le RPLIDAR
// Le pin 18 semble être utilisé pour contrôler le moteur du LIDAR
rplidar_lib lidar(Serial2, 18);

// Structure d'un point LIDAR brut
struct LidarPoint {
  float angle;
  float distance;
  int quality;
  int new_scan;
};

// Queue FreeRTOS entre la tâche de lecture LIDAR et la tâche de traitement.
// Cela permet de conserver beaucoup plus de points au lieu d'écraser la dernière mesure.
QueueHandle_t lidarQueue = NULL;

// Mutex utilisé pour protéger l'accès à la position robot reçue par Serial1
SemaphoreHandle_t dataMutex;

// Décalage angulaire du LIDAR par rapport au robot
float angle_lidar = 0;

// Variable globale pour stocker un angle calculé ou utilisé plus tard
float angle;

bool test = true;

// Position du robot reçue depuis l'ESP32 principal
float rob_x = 300;
float rob_y = 300;
float rob_theta = 0;

// Position calculée d'un obstacle dans le repère du robot ou du terrain
float adv_rob_x = 0;
float adv_rob_y = 0;

// Buffer prévu pour stocker un message venant de l'ESP32 principal
String bufferCentral = "";

// Handles des tâches lancées sur les deux cœurs
TaskHandle_t lidarTaskHandle = NULL;
TaskHandle_t printTaskHandle = NULL;

// Prototypes
void desactiverWatchdogs();
void lire();
void afficherMessageRecu();
void lidarTask(void* pvParameters);
void printTask(void* pvParameters);

void setup()
{
  // Initialisation de l'UART0 pour le moniteur série du PC
  Serial.begin(PC_BAUD);

  // Désactivation volontaire du watchdog avant de lancer les tâches longues
  desactiverWatchdogs();

  // Initialisation de Serial1 pour la communication avec l'ESP32 principal
  Serial1.begin(LINK_BAUD, SERIAL_8N1, COM_RX, COM_TX); // UART1 pins 25/26

  // Initialisation de Serial2 pour la communication avec le RPLIDAR
  Serial2.begin(256000, SERIAL_8N1, RX, TX); // UART2 pins 16/17

  // Initialisation du LIDAR
  lidar.init();

  // Arrêt du moteur du LIDAR
  lidar.motor(0);

  // Nettoyage du buffer série du LIDAR
  lidar.flush();

  // Demande d'état de santé du LIDAR
  lidar.getheal();

  // Démarrage du moteur du LIDAR avec une vitesse donnée
  lidar.motor(150);

  // Nouvelle vérification de l'état de santé du LIDAR
  lidar.getheal();

  // Démarrage du scan du LIDAR
  lidar.start_scan();

  // Création du mutex pour protéger la position robot
  dataMutex = xSemaphoreCreateMutex();

  // Création de la queue LIDAR pour stocker plusieurs points en attente de traitement
  lidarQueue = xQueueCreate(LIDAR_QUEUE_LEN, sizeof(LidarPoint));

  if (lidarQueue == NULL) {
    Serial.println("Erreur : impossible de creer lidarQueue");
    while (true) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }

  // Création de deux tâches FreeRTOS explicitement placées sur deux cœurs.
  // - lidarTask : lecture rapide du LIDAR sur le core 1
  // - printTask : communication Serial1 + traitement/envoi sur le core 0
  xTaskCreatePinnedToCore(
    lidarTask,          // fonction
    "Read Lidar",       // nom
    8192,               // stack size
    NULL,               // paramètres
    2,                  // priorité plus haute pour ne pas rater trop de points
    &lidarTaskHandle,   // handle
    CORE_LIDAR          // core 1
  );

  xTaskCreatePinnedToCore(
    printTask,          // fonction
    "Print Lidar Data", // nom
    8192,               // stack size
    NULL,               // paramètres
    1,                  // priorité
    &printTaskHandle,   // handle
    CORE_COMMUNICATION  // core 0
  );

  Serial.println("Commencement");
  Serial.printf("setup() sur core %d\n", xPortGetCoreID());
}

void desactiverWatchdogs()
{
#if DESACTIVER_WATCHDOG
  // IMPORTANT : sur certaines versions Arduino-ESP32 / ESP-IDF,
  // disableCore0WDT() ou disableCore1WDT() peuvent provoquer un abort()
  // si l'idle task du cœur n'est pas inscrite au watchdog.
  // C'est exactement l'erreur : unsubscribe_idle / esp_task_wdt_delete.
  //
  // On désactive donc seulement le watchdog de la tâche loop Arduino,
  // puis on laisse les tâches FreeRTOS respirer avec vTaskDelay().
  // Cela évite les reboot sans appeler les fonctions qui font planter.
  disableLoopWDT();

  Serial.println("Watchdog loop desactive. Core WDT non force pour eviter abort().");
#endif
}

void lire()
{
  // Envoi PC -> Serial1
  while (Serial.available()) {
    char c = Serial.read();

    Serial1.write(c);

    Serial.print("Envoye : ");
    Serial.println(c);
  }

  // Reception Serial1 -> PC
  while (Serial1.available()) {
    char c = Serial1.read();

    // Mise à jour du temps du dernier caractère reçu
    dernierCaractereRecu = millis();

    // Si on reçoit '\n', on considère que le message est terminé
    if (c == '\n') {
      afficherMessageRecu();
    }
    // On ignore '\r' pour éviter les problèmes avec les retours chariot
    else if (c != '\r') {
      bufferRecu += c;
    }
  }

  // Si on a reçu un message mais pas de \n, on l'affiche après un petit délai
  if (bufferRecu.length() > 0 && millis() - dernierCaractereRecu > TIMEOUT_MESSAGE) {
    afficherMessageRecu();
  }
}

void afficherMessageRecu()
{
  if (bufferRecu.length() > 0) {

    bufferRecu.trim();

    if (bufferRecu.startsWith("test")) {
      test = !test;
      
    }
    //Serial.println(bufferRecu);
    // Si le message commence par pos_rob(...),
    // alors on considère que l'ESP32 principal envoie une nouvelle position robot
    if (bufferRecu.startsWith("pos_rob(")) {
      float x, y, theta;

      // Ton ESP32 principal envoie plutôt : pos_rob(0.00;0.00;0.00)
      // Donc on parse d'abord avec des ';', puis avec des ',' en secours.
      int result = sscanf(bufferRecu.c_str(), "pos_rob(%f;%f;%f)", &x, &y, &theta);

      if (result != 3) {
        result = sscanf(bufferRecu.c_str(), "pos_rob(%f,%f,%f)", &x, &y, &theta);
      }

      if (result == 3) {
        //Serial.printf("rob_x : %.2f, rob_y : %.2f, theta : %.2f\n", x, y, theta);
        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
          rob_x = x;
          rob_y = y;
          rob_theta = theta;
          xSemaphoreGive(dataMutex);
          //Serial.println("rob_x :%.2f, rob_y : %.2f,theta : %.2f",rob_x,rob_y,rob_theta);
        }
      }
      else {
        Serial.println("Erreur format pos_rob");
      }
    }

    bufferRecu = "";
  }
}

void loop()
{
  // Le loop Arduino ne fait plus le travail principal.
  // Les deux vraies tâches sont lidarTask et printTask, chacune épinglée à un cœur.
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void lidarTask(void* pvParameters)
{
  (void)pvParameters;

  Serial.printf("lidarTask lecture LIDAR sur core %d\n", xPortGetCoreID());

  for (;;) {
    // Lecture d'une réponse brute du LIDAR avec un tableau fixe de 5 octets.
    // Une mesure standard RPLIDAR fait 5 octets.
    byte hit[5];
    size_t nb_read = lidar.read_response(hit, 5);

    // Si on n'a pas reçu exactement 5 octets, la trame est incomplète.
    // On ignore cette mesure pour éviter de traiter des anciennes valeurs mémoire.
    if (nb_read != 5) {
      taskYIELD();
      continue;
    }

    // Variables qui seront remplies par process_scan
    int new_scan, quality;
    float angle, distance;

    // Traitement de la trame LIDAR
    // status indique si une mesure valide a été obtenue
    int status = lidar.process_scan(hit, new_scan, quality, angle, distance);

    // Si une mesure LIDAR valide est reçue, on l'ajoute dans la queue.
    // Contrairement à shared_angle/shared_distance, la queue garde plusieurs points.
    if (status) {
      LidarPoint p;
      p.angle = angle;
      p.distance = distance;
      p.quality = quality;
      p.new_scan = new_scan;

      // Envoi non bloquant : si la queue est pleine, on supprime le plus ancien point
      // puis on ajoute le nouveau. Comme ça, on garde les données les plus récentes.
      if (xQueueSend(lidarQueue, &p, 0) != pdTRUE) {
        LidarPoint oldPoint;
        xQueueReceive(lidarQueue, &oldPoint, 0);
        xQueueSend(lidarQueue, &p, 0);
      }
    }

    // Laisse régulièrement la main au scheduler.
    // taskYIELD() seul ne suffit pas toujours à laisser tourner l'idle task,
    // donc on bloque brièvement la tâche de temps en temps pour éviter les watchdog reset.
    static uint32_t compteur_lidar = 0;
    compteur_lidar++;

    if (compteur_lidar >= 50) {
      compteur_lidar = 0;
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    else {
      taskYIELD();
    }
  }
}

void printTask(void* pvParameters)
{
  (void)pvParameters;

  Serial.printf("printTask communication/traitement sur core %d\n", xPortGetCoreID());

  // Etat fiable de detection avec hysteresis :
  // - devient true si au moins MIN_POINTS_PASSAGE_A_1 points valides sont vus
  // - redevient false seulement si 0 point valide est vu
  // - entre 1 et MIN_POINTS_PASSAGE_A_1 - 1 points, on conserve l ancien etat
  bool detection_fiable = false;

  // Compteurs utiles pour vérifier si on traite vraiment plus de points
  uint32_t nb_points_lus_interval = 0;
  uint32_t nb_points_valides_interval = 0;

  // Temps du dernier envoi true/false
  unsigned long dernier_envoi_bool = 0;

  for (;;) {

    // Lecture des messages venant du PC ou de l'ESP32 principal.
    // Cette fonction tourne maintenant sur le core 0.
    lire();

    float local_rob_x = 0;
    float local_rob_y = 0;
    float local_rob_theta = 0;

    // On copie la position robot une seule fois par cycle.
    // C'est plus rapide que de prendre le mutex pour chaque point LIDAR.
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      local_rob_x = rob_x;
      local_rob_y = rob_y;
      local_rob_theta = rob_theta;
      xSemaphoreGive(dataMutex);
    }

    // On traite un paquet de points disponibles dans la queue.
    // Plus MAX_POINTS_PAR_CYCLE est grand, plus on peut absorber de points.
    LidarPoint p;
    int pointsTraitesCeCycle = 0;

    while (pointsTraitesCeCycle < MAX_POINTS_PAR_CYCLE &&
           xQueueReceive(lidarQueue, &p, 0) == pdTRUE) {

      pointsTraitesCeCycle++;
      nb_points_lus_interval++;

      // Filtre sur la distance pour ignorer les valeurs trop proches ou trop loin
      if (p.distance <= DISTANCE_MAX_MM && p.distance >= DISTANCE_MIN_MM) {

        float angle_deg = p.angle + angle_lidar - local_rob_theta;

        // Conversion en radians pour utiliser cos() et sin()
        float angle_rad = angle_deg * PI / 180.0f;

        // Calcul de la position de l'obstacle dans le repère global
        float obs_x = (p.distance * cosf(angle_rad)) + local_rob_x;
        float obs_y = (-p.distance * sinf(angle_rad)) + local_rob_y;

        if (obs_x >= 0 && obs_x <= 3000 && obs_y >= 0 && obs_y <= 2000){

        // --- Zones de détection d'obstacle ---
        // On ramène l'angle dans [-180°, +180°] pour comparer au secteur frontal.
        float angle_norm = p.angle + angle_lidar;

        bool dans_secteur_frontal = (angle_norm <= DETECTION_ANGLE_FRONT_DEG and angle_norm >= -DETECTION_ANGLE_FRONT_DEG);

        // Zone 1 : 360° entier — détection si distance <= 30 mm
        bool detection_360   = (p.distance <= DETECTION_DISTANCE_360_MM);

        // Zone 2 : secteur ±60° centré sur 0° — détection si distance <= 50 mm
        bool detection_front = dans_secteur_frontal && (p.distance <= DETECTION_DISTANCE_FRONT_MM);

        if (detection_360 || detection_front) {
          nb_points_valides_interval++;

          // Ne pas afficher chaque point : Serial.printf est très lent et fait perdre des mesures.
          // Pour debug ponctuel seulement, tu peux réactiver cette ligne.
          //if (test) Serial.printf("angle=%.1f | dist=%.1f | front=%d | 360=%d\n", angle_norm, p.distance, detection_front, detection_360);
          
          //if (test) Serial.printf("rob_adv : x = %.2f | y = %.2f\n", obs_x, obs_y);
        }
        }
        }

    }

    // Toutes les 100 ms, on envoie l'état de détection,
    // même si aucune nouvelle mesure LIDAR n'a été consommée à cet instant.
    if (millis() - dernier_envoi_bool >= ENVOI_BOOL_MS) {
      dernier_envoi_bool = millis();

      // Hysteresis de detection :
      // - 5 points ou plus : on confirme l obstacle => 1
      // - 0 point : on confirme l absence d obstacle => 0
      // - 1 a 4 points : zone incertaine, on garde l ancien etat
      if (nb_points_valides_interval >= MIN_POINTS_PASSAGE_A_1) {
        detection_fiable = true;
      }
      else if (nb_points_valides_interval == 0) {
        detection_fiable = false;
      }

      // Garde ce format si l ESP32 principal attend seulement 0/1
      Serial1.println(detection_fiable ? 1 : 0);

      // Debug cote PC : nombre de points reellement traites pendant l intervalle
      if (test) {
        Serial.printf("detect=%d | points_lus=%lu | points_valides=%lu | queue_restante=%u\n",
                      detection_fiable ? 1 : 0,
                      nb_points_lus_interval,
                      nb_points_valides_interval,
                      uxQueueMessagesWaiting(lidarQueue));
      }

      // On remet les compteurs a zero pour le prochain intervalle
      nb_points_lus_interval = 0;
      nb_points_valides_interval = 0;
    }

    // Pause courte : 1 ms laisse respirer le core 0 tout en traitant beaucoup plus souvent qu'avec 5 ms.
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}
