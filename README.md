# Projet RPLIDAR A12 / A2M12 - Détection d'obstacles avec ESP32

Ce dépôt contient un programme Arduino/ESP32 permettant d'utiliser un capteur **SLAMTEC RPLIDAR** pour détecter la présence d'un obstacle autour d'un robot.

Le projet lit les mesures du LIDAR en continu, filtre les points utiles, applique une logique de détection avec hystérésis, puis envoie périodiquement un booléen `0` ou `1` à un ESP32 principal.

> Dans les fichiers du projet, le nom utilisé est `RPLidar_A12_RIR`. La documentation fournie correspond au **RPLIDAR A2M12**, compatible avec le protocole SLAMTEC utilisé ici.

---

## Objectif du projet

L'objectif est de créer un module LIDAR séparé qui :

1. communique avec le RPLIDAR en UART ;
2. récupère les points de mesure angle/distance ;
3. reçoit la position actuelle du robot depuis un ESP32 principal ;
4. vérifie si des points détectés correspondent à un obstacle proche ;
5. envoie à l'ESP32 principal :
   - `0` si aucun obstacle fiable n'est détecté ;
   - `1` si un obstacle fiable est détecté.

Ce fonctionnement permet de décharger l'ESP32 principal : la lecture rapide du LIDAR et le filtrage des points sont faits sur un ESP32 dédié.

---

## Matériel utilisé

- 1 × ESP32 dédié au LIDAR ;
- 1 × RPLIDAR SLAMTEC, type A2/A2M12 ou modèle compatible ;
- 1 × ESP32 principal ou autre carte de contrôle robot ;
- alimentation 5 V stable pour le LIDAR ;
- masses communes entre les cartes ;
- câbles UART pour relier le LIDAR et les deux ESP32.

### Rappel important sur l'alimentation

Le RPLIDAR doit être alimenté avec une alimentation 5 V stable. Une alimentation trop faible ou bruitée peut provoquer des erreurs de communication, des scans instables ou des redémarrages.

Pour le RPLIDAR A2M12, la documentation constructeur indique une alimentation typique de 5 V, une communication UART TTL à 256000 bauds et un courant de démarrage pouvant être élevé. Il est donc conseillé de prévoir une alimentation avec une marge suffisante.

---

## Branchement

### Liaison ESP32 LIDAR ↔ RPLIDAR

| Élément RPLIDAR | Rôle | GPIO ESP32 utilisé dans le code |
|---|---:|---:|
| TX du LIDAR | Données envoyées par le LIDAR | `RX = GPIO16` |
| RX du LIDAR | Commandes envoyées au LIDAR | `TX = GPIO17` |
| GND | Masse commune | GND |
| VCC | Alimentation du LIDAR | 5 V |
| MOTOCTL / moteur | Contrôle moteur par PWM | `GPIO18` |

Dans le code, la communication RPLIDAR utilise :

```cpp
Serial2.begin(256000, SERIAL_8N1, RX, TX);
rplidar_lib lidar(Serial2, 18);
```

### Liaison ESP32 LIDAR ↔ ESP32 principal

| ESP32 LIDAR | Rôle | À relier côté ESP32 principal |
|---|---|---|
| `COM_RX = GPIO25` | Réception des messages venant de l'ESP32 principal | TX de l'ESP32 principal |
| `COM_TX = GPIO26` | Envoi du booléen de détection | RX de l'ESP32 principal |
| GND | Masse commune | GND |

La liaison entre les deux ESP32 fonctionne à :

```cpp
#define LINK_BAUD 115200
Serial1.begin(LINK_BAUD, SERIAL_8N1, COM_RX, COM_TX);
```

---

## Structure du dépôt

```text
.
├── RPLidar_A12_RIR.ino      # Programme principal ESP32
├── rplidar_lib.h            # Déclaration de la classe RPLIDAR
├── rplidar_lib.cpp          # Commandes, lecture et décodage du protocole RPLIDAR
└── README.md                # Documentation du projet
```

---

## Rôle des fichiers

### `RPLidar_A12_RIR.ino`

Fichier principal du projet. Il contient :

- la configuration des ports série ;
- l'initialisation du RPLIDAR ;
- la création des tâches FreeRTOS ;
- la réception de la position robot ;
- la lecture des points LIDAR ;
- le filtrage des mesures ;
- la détection d'obstacle ;
- l'envoi du booléen `0` ou `1` à l'ESP32 principal.

### `rplidar_lib.h`

Fichier d'en-tête de la bibliothèque. Il déclare la classe `rplidar_lib` et ses fonctions principales :

- `init()` ;
- `motor()` ;
- `getheal()` ;
- `start_scan()` ;
- `stop()` ;
- `reset()` ;
- `read_descriptor()` ;
- `read_response()` ;
- `process_scan()`.

### `rplidar_lib.cpp`

Implémentation de la communication avec le RPLIDAR. Ce fichier contient :

- les commandes envoyées au LIDAR ;
- la lecture du descripteur de réponse ;
- la lecture des paquets de mesure ;
- le décodage des trames standard de 5 octets ;
- la conversion de l'angle et de la distance.

---

## Fonctionnement général du programme

Au démarrage, le programme effectue les étapes suivantes :

1. initialise le moniteur série USB avec `Serial` ;
2. initialise la communication avec l'ESP32 principal via `Serial1` ;
3. initialise la communication avec le RPLIDAR via `Serial2` ;
4. réinitialise le LIDAR ;
5. arrête puis relance le moteur ;
6. vérifie l'état de santé du LIDAR avec `getheal()` ;
7. envoie la commande `start_scan()` ;
8. crée une queue FreeRTOS pour stocker les points LIDAR ;
9. lance deux tâches FreeRTOS sur deux cœurs différents.

---

## Organisation FreeRTOS

Le programme utilise deux tâches principales.

### `lidarTask`

Cette tâche est épinglée sur le cœur 1 de l'ESP32.

Son rôle est de lire le plus rapidement possible les mesures du RPLIDAR :

- lecture de paquets de 5 octets ;
- décodage angle/distance/qualité ;
- stockage des points dans une queue FreeRTOS.

Cette tâche a une priorité plus élevée afin de limiter les pertes de mesures.

### `printTask`

Cette tâche est épinglée sur le cœur 0 de l'ESP32.

Son rôle est de gérer la communication et le traitement :

- lecture des messages venant de l'ESP32 principal ;
- récupération de la position du robot ;
- traitement des points présents dans la queue ;
- filtrage des points ;
- calcul de détection ;
- envoi du booléen `0` ou `1`.

---

## Communication série entre les deux ESP32

### Message reçu par l'ESP32 LIDAR

L'ESP32 LIDAR peut recevoir la position du robot sous cette forme :

```text
pos_rob(x;y;theta)
```

Exemple :

```text
pos_rob(1200.00;850.00;90.00)
```

Le code accepte aussi une version avec des virgules :

```text
pos_rob(1200.00,850.00,90.00)
```

Les valeurs sont utilisées de cette façon :

- `x` : position X du robot ;
- `y` : position Y du robot ;
- `theta` : orientation du robot en degrés.

Dans le code actuel, les distances du LIDAR sont en millimètres. Il est donc conseillé d'envoyer aussi `x` et `y` en millimètres pour garder une unité cohérente.

### Message envoyé vers l'ESP32 principal

Toutes les `ENVOI_BOOL_MS`, l'ESP32 LIDAR envoie :

```text
0
```

ou :

```text
1
```

La valeur envoyée correspond à :

| Valeur | Signification |
|---:|---|
| `0` | Aucun obstacle fiable détecté |
| `1` | Obstacle fiable détecté |

Dans cette version :

```cpp
#define ENVOI_BOOL_MS 200
```

Le booléen est donc envoyé toutes les 200 ms.

---

## Logique de détection

Le programme filtre d'abord les points selon la distance :

```cpp
#define DISTANCE_MIN_MM 5
#define DISTANCE_MAX_MM 500
```

Un point est donc traité seulement si sa distance est comprise entre 5 mm et 500 mm.

Ensuite, le programme calcule la position de l'obstacle dans le repère global :

```cpp
float obs_x = (p.distance * cosf(angle_rad)) + local_rob_x;
float obs_y = (-p.distance * sinf(angle_rad)) + local_rob_y;
```

Le point est conservé seulement s'il se situe dans la zone :

```cpp
0 <= x <= 3000
0 <= y <= 2000
```

Cela correspond probablement au terrain ou à la zone utile du robot.

### Zones surveillées

Le code utilise deux zones de détection :

| Zone | Condition actuelle |
|---|---|
| Détection 360° | obstacle à `<= 350 mm` |
| Détection frontale | obstacle à `<= 500 mm` dans un secteur théorique de ±60° |

Les constantes concernées sont :

```cpp
#define DETECTION_DISTANCE_360_MM   350.0f
#define DETECTION_DISTANCE_FRONT_MM 500.0f
#define DETECTION_ANGLE_FRONT_DEG   60.0f
```

---

## Hystérésis de détection

Pour éviter que la sortie passe rapidement de `0` à `1` à cause d'un seul point parasite, le programme utilise une hystérésis simple.

```cpp
#define MIN_POINTS_PASSAGE_A_1 5
```

La logique est la suivante :

| Nombre de points valides sur l'intervalle | État de sortie |
|---:|---|
| 5 points ou plus | passage à `1` |
| 0 point | passage à `0` |
| entre 1 et 4 points | conservation de l'ancien état |

Cela rend la détection plus stable.

---

## Décodage des trames RPLIDAR

Le projet utilise le mode de scan standard du protocole SLAMTEC.

La commande envoyée pour démarrer le scan est :

```text
A5 20
```

Chaque mesure standard est ensuite reçue sous forme d'une trame de 5 octets.

Dans `process_scan()`, le code extrait :

- la qualité de la mesure ;
- le bit de début de nouveau tour ;
- l'angle brut ;
- la distance brute.

Les conversions appliquées sont :

```cpp
distance = distance / 4.0;
angle = angle / 64.0;
```

Cela vient du format standard du protocole RPLIDAR :

- `distance_q2 / 4.0` donne la distance réelle en millimètres ;
- `angle_q6 / 64.0` donne l'angle réel en degrés.

---

## Paramètres importants à modifier

Les principaux paramètres sont regroupés au début de `RPLidar_A12_RIR.ino`.

| Constante | Rôle |
|---|---|
| `PC_BAUD` | Vitesse du moniteur série USB |
| `LINK_BAUD` | Vitesse de communication entre les deux ESP32 |
| `RX` / `TX` | Pins UART2 pour le RPLIDAR |
| `COM_RX` / `COM_TX` | Pins UART1 pour l'ESP32 principal |
| `ENVOI_BOOL_MS` | Période d'envoi du booléen |
| `LIDAR_QUEUE_LEN` | Taille de la queue de points LIDAR |
| `MAX_POINTS_PAR_CYCLE` | Nombre max de points traités par cycle |
| `DISTANCE_MIN_MM` | Distance minimale prise en compte |
| `DISTANCE_MAX_MM` | Distance maximale prise en compte |
| `DETECTION_DISTANCE_360_MM` | Seuil de détection sur 360° |
| `DETECTION_DISTANCE_FRONT_MM` | Seuil de détection frontal |
| `DETECTION_ANGLE_FRONT_DEG` | Demi-angle du secteur frontal |
| `MIN_POINTS_PASSAGE_A_1` | Nombre de points nécessaires pour valider un obstacle |

---

## Installation avec l'IDE Arduino

1. Installer l'IDE Arduino.
2. Installer le support des cartes ESP32.
3. Ouvrir le fichier `RPLidar_A12_RIR.ino`.
4. Placer `rplidar_lib.h` et `rplidar_lib.cpp` dans le même dossier que le `.ino`.
5. Sélectionner la bonne carte ESP32.
6. Sélectionner le bon port COM.
7. Compiler puis téléverser.
8. Ouvrir le moniteur série à `115200 bauds`.

---

## Exemple de sortie debug

Lorsque le debug est activé, le moniteur série peut afficher :

```text
detect=1 | points_lus=85 | points_valides=9 | queue_restante=12
```

Signification :

| Champ | Description |
|---|---|
| `detect` | état envoyé à l'ESP32 principal |
| `points_lus` | nombre de points traités pendant l'intervalle |
| `points_valides` | nombre de points qui valident la détection |
| `queue_restante` | nombre de points encore en attente dans la queue |

Pour activer ou désactiver l'affichage debug, envoyer :

```text
test
```

sur la liaison série.

---

## Points d'attention

### Watchdog

Le watchdog de la boucle Arduino est désactivé dans cette version :

```cpp
#define DESACTIVER_WATCHDOG true
```

Ce choix facilite les tests, mais pour une version finale il est préférable de réactiver le watchdog et de vérifier que toutes les tâches ont des délais suffisants.

### Mode de scan utilisé

Le code actuel décode les trames standard de 5 octets du mode `SCAN`.

Le RPLIDAR A2M12 peut utiliser des modes plus rapides, comme les modes express ou ultra-capsuled, mais ils nécessitent un décodage plus complexe. Ce projet ne les implémente pas pour le moment.

### Secteur frontal

Le commentaire du code indique une normalisation de l'angle dans `[-180°, +180°]`. Dans l'état actuel, la comparaison est faite directement avec :

```cpp
float angle_norm = p.angle + angle_lidar;
```

Si le comportement frontal n'est pas correct autour de 360°/0°, il faudra ajouter une vraie normalisation angulaire.

### Champ `new_scan`

La structure `LidarPoint` contient un champ `new_scan`, mais celui-ci n'est pas utilisé dans le traitement actuel. Il pourra servir plus tard pour compter les tours complets ou estimer la vitesse de rotation.

---

## Problèmes possibles

### Le LIDAR ne répond pas

Vérifier :

- le croisement TX/RX ;
- la masse commune ;
- le baudrate `256000` ;
- l'alimentation 5 V ;
- le câblage du contrôle moteur ;
- le fait que le moteur tourne correctement.

### Message `descripteur incomplet`

Causes possibles :

- mauvais baudrate ;
- LIDAR pas encore prêt ;
- alimentation instable ;
- mauvais câblage UART ;
- données parasites restantes dans le buffer.

### La détection oscille trop

Ajuster :

```cpp
#define MIN_POINTS_PASSAGE_A_1 5
#define DETECTION_DISTANCE_360_MM   350.0f
#define DETECTION_DISTANCE_FRONT_MM 500.0f
```

Augmenter `MIN_POINTS_PASSAGE_A_1` rend la détection plus stricte.

### La queue se remplit trop

Si `queue_restante` reste élevé, il faut réduire le nombre d'affichages série ou augmenter :

```cpp
#define MAX_POINTS_PAR_CYCLE 120
#define LIDAR_QUEUE_LEN 1024
```

---

## Améliorations possibles

- Ajouter une vraie normalisation d'angle dans `[-180°, +180°]`.
- Réactiver proprement le watchdog pour une version finale.
- Ajouter une commande série pour modifier les seuils sans recompiler.
- Envoyer non seulement `0/1`, mais aussi la position estimée de l'obstacle.
- Ajouter un mode debug désactivé par défaut.
- Implémenter le mode `EXPRESS_SCAN` pour exploiter davantage les performances du RPLIDAR A2M12.
- Ajouter un schéma de câblage dans le dépôt.

---

## Références utiles

- Documentation protocole SLAMTEC RPLIDAR : `LR001_SLAMTEC_rplidar_protocol_v2.1_en.pdf`
- Datasheet RPLIDAR A2M12 : `pj2-ld310-slamtec-rplidar-datasheet-a2m12-v1-0-en-2874 (1).pdf`
- SDK officiel SLAMTEC : <https://github.com/Slamtec/rplidar_sdk>

---

## Licence

Licence à compléter selon l'utilisation du projet.

Exemple possible : MIT, GPL, ou dépôt privé sans licence publique.
