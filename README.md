README - Contrôle Thermostat Zigbee
Description
Ce projet implémente un contrôleur de thermostat basé sur une puce ESP32, utilisant le protocole Zigbee pour communiquer avec un thermostat et un relais. L'application permet de surveiller la température, ajuster le setpoint (point de consigne) et contrôler l'état du relais via une interface web accessible sur le réseau local.
Prérequis

Matériel : Une carte ESP32 compatible avec le firmware ESP-ZB.
Logiciel : Environnement de développement ESP-IDF installé.
Réseau Zigbee : Un coordinateur Zigbee (par exemple, un dongle Zigbee connecté à un Raspberry Pi avec Zigbee2MQTT).

Configuration Initiale

Configuration Zigbee :

Avant de lancer le système, le réseau Zigbee doit être configuré via un outil comme Zigbee2MQTT.
Ajoutez le thermostat (adresse : 0xD5AA) et le relais (adresse : 0xB377) au réseau Zigbee.
Effectuez le binding entre l'ESP32 (endpoint 10) et les appareils Zigbee pour permettre la communication (voir les adresses IEEE dans esp_zigbee_chauffage.h).
Une fois le binding effectué, l'ESP32 peut fonctionner en autonomie sans dépendre de Zigbee2MQTT.


Configuration Wi-Fi :

Modifiez les constantes WIFI_SSID et WIFI_PASSWORD dans esp_zigbee_chauffage.h avec vos identifiants Wi-Fi.
Les paramètres IP statiques sont définis (par exemple, 192.168.1.160). Ajustez-les si nécessaire dans le même fichier.


Compilation et Téléversement :

Compilez le projet avec idf.py build.
Téléversez le firmware avec idf.py -p <PORT> flash.
Téléversez le système de fichiers SPIFFS contenant index.html avec idf.py -p <PORT> spiffs upload.



Fonctionnement

Une fois démarré, l'ESP32 se connecte au réseau Wi-Fi et rejoint le réseau Zigbee.
Une interface web est accessible à l'adresse IP statique (par défaut : 192.168.1.160).
Fonctionnalités :
Affichage en temps réel de la température, du setpoint, et de l'état du relais.
Mise à jour du setpoint via l'interface web (bouton "Update Setpoint").
Contrôle automatique du relais basé sur la température et le setpoint (avec hystérésis par défaut à 0.1°C si non configuré).


Les données sont mises à jour toutes les 5 secondes via des requêtes AJAX.

Limitations Actuelles

Les attributs d'hystérésis (0x0101 et 0x0102) ne sont pas encore pris en charge par le thermostat. Cela peut nécessiter une vérification de la documentation du thermostat ou une mise à jour future du code.

Dépannage

Consultez les logs via le moniteur série pour diagnostiquer les erreurs (connexion Wi-Fi, Zigbee, ou serveur web).
Si le binding Zigbee échoue, vérifiez les adresses IEEE et réinitialisez les appareils.

Contribution
Des contributions sont bienvenues pour améliorer la gestion des hystérésis ou ajouter de nouvelles fonctionnalités. Contactez l'auteur pour plus d'informations.

README - Zigbee Thermostat Control
Description
This project implements a thermostat controller based on an ESP32 chip, using the Zigbee protocol to communicate with a thermostat and a relay. The application allows monitoring temperature, adjusting the setpoint (target temperature), and controlling the relay state via a web interface accessible on the local network.
Prerequisites

Hardware : An ESP32 board compatible with the ESP-ZB firmware.
Software : ESP-IDF development environment installed.
Zigbee Network : A Zigbee coordinator (e.g., a Zigbee dongle connected to a Raspberry Pi with Zigbee2MQTT).

Initial Setup

Zigbee Configuration :

Before running the system, the Zigbee network must be configured using a tool like Zigbee2MQTT.
Add the thermostat (address: 0xD5AA) and the relay (address: 0xB377) to the Zigbee network.
Perform the binding between the ESP32 (endpoint 10) and the Zigbee devices to enable communication (see IEEE addresses in esp_zigbee_chauffage.h).
Once the binding is done, the ESP32 can operate autonomously without relying on Zigbee2MQTT.


Wi-Fi Configuration :

Update the WIFI_SSID and WIFI_PASSWORD constants in esp_zigbee_chauffage.h with your Wi-Fi credentials.
Static IP settings are defined (e.g., 192.168.1.160). Adjust them if needed in the same file.


Compilation and Flashing :

Build the project with idf.py build.
Flash the firmware with idf.py -p <PORT> flash.
Upload the SPIFFS filesystem containing index.html with idf.py -p <PORT> spiffs upload.



Operation

Once started, the ESP32 connects to the Wi-Fi network and joins the Zigbee network.
A web interface is accessible at the static IP address (default: 192.168.1.160).
Features :
Real-time display of temperature, setpoint, and relay state.
Setpoint update via the web interface (using the "Update Setpoint" button).
Automatic relay control based on temperature and setpoint (with a default hysteresis of 0.1°C if not configured).


Data is refreshed every 5 seconds using AJAX requests.

Current Limitations

The hysteresis attributes (0x0101 and 0x0102) are not yet supported by the thermostat. This may require checking the thermostat documentation or a future code update.

Troubleshooting

Check logs via the serial monitor to diagnose issues (Wi-Fi connection, Zigbee, or web server).
If Zigbee binding fails, verify the IEEE addresses and reset the devices.

Contribution
Contributions are welcome to improve hysteresis management or add new features. Contact the author for more information.