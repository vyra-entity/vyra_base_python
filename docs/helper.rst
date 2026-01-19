Helper-Funktionen
=================

Das Helper-Modul bietet Hilfsfunktionen für wiederkehrende Aufgaben wie Logging,
Fehlerbehandlung, Kryptografie und Datei-Operationen.

Übersicht
---------

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Helper-Modul
     - Beschreibung
   * - **Logger**
     - Konfigurierbares Logging-System
   * - **ErrorHandler / ErrorTraceback**
     - Fehlerbehandlung und Exception-Tracking
   * - **CryptoHelper**
     - Kryptografische Funktionen (Hashing, Verschlüsselung)
   * - **FileReader / FileWriter**
     - Datei-Operationen (JSON, YAML, etc.)
   * - **FileLock**
     - Datei-Locking für gleichzeitigen Zugriff
   * - **EnvHandler**
     - Umgebungsvariablen-Verwaltung

Logger
------

Konfigu rierbares Logging mit ROS2-Integration:

Verwendung
^^^^^^^^^^

.. code-block:: python

   from vyra_base.helper.logger import Logger
   import logging
   
   # Logger erstellen
   logger = Logger.get_logger(__name__)
   
   # Logging
   logger.info("Modul gestartet")
   logger.warning("Warnung: Niedriger Speicher")
   logger.error("Fehler beim Laden der Konfiguration")
   logger.debug("Debug-Information")

Konfiguration
^^^^^^^^^^^^^

.. code-block:: python

   # Logger mit custom Config
   log_config = {
       "level": "INFO",  # DEBUG, INFO, WARNING, ERROR, CRITICAL
       "format": "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
       "handlers": {
           "console": True,
           "file": "/workspace/log/module.log"
       }
   }
   
   Logger.configure(log_config)

.. tip::
   Verwenden Sie strukturiertes Logging für bessere Auswertbarkeit:
   
   .. code-block:: python
   
      logger.info("Sensor-Wert", extra={
          "sensor_id": "temp_1",
          "value": 23.5,
          "unit": "°C"
      })

ErrorHandler & ErrorTraceback
------------------------------

Fehlerbehandlung mit automatischem Traceback:

ErrorTraceback Decorator
^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.helper.error_traceback import ErrorTraceback
   
   @ErrorTraceback.w_check_error_exist
   async def my_function():
       """Fehler werden automatisch geloggt"""
       # Bei Exception: Traceback wird geloggt und Error-Feeder benachrichtigt
       raise ValueError("Etwas ist schiefgelaufen")

**Funktionalität:**

* Automatisches Exception-Logging
* Vollständiger Traceback
* Integration mit ErrorFeeder
* Keine Unterbrechung des Programmflusses

ErrorHandler
^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.helper.error_handler import ErrorHandler
   
   try:
       # Riskante Operation
       result = await dangerous_operation()
   except Exception as e:
       # Fehler behandeln
       ErrorHandler.handle_exception(e, context={
           "function": "dangerous_operation",
           "module": "my_module"
       })

CryptoHelper
------------

Kryptografische Funktionen für Sicherheit:

Hashing
^^^^^^^

.. code-block:: python

   from vyra_base.helper.crypto_helper import CryptoHelper
   
   # SHA-256 Hash
   hash_value = CryptoHelper.hash_sha256("mein_passwort")
   print(f"Hash: {hash_value}")
   
   # Passwort-Hashing mit Salt (empfohlen)
   hashed_password = CryptoHelper.hash_password("mein_passwort")
   
   # Passwort-Verifikation
   is_valid = CryptoHelper.verify_password("mein_passwort", hashed_password)

Verschlüsselung
^^^^^^^^^^^^^^^

.. code-block:: python

   # Daten verschlüsseln
   encrypted = CryptoHelper.encrypt("Geheime Nachricht", key="mein_schluessel")
   
   # Daten entschlüsseln
   decrypted = CryptoHelper.decrypt(encrypted, key="mein_schluessel")

.. warning::
   Verwenden Sie niemals hartcodierte Schlüssel in Produktionscode!
   Nutzen Sie Umgebungsvariablen oder Secrets-Management.

Zufallswerte
^^^^^^^^^^^^

.. code-block:: python

   # Token generieren
   token = CryptoHelper.generate_token(length=32)
   
   # UUID generieren
   unique_id = CryptoHelper.generate_uuid()

FileReader & FileWriter
-----------------------

Datei-Operationen mit automatischer Format-Erkennung:

FileReader
^^^^^^^^^^

.. code-block:: python

   from vyra_base.helper.file_reader import FileReader
   
   # JSON lesen
   data = FileReader.read_json("/workspace/config/settings.json")
   
   # YAML lesen
   config = FileReader.read_yaml("/workspace/config/config.yaml")
   
   # Text lesen
   content = FileReader.read_text("/workspace/data/info.txt")
   
   # Binär lesen
   binary_data = FileReader.read_binary("/workspace/data/image.png")

**Unterstützte Formate:**

* JSON (``.json``)
* YAML (``.yaml``, ``.yml``)
* TOML (``.toml``)
* Text (``.txt``, ``.md``, etc.)
* Binär (alle anderen)

FileWriter
^^^^^^^^^^

.. code-block:: python

   from vyra_base.helper.file_writer import FileWriter
   
   # JSON schreiben
   FileWriter.write_json(
       "/workspace/config/output.json",
       {"key": "value", "number": 42}
   )
   
   # YAML schreiben
   FileWriter.write_yaml(
       "/workspace/config/output.yaml",
       {"settings": {"debug": True}}
   )
   
   # Text schreiben
   FileWriter.write_text(
       "/workspace/log/output.txt",
       "Log-Nachricht"
   )

FileLock
--------

Datei-Locking für gleichzeitigen Zugriff:

.. code-block:: python

   from vyra_base.helper.file_lock import FileLock
   
   lock_file = "/tmp/my_process.lock"
   
   with FileLock(lock_file) as lock:
       if lock.acquired:
           # Exklusiver Zugriff
           with open("/workspace/data/shared_file.txt", "w") as f:
               f.write("Kritische Daten")
       else:
           print("Datei ist gesperrt durch anderen Prozess")

.. tip::
   FileLock verhindert Race-Conditions bei gleichzeitigen Schreibzugriffen.

EnvHandler
----------

Umgebungsvariablen-Verwaltung:

.. code-block:: python

   from vyra_base.helper.env_handler import EnvHandler
   
   # Umgebungsvariable lesen
   module_name = EnvHandler.get("MODULE_NAME", default="unknown")
   
   # Umgebungsvariable setzen
   EnvHandler.set("DEBUG_MODE", "true")
   
   # Alle Variablen laden (aus .env-Datei)
   EnvHandler.load_dotenv("/workspace/.env")
   
   # Boolean-Wert parsen
   is_debug = EnvHandler.get_bool("DEBUG_MODE", default=False)
   
   # Integer-Wert parsen
   port = EnvHandler.get_int("PORT", default=8080)

**.env-Datei Beispiel:**

.. code-block:: text

   MODULE_NAME=v2_modulemanager
   DEBUG_MODE=true
   LOG_LEVEL=INFO
   REDIS_HOST=redis
   REDIS_PORT=6379

Best Practices
--------------

Logging
^^^^^^^

✅ **Empfohlen:**

* Verwenden Sie strukturiertes Logging
* Nutzen Sie passende Log-Levels (DEBUG < INFO < WARNING < ERROR)
* Loggen Sie wichtige Events (Start, Stop, Fehler)
* Vermeiden Sie sensible Daten in Logs

❌ **Vermeiden:**

* Excessive Logging in Hochfrequenz-Schleifen
* Persönliche oder Sicherheits-relevante Daten
* Logging ohne Context-Informationen

Fehlerbehandlung
^^^^^^^^^^^^^^^^

✅ **Empfohlen:**

* Verwenden Sie ``@ErrorTraceback`` für async Funktionen
* Loggen Sie Fehler mit vollständigem Context
* Nutzen Sie ErrorFeeder für zentrale Fehlersammlung

Kryptografie
^^^^^^^^^^^^

✅ **Empfohlen:**

* Verwenden Sie ``hash_password()`` statt einfachem SHA-256
* Speichern Sie Schlüssel in Umgebungsvariablen
* Nutzen Sie starke, zufällige Schlüssel

❌ **Vermeiden:**

* Hartcodierte Passwörter oder Schlüssel
* Schwache Hashing-Algorithmen (MD5, SHA1)
* Wiederverwendung von Salts

Datei-Operationen
^^^^^^^^^^^^^^^^^

✅ **Empfohlen:**

* Verwenden Sie FileReader/FileWriter statt direktem I/O
* Nutzen Sie FileLock bei gleichzeitigem Zugriff
* Prüfen Sie Pfad-Existenz vor Schreiboperationen

Weiterführende Informationen
-----------------------------

* :doc:`vyra_base.helper` - API-Referenz
* :class:`~vyra_base.helper.logger.Logger` - Logger-Klasse
* :class:`~vyra_base.helper.error_traceback.ErrorTraceback` - Error-Decorator
* :class:`~vyra_base.helper.crypto_helper.CryptoHelper` - Krypto-Funktionen
* :class:`~vyra_base.helper.file_reader.FileReader` - Datei lesen
* :class:`~vyra_base.helper.file_writer.FileWriter` - Datei schreiben
* :class:`~vyra_base.helper.file_lock.FileLock` - Datei-Locking
* :class:`~vyra_base.helper.env_handler.EnvHandler` - Umgebungsvariablen
