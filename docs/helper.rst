Helper-functions
=================

The Helper-Modul bietet Hilfsfunktionen for wiederkehrende Aufgaben wie Logging,
Error Handling, Kryptografie and Datei-Operationen.

Overview
---------

.. lis-table::
   :header-rows: 1
   :widths: 30 70

   * - Helper-Modul
     - Description
   * - **Logger**
     - Konfigurierbares Logging-System
   * - **ErrorHandler / ErrorTraceback**
     - Error Handling and Exception-Tracking
   * - **CryptoHelper**
     - Kryptografische functions (Hashing, encryption)
   * - **FileReader / FileWriter**
     - Datei-Operationen (JSON, YAML, etc.)
   * - **FileLock**
     - Datei-Locking for gleichzeitigen Zugriff
   * - **EnvHandler**
     - environment variables-Verwaltung

Logger
------

Konfigu rierbares Logging with ROS2-Integration:

Verwendung
^^^^^^^^^^

.. code-block:: python

   from vyra_base.helper.logger import Logger
   import logging
   
   # Logger create
   logger = Logger.get_logger(__name__)
   
   # Logging
   logger.info("Modul gestartet")
   logger.warning("Warnung: Niedriger Speicher")
   logger.error("Fehler on Laden der Konfiguration")
   logger.debug("Debug-Information")

Konfiguration
^^^^^^^^^^^^^

.. code-block:: python

   # Logger with custom Config
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
   Use you strukturiertes Logging for bessere Auswertbarkeit:
   
   .. code-block:: python
   
      logger.info("Sensor-Wert", extra={
          "sensor_id": "temp_1",
          "value": 23.5,
          "unit": "°C"
      })

ErrorHandler & ErrorTraceback
------------------------------

Error Handling with automaticallyem Traceback:

ErrorTraceback Decorato
^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.helper.error_traceback import ErrorTraceback
   
   @ErrorTraceback.w_check_error_existt
   async def my_function():
       """Fehler are automatically geloggt"""
       # Bei Exception: Traceback is geloggt and Error-Feeder notified
       raise ValueError("Etwas is schiefgelonen")

**Functionality:**

* Automatisches Exception-Logging
* Completer Traceback
* Integration with ErrorFeeder
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

Kryptografische functions for Sicherheit:

Hashing
^^^^^^^

.. code-block:: python

   from vyra_base.helper.crypto_helper import CryptoHelper
   
   # SHA-256 Hash
   hash_value = CryptoHelper.hash_sha256("mein_passwort")
   print(f"Hash: {hash_value}")
   
   # Passwort-Hashing with Salt (recommended)
   hashed_password = CryptoHelper.hash_password("mein_passwort")
   
   # Passwort-Verifikation
   is_valid = CryptoHelper.verify_password("mein_passwort", hashed_password)

encryption
^^^^^^^^^^^^^^^

.. code-block:: python

   # Daten encrypt
   encrypted = CryptoHelper.encrypt("Geheime Nachricht", key="mein_schluessel")
   
   # Daten decrypt
   decrypted = CryptoHelper.decrypt(encrypted, key="mein_schluessel")

.. warning::
   Use you niemals hartcodierte Schlüssel in Produktionscode!
   Use you environment variables or Secrets-Management.

Zufallswerte
^^^^^^^^^^^^

.. code-block:: python

   # Token generieren
   token = CryptoHelper.generate_token(length=32)
   
   # UUID generieren
   unique_id = CryptoHelper.generate_uuid()

FileReader & FileWriter
-----------------------

Datei-Operationen with automaticallyer Format-Erkennung:

FileReader
^^^^^^^^^^

.. code-block:: python

   from vyra_base.helper.file_reader import FileReader
   
   # JSON read
   data = FileReader.read_json("/workspace/config/settings.json")
   
   # YAML read
   config = FileReader.read_yaml("/workspace/config/config.yaml")
   
   # Text read
   content = FileReader.read_text("/workspace/data/info.txt")
   
   # Binär read
   binary_data = FileReader.read_binary("/workspace/data/image.png")

**Unterstützte Formate:**

* JSON (``.json``)
* YAML (``.yaml``, ``.yml``)
* TOML (``.toml``)
* Text (``.txt``, ``.md``, etc.)
* Binär (alle underen)

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

Datei-Locking for gleichzeitigen Zugriff:

.. code-block:: python

   from vyra_base.helper.file_lock import FileLock
   
   lock_file = "/tmp/my_process.lock"
   
   with FileLock(lock_file) as lock:
       if lock.acquired:
           # Exklusiver Zugriff
           with open("/workspace/data/shared_file.txt", "w") as f:
               f.write("Kritische Daten")
       else:
           print("Datei is gesperrt durch underen Prozess")

.. tip::
   FileLock verhindert Race-Conditions at gleichzeitigen Schreibaccessesn.

EnvHandler
----------

environment variables-Verwaltung:

.. code-block:: python

   from vyra_base.helper.env_handler import EnvHandler
   
   # Umgebungsvariable read
   module_name = EnvHandler.get("MODULE_NAME", default="unknown")
   
   # Umgebungsvariable set
   EnvHandler.set("DEBUG_MODE", "true")
   
   # Alle Variablen load (from .env-Datei)
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

* Use you strukturiertes Logging
* Use you passende Log-Levels (DEBUG < INFO < WARNING < ERROR)
* Loggen you wichtige Events (Start, Stop, Fehler)
* Vermeiden you sensible Daten in Logs

❌ **Vermeiden:**

* Excessive Logging in Hochfrequenz-Schleifen
* Persönliche or Sicherheits-relevante Daten
* Logging without Context-information

Error Handling
^^^^^^^^^^^^^^^^

✅ **Empfohlen:**

* Use you ``@ErrorTraceback`` for async functions
* Loggen you Fehler with completem Context
* Use you ErrorFeeder for zentrale Fehlersammlung

Kryptografie
^^^^^^^^^^^^

✅ **Empfohlen:**

* Use you ``hash_password()`` statt einfachem SHA-256
* Speichern you Schlüssel in environment variables
* Use you starke, tofällige Schlüssel

❌ **Vermeiden:**

* Hartcodierte Passwörter or Schlüssel
* Schwache Hashing-Algorithmen (MD5, SHA1)
* Wiederverwendung of Salts

Datei-Operationen
^^^^^^^^^^^^^^^^^

✅ **Empfohlen:**

* Use you FileReader/FileWriter statt direktem I/O
* Use you FileLock at gleichzeitigem Zugriff
* Prüfen you Pfad-Exisenz vor Schreiboperationen

Further Information
-----------------------------

* :doc:`vyra_base.helper` - API-Referenz
* :class:`~vyra_base.helper.logger.Logger` - Logger-class
* :class:`~vyra_base.helper.error_traceback.ErrorTraceback` - Error-Decorato
* :class:`~vyra_base.helper.crypto_helper.CryptoHelper` - Krypto-functions
* :class:`~vyra_base.helper.file_reader.FileReader` - Datei read
* :class:`~vyra_base.helper.file_writer.FileWriter` - Datei schreiben
* :class:`~vyra_base.helper.file_lock.FileLock` - Datei-Locking
* :class:`~vyra_base.helper.env_handler.EnvHandler` - environment variables
