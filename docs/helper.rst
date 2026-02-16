Helper-functions
=================

The Helper-Modul bietet Hilfsfunktionen for wiederkehrende Aufgaben wie Logging,
Error Handling, Kryptografie and Datei-Operationen.

Overview
---------

.. list-table::
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
     - Datei-Locking for gleichzeitigen Access
   * - **EnvHandler**
     - environment variables-Verwaltung


ErrorHandler & ErrorTraceback
------------------------------

Error Handling with automatic Traceback:

ErrorTraceback Decorator
^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.helper.error_traceback import ErrorTraceback
   
   @ErrorTraceback.w_check_error_existt
   async def my_function():
       """Error are automatically geloggt"""
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
       # Error behandeln
       ErrorHandler.handle_exception(e, context={
           "function": "dangerous_operation",
           "module": "my_module"
       })

CryptoHelper
------------

Cryptographic functions for security:

Hashing
^^^^^^^

.. code-block:: python

   from vyra_base.helper.crypto_helper import CryptoHelper
   
   # SHA-256 Hash
   hash_value = CryptoHelper.hash_sha256("my_password")
   print(f"Hash: {hash_value}")
   
   # Password Hashing with Salt (recommended)
   hashed_password = CryptoHelper.hash_password("my_password")
   
   # Password Verification
   is_valid = CryptoHelper.verify_password("my_password", hashed_password)

encryption
^^^^^^^^^^

.. code-block:: python

   # Data encrypt
   encrypted = CryptoHelper.encrypt("Secret message", key="my_key")
   
   # Data decrypt
   decrypted = CryptoHelper.decrypt(encrypted, key="my_key")

.. warning::
   Never use hardcoded keys in production code!
   Use environment variables or secrets management.

Random Values
^^^^^^^^^^^^^

.. code-block:: python

   # Generate token
   token = CryptoHelper.generate_token(length=32)
   
   # UUID generate
   unique_id = CryptoHelper.generate_uuid()

FileReader & FileWriter
-----------------------

File operations with automatic format detection:

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

**Supported Formats:**

* JSON (``.json``)
* YAML (``.yaml``, ``.yml``)
* TOML (``.toml``)
* Text (``.txt``, ``.md``, etc.)
* Binary (all others)

FileWriter
^^^^^^^^^^

.. code-block:: python

   from vyra_base.helper.file_writer import FileWriter
   
   # JSON write
   FileWriter.write_json(
       "/workspace/config/output.json",
       {"key": "value", "number": 42}
   )
   
   # YAML write
   FileWriter.write_yaml(
       "/workspace/config/output.yaml",
       {"settings": {"debug": True}}
   )
   
   # Text write
   FileWriter.write_text(
       "/workspace/log/output.txt",
       "Log message"
   )

FileLock
--------

File locking for concurrent access:

.. code-block:: python

   from vyra_base.helper.file_lock import FileLock
   
   lock_file = "/tmp/my_process.lock"
   
   with FileLock(lock_file) as lock:
       if lock.acquired:
           # Exclusive access
           with open("/workspace/data/shared_file.txt", "w") as f:
               f.write("Critical data")
       else:
           print("File is locked by another process")

.. tip::
   FileLock prevents race conditions during concurrent write access.

EnvHandler
----------

environment variables management:

.. code-block:: python

   from vyra_base.helper.env_handler import EnvHandler
   
   # Read environment variable
   module_name = EnvHandler.get("MODULE_NAME", default="unknown")
   
   # Set environment variable
   EnvHandler.set("DEBUG_MODE", "true")
   
   # Load all variables (from .env file)
   EnvHandler.load_dotenv("/workspace/.env")
   
   # Parse boolean value
   is_debug = EnvHandler.get_bool("DEBUG_MODE", default=False)
   
   # Parse integer value
   port = EnvHandler.get_int("PORT", default=8080)

**.env file Example:**

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

✅ **Recommended:**

* Use you structured logging
* Use you appropriate log levels (DEBUG < INFO < WARNING < ERROR)
* Log important events (start, stop, error)
* Avoid sensitive data in logs

❌ **Avoid:**

* Excessive logging in high-frequency loops
* Personal or sensitive data
* Logging without context information

Error Handling
^^^^^^^^^^^^^^^^

✅ **Recommended:**

* Use you ``@ErrorTraceback`` for async functions
* Log errors with complete context
* Use you ErrorFeeder for central error collection

Cryptography
^^^^^^^^^^^^

✅ **Recommended:**

* Use you ``hash_password()`` instead of simple SHA-256
* Store your keys in environment variables
* Use you strong, random keys

❌ **Avoid:**

* Hardcoded passwords or keys
* Weak hashing algorithms (MD5, SHA1)
* Reuse of salts

File Operations
^^^^^^^^^^^^^^^
✅ **Recommended:**

* Use you FileReader/FileWriter instead of direct I/O
* Use you FileLock for concurrent access
* Check you path existence before write operations

Further Information
-------------------

* :doc:`vyra_base.helper` - API-Reference
* :class:`~vyra_base.helper.logger.Logger` - Logger class
* :class:`~vyra_base.helper.error_traceback.ErrorTraceback` - Error Decorator
* :class:`~vyra_base.helper.crypto_helper.CryptoHelper` - Crypto functions
* :class:`~vyra_base.helper.file_reader.FileReader` - File read
* :class:`~vyra_base.helper.file_writer.FileWriter` - File write
* :class:`~vyra_base.helper.file_lock.FileLock` - File locking
* :class:`~vyra_base.helper.env_handler.EnvHandler` - Environment variables
