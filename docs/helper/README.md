# Helper Functions - Hilfsfunktionen

Die VYRA Helper-Module bieten nützliche Utility-Funktionen für Logging, Fehlerbehandlung, Datei-I/O und mehr.

## Übersicht

**Hauptmodule:**
- **Logger**: Strukturiertes Logging mit verschiedenen Log-Leveln
- **ErrorTraceback**: Fehlerbehandlung und Stack-Trace-Erfassung
- **FileReader**: Asynchrones Lesen von JSON, YAML, Markdown
- **FileWriter**: Asynchrones Schreiben von JSON, YAML, Binary
- **EnvHandler**: Umgebungsvariablen laden und verwalten
- **CryptoHelper**: Verschlüsselung, Hashing, Token-Generierung

## Inhaltsverzeichnis

1. [Logger](#1-logger)
2. [Error Handling](#2-error-handling)
3. [File Operations](#3-file-operations)
4. [Environment Variables](#4-environment-variables)
5. [Cryptographic Functions](#5-cryptographic-functions)
6. [Utility Functions](#6-utility-functions)

---

## 1. Logger

Zentrales Logging-System für strukturierte Log-Nachrichten.

### Grundlegende Verwendung

```python
from vyra_base.helper.logger import Logger, LogEntry

# Info-Log
Logger.log(LogEntry("Application started"))

# Warning-Log
Logger.log(LogEntry("Configuration missing, using defaults").warn())

# Error-Log
Logger.log(LogEntry("Database connection failed").error())

# Debug-Log
Logger.log(LogEntry("Processing request data").debug())
```

### Logger initialisieren

```python
from vyra_base.helper.logger import Logger
from pathlib import Path

# Logger konfigurieren
Logger.initialize(
    logger_name="my_module",
    log_path="/workspace/log/vyra",
    log_active=True
)

# Mit externer Konfiguration
Logger.initialize_from_config(
    config_path=Path("/workspace/config/logger_config.json")
)
```

### Log-Level

```python
from vyra_base.helper.logger import LogMode, LogEntry

# Explizite Modi
entry = LogEntry("Message", mode=LogMode.DEBUG)
Logger.log(entry)

# Convenience-Methoden
Logger.log(LogEntry("Debug message").debug())
Logger.log(LogEntry("Info message"))  # Standard: INFO
Logger.log(LogEntry("Warning message").warn())
Logger.log(LogEntry("Error message").error())
```

### Strukturierte Logs

```python
# Mit Kontext-Informationen
Logger.log(LogEntry(
    f"User {username} logged in from {ip_address}"
).debug())

# Fehler mit Details
try:
    result = await db.get_by_id(id=999)
except Exception as e:
    Logger.log(LogEntry(
        f"Database query failed: {e}"
    ).error())
```

### Log-Ausgabe

```
2026-01-29 14:30:15 - INFO     - my_module.application - Application started
2026-01-29 14:30:16 - WARNING  - my_module.config - Configuration missing, using defaults
2026-01-29 14:30:17 - ERROR    - my_module.database - Database connection failed
2026-01-29 14:30:18 - DEBUG    - my_module.handler - Processing request data
```

---

## 2. Error Handling

Fehlerbehandlung mit automatischem Stack-Trace-Logging.

### Decorator für Fehlerbehandlung

```python
from vyra_base.helper.error_handler import ErrorTraceback

@ErrorTraceback.w_check_error_exist
async def my_function():
    """Automatische Fehlerbehandlung."""
    # Fehler werden automatisch geloggt
    result = await risky_operation()
    return result

# Auch für synchrone Funktionen
@ErrorTraceback.w_check_error_exist
def sync_function():
    return calculate_result()
```

### Manuelle Fehlerprüfung

```python
from vyra_base.helper.error_handler import ErrorTraceback

error_details = []

try:
    result = await some_operation()
except Exception:
    if ErrorTraceback.check_error_exist(error_details, log_print=True):
        print(f"Fehler aufgetreten: {error_details}")
        # error_details enthält Stack-Trace
```

### Fehler-Kontext

```python
@ErrorTraceback.w_check_error_exist
async def process_user(user_id: int):
    """Verarbeitet Benutzer mit Fehlerbehandlung."""
    try:
        user = await db.get_by_id(id=user_id)
        if not user:
            Logger.log(LogEntry(f"User {user_id} not found").warn())
            return None
        
        # Weitere Verarbeitung
        return await update_user(user)
        
    except Exception as e:
        Logger.log(LogEntry(f"Error processing user {user_id}: {e}").error())
        raise  # Exception wird re-raised mit vollständigem Stack-Trace
```

---

## 3. File Operations

Asynchrone Datei-I/O-Operationen mit Lock-Mechanismus.

### JSON Lesen

```python
from vyra_base.helper.file_reader import FileReader
from pathlib import Path

# JSON-Datei lesen
config = await FileReader.open_json_file(
    config_file=Path("/workspace/config/settings.json")
)
print(f"Timeout: {config['timeout']}")

# Mit Fallback auf Default
config = await FileReader.open_json_file(
    config_file=Path("/workspace/config/settings.json"),
    config_default=Path("/workspace/config/settings.default.json")
)
```

### JSON Schreiben

```python
from vyra_base.helper.file_writer import FileWriter
from pathlib import Path

# JSON-Datei schreiben
config = {
    "timeout": 30,
    "retry_count": 3,
    "enabled": True
}

success = await FileWriter.write_json_file(
    file=Path("/workspace/config/settings.json"),
    file_content=config
)

if success:
    print("✅ Konfiguration gespeichert")
```

### YAML Lesen/Schreiben

```python
# YAML lesen
config = await FileReader.open_yaml_file(
    config_file=Path("/workspace/config/config.yml")
)

# YAML schreiben
await FileWriter.write_yaml_file(
    file=Path("/workspace/config/config.yml"),
    file_content=config
)
```

### Markdown Lesen

```python
# Markdown-Datei lesen
readme = await FileReader.open_markdown_file(
    config_file=Path("/workspace/docs/README.md")
)
print(readme)  # String mit Markdown-Inhalt
```

### Binary Files

```python
# Binary lesen
content = await FileReader.open_binary_file(
    file=Path("/workspace/data/image.png")
)

# Binary schreiben
await FileWriter.write_binary_file(
    file=Path("/workspace/data/output.bin"),
    content=binary_data
)
```

### Synchrone Varianten

```python
# Für nicht-async Kontext
config = FileReader.open_json_file_sync(
    config_file=Path("/workspace/config/settings.json")
)

FileWriter.write_json_file_sync(
    file=Path("/workspace/config/settings.json"),
    file_content=config
)
```

---

## 4. Environment Variables

Umgebungsvariablen laden und verwalten.

### .env Datei laden

```python
from vyra_base.helper.env_handler import EnvHandler
from pathlib import Path

# .env-Datei laden
env_path = Path("/workspace/.env")
EnvHandler.load_env(env_path)

# Mit dotenv
from dotenv import load_dotenv
load_dotenv(env_path)
```

### Umgebungsvariablen lesen

```python
import os

# Standard os.getenv
redis_host = os.getenv('REDIS_HOST', 'localhost')
redis_port = int(os.getenv('REDIS_PORT', '6379'))

# Boolean-Werte
debug_mode = os.getenv('DEBUG_MODE', 'false').lower() == 'true'
tls_enabled = os.getenv('REDIS_TLS_ENABLED', 'true').lower() == 'true'
```

### Typische VYRA Umgebungsvariablen

```bash
# Redis
REDIS_HOST=redis
REDIS_PORT=6379
REDIS_USERNAME=modulemanager
REDIS_PASSWORD=secure_password
REDIS_TLS_ENABLED=true

# Paths
STORAGE_PATH=/workspace/storage
CERTIFICATES_PATH=/workspace/storage/certificates
LOG_PATH=/workspace/log

# Development
VYRA_DEV_MODE=true
DEBUG_MODE=false
```

---

## 5. Cryptographic Functions

Verschlüsselung, Hashing und Token-Generierung.

### Password Hashing

```python
from vyra_base.helper.crypto_helper import CryptoHelper

# Passwort hashen (SHA-256)
password = "admin"
password_hash = CryptoHelper.hash_password(password)
print(password_hash)  # 8c6976e5b5410415bde908bd4dee15dfb167a9c873fc4bb8a81f6f2ab448a918

# Passwort verifizieren
is_valid = CryptoHelper.verify_password(
    password="admin",
    password_hash=password_hash
)
print(f"Valid: {is_valid}")  # True
```

### Token-Generierung

```python
# Zufälliger Token (32 Zeichen hex)
token = CryptoHelper.generate_token(length=32)
print(token)  # b4a3f5e7c9d1a2b8f3e6d4c7a9e1b5f2...

# Session-Token
session_token = CryptoHelper.generate_session_token()
print(session_token)  # 64-Zeichen hex
```

### UUID-Generierung

```python
import uuid

# UUID4 (random)
unique_id = str(uuid.uuid4())
print(unique_id)  # 550e8400-e29b-41d4-a716-446655440000

# UUID3 (namespace + name)
namespace = uuid.NAMESPACE_DNS
unique_id = str(uuid.uuid3(namespace, "my_module"))
```

### Verschlüsselung (AES)

```python
from vyra_base.helper.crypto_helper import CryptoHelper

# Daten verschlüsseln
key = CryptoHelper.generate_key()  # AES-256 Key
plaintext = "Sensitive data"

encrypted = CryptoHelper.encrypt(plaintext, key)
print(f"Encrypted: {encrypted}")

# Daten entschlüsseln
decrypted = CryptoHelper.decrypt(encrypted, key)
print(f"Decrypted: {decrypted}")  # "Sensitive data"
```

---

## 6. Utility Functions

Verschiedene Hilfsfunktionen.

### Deep Merge (Dictionaries)

```python
from vyra_base.helper.func import deep_merge

base_config = {
    "timeout": 30,
    "database": {
        "host": "localhost",
        "port": 5432
    }
}

user_config = {
    "timeout": 60,
    "database": {
        "port": 3306,
        "ssl": True
    },
    "logging": {
        "level": "DEBUG"
    }
}

merged = deep_merge(base_config, user_config)
print(merged)
# {
#     "timeout": 60,
#     "database": {
#         "host": "localhost",
#         "port": 3306,
#         "ssl": True
#     },
#     "logging": {
#         "level": "DEBUG"
#     }
# }
```

### File Lock

```python
from vyra_base.helper.file_lock import get_lock_for_file, release_lock_for_file
from pathlib import Path

# Async Lock
file_path = Path("/workspace/data/shared.json")
lock = await get_lock_for_file(file_path)

async with lock:
    # Exklusiver Zugriff auf Datei
    data = await read_file(file_path)
    await write_file(file_path, modified_data)

await release_lock_for_file(file_path)

# Sync Lock
from vyra_base.helper.file_lock import get_lock_for_file_sync, release_lock_for_file_sync

lock = get_lock_for_file_sync(file_path)
with lock:
    # Exklusiver Zugriff
    pass
release_lock_for_file_sync(file_path)
```

---

## Best Practices

### ✅ Empfohlen

```python
# Logging mit Kontext
Logger.log(LogEntry(f"Processing user {user_id}").debug())

# Error-Decorator verwenden
@ErrorTraceback.w_check_error_exist
async def risky_operation():
    pass

# Async File I/O
config = await FileReader.open_json_file(config_path)

# Type-Hints
from typing import Optional
def process_data(data: dict) -> Optional[str]:
    pass
```

### ❌ Vermeiden

```python
# NICHT: Logging ohne Kontext
Logger.log(LogEntry("Error"))  # Was für ein Fehler?

# NICHT: Fehler verschlucken
try:
    await operation()
except:
    pass  # Fehler wird ignoriert!

# NICHT: Sync I/O in async Kontext
with open("file.json") as f:  # Blockiert Event Loop!
    data = json.load(f)

# BESSER:
data = await FileReader.open_json_file(Path("file.json"))
```

---

## Integration in Component

```python
from vyra_base.state import OperationalStateMachine
from vyra_base.helper.logger import Logger, LogEntry
from vyra_base.helper.error_handler import ErrorTraceback
from vyra_base.helper.file_reader import FileReader
from pathlib import Path

class Component(OperationalStateMachine):
    
    @ErrorTraceback.w_check_error_exist
    async def initialize(self, request=None, response=None) -> bool:
        """Initialisiert Component mit Helper-Funktionen."""
        Logger.log(LogEntry("Initializing component").debug())
        
        # Konfiguration laden
        try:
            self.config = await FileReader.open_json_file(
                config_file=Path("/workspace/config/component.json")
            )
            Logger.log(LogEntry("Configuration loaded").debug())
        except FileNotFoundError:
            Logger.log(LogEntry("Configuration file not found, using defaults").warn())
            self.config = self._get_default_config()
        
        Logger.log(LogEntry("✅ Component initialized"))
        return True
    
    def _get_default_config(self) -> dict:
        """Gibt Default-Konfiguration zurück."""
        return {
            "timeout": 30,
            "retry_count": 3,
            "enabled": True
        }
```

---

## Siehe auch

- **Storage API**: [../storage/README.md](../storage/README.md)
- **State Machine**: [../state/README.md](../state/README.md)
- **Communication**: [../com/README.md](../com/README.md)
