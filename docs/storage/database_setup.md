# Datenbankzugriff einrichten

Diese Anleitung zeigt, wie Sie Datenbankzugriff in Ihrem VYRA-Modul einrichten.

## 1. Grundlegende Einrichtung

### Imports

```python
from vyra_base.storage.db_access import DbAccess, DBSTATUS
from vyra_base.storage.db_manipulator import DbManipulator, DBReturnValue
from vyra_base.storage.tb_base import Base
```

### DbAccess erstellen

Die `DbAccess`-Klasse verwaltet die SQLAlchemy-Engine und Sessions.

```python
# Initialisierung
db = DbAccess(
    db_path="/workspace/storage/database/my_module.db",
    echo=False  # SQL-Logging deaktivieren (für Produktion)
)

# Verbindung öffnen
await db.open()

# ... Operationen durchführen ...

# Verbindung schließen
await db.close()
```

**Standard-Pfad:** `/workspace/storage/database/<modulename>.db`

### Context Manager (empfohlen)

```python
async with DbAccess(db_path="/workspace/storage/database/module.db") as db:
    # Verbindung automatisch geöffnet
    user_db = DbManipulator(db_access=db, model=User)
    result = await user_db.get_all()
    # Verbindung wird automatisch geschlossen
```

## 2. DbManipulator erstellen

Für jede Tabelle (SQLAlchemy-Model) erstellen Sie einen `DbManipulator`:

```python
from your_module.models import User, Settings, LogEntry

# Initialisierung der Manipulatoren
user_db = DbManipulator(db_access=db, model=User)
settings_db = DbManipulator(db_access=db, model=Settings)
log_db = DbManipulator(db_access=db, model=LogEntry)
```

## 3. Integration in Ihre Komponente

### Beispiel: Component-Klasse

```python
from vyra_base.state import OperationalStateMachine
from vyra_base.storage.db_access import DbAccess
from vyra_base.storage.db_manipulator import DbManipulator
from vyra_base.helper.error_handler import ErrorTraceback

from .models import User, Parameter

class Component(OperationalStateMachine):
    
    @ErrorTraceback.w_check_error_exist
    def __init__(self):
        super().__init__()
        self.db: DbAccess = None
        self.user_db: DbManipulator = None
        self.param_db: DbManipulator = None
    
    async def initialize(self, request=None, response=None) -> bool:
        """Initialisiert Datenbankverbindungen."""
        # Datenbank öffnen
        self.db = DbAccess(
            db_path="/workspace/storage/database/component.db"
        )
        await self.db.open()
        
        # Manipulatoren erstellen
        self.user_db = DbManipulator(db_access=self.db, model=User)
        self.param_db = DbManipulator(db_access=self.db, model=Parameter)
        
        # Überprüfen, ob Tabellen existieren
        await self._ensure_tables()
        
        return True
    
    async def _ensure_tables(self):
        """Erstellt Tabellen, falls sie nicht existieren."""
        # Tabellen werden automatisch erstellt bei db.open()
        # wenn Modelle in Base registriert sind
        pass
    
    async def stop(self, request=None, response=None) -> bool:
        """Schließt Datenbankverbindungen."""
        if self.db:
            await self.db.close()
        return True
```

## 4. Datenbankpfade

### Empfohlene Struktur

```
/workspace/storage/
├── database/
│   ├── module_name.db          # Haupt-Datenbank
│   ├── user_management.db      # Benutzer (optional separate DB)
│   └── logs.db                 # Logs (optional separate DB)
├── certificates/
│   └── redis/
└── config/
```

### Umgebungsvariablen

```python
import os
from pathlib import Path

# Aus Umgebungsvariablen
storage_path = Path(os.getenv('STORAGE_PATH', '/workspace/storage'))
db_path = storage_path / 'database' / 'module.db'

db = DbAccess(db_path=str(db_path))
```

## 5. Mehrere Datenbanken

Sie können mehrere Datenbanken parallel verwenden:

```python
# Haupt-Datenbank
main_db = DbAccess(db_path="/workspace/storage/database/main.db")
await main_db.open()

# Benutzer-Datenbank
user_db = DbAccess(db_path="/workspace/storage/database/users.db")
await user_db.open()

# Manipulatoren für verschiedene Datenbanken
user_manipulator = DbManipulator(db_access=user_db, model=User)
config_manipulator = DbManipulator(db_access=main_db, model=Config)

# ... Operationen ...

# Aufräumen
await main_db.close()
await user_db.close()
```

## 6. Fehlerbehandlung

```python
from vyra_base.helper.logger import Logger, LogEntry

try:
    db = DbAccess(db_path="/workspace/storage/database/module.db")
    await db.open()
except Exception as e:
    Logger.log(LogEntry(f"Datenbankverbindung fehlgeschlagen: {e}").error())
    raise

# Mit ErrorTraceback-Decorator
@ErrorTraceback.w_check_error_exist
async def setup_database():
    db = DbAccess(db_path="/workspace/storage/database/module.db")
    await db.open()
    return db
```

## 7. Debugging

### SQL-Logging aktivieren

```python
db = DbAccess(
    db_path="/workspace/storage/database/module.db",
    echo=True  # Alle SQL-Befehle werden geloggt
)
```

**Ausgabe:**
```
2026-01-29 14:00:00 INFO sqlalchemy.engine.Engine SELECT user.id, user.username ...
```

### Tabellenstruktur prüfen

```python
user_db = DbManipulator(db_access=db, model=User)
result = user_db.get_table_structure()

if result.status == DBSTATUS.SUCCESS:
    columns = result.value
    print(f"Spalten: {columns}")
    # Ausgabe: ['id', 'username', 'password_hash', 'email', 'role', ...]
```

## Nächste Schritte

- [Datenbank-Operationen durchführen](./database_operations.md)
- [Tabellen und Modelle erstellen](./table_creation.md)
- [API-Referenz](./api_reference.md)
