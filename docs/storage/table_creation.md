# Datenbank-Tabellen anlegen

Dieser Guide zeigt, wie Sie eigene Datenbank-Tabellen mit SQLAlchemy-Modellen erstellen.

## 1. Grundlagen

### SQLAlchemy Mapped Columns

VYRA verwendet die moderne SQLAlchemy 2.0 Syntax mit `Mapped` und `mapped_column`:

```python
from sqlalchemy import String, Integer, Boolean, JSON
from sqlalchemy.orm import Mapped, mapped_column
from vyra_base.storage.tb_base import Base

class MyTable(Base):
    __tablename__ = "my_table"
    
    # Primary Key (required)
    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)
    
    # String-Felder
    name: Mapped[str] = mapped_column(String(100), nullable=False)
    description: Mapped[str] = mapped_column(String(500), default="")
    
    # Numerische Felder
    count: Mapped[int] = mapped_column(Integer(), default=0)
    value: Mapped[float] = mapped_column(default=0.0)
    
    # Boolean
    enabled: Mapped[bool] = mapped_column(Boolean(), default=True)
    
    # JSON-Felder
    metadata: Mapped[dict] = mapped_column(JSON(), default={})
```

### Wichtige Regeln

1. **Immer von `Base` erben**: `from vyra_base.storage.tb_base import Base`
2. **`__tablename__` definieren**: Tabellenname in Datenbank
3. **Primary Key erforderlich**: Mindestens eine Spalte als `primary_key=True`
4. **Type Hints verwenden**: `Mapped[type]` für alle Spalten

## 2. Beispiel: Parameter-Tabelle (tb_params)

Vollständiges Beispiel aus vyra_base:

```python
from enum import Enum
from sqlalchemy import JSON, String
from sqlalchemy import Enum as SQLEnum
from sqlalchemy.orm import Mapped, mapped_column
from typing import Any, Optional

from vyra_base.storage.tb_base import Base


class TypeEnum(str, Enum):
    """
    Enumeration für unterstützte Parameter-Datentypen.
    """
    integer = "int"
    string = "string"
    boolean = "bool"
    float = "float"
    list = "list"
    dict = "dict"


class Parameter(Base):
    """
    SQLAlchemy-Modell für Modul-Parameter mit Metadaten.
    
    Speichert konfigurierbare Parameter mit Typinformationen,
    Constraints, Display-Metadaten und Sichtbarkeits-Flags.
    """
    __tablename__ = "parameter"

    # Primary Key
    name: Mapped[str] = mapped_column(
        primary_key=True,
        unique=True
    )
    
    # Parameter-Wert (JSON)
    value: Mapped[dict[str, Any]] = mapped_column(
        JSON(), 
        nullable=False
    )
    
    # Default-Wert für Reset
    default_value: Mapped[dict[str, Any]] = mapped_column(
        JSON(), 
        nullable=False, 
        default={}
    )
    
    # Datentyp (Enum)
    type: Mapped[TypeEnum] = mapped_column(
        SQLEnum(TypeEnum, values_callable=lambda enum: [e.value for e in enum]),
        nullable=False
    )
    
    # UI-Flags
    visible: Mapped[bool] = mapped_column(
        nullable=False, 
        default=True
    )
    editable: Mapped[bool] = mapped_column(
        nullable=False, 
        default=True
    )
    
    # Display-Informationen
    displayname: Mapped[str] = mapped_column(
        nullable=False, 
        default=""
    )
    description: Mapped[str] = mapped_column(
        nullable=False, 
        default=""
    )
    
    # Validierungs-Constraints (optional)
    min_value: Mapped[Optional[str]] = mapped_column(
        String(), 
        nullable=True, 
        default=None
    )
    max_value: Mapped[Optional[str]] = mapped_column(
        String(), 
        nullable=True, 
        default=None
    )
    range_value: Mapped[Optional[dict[str, Any]]] = mapped_column(
        JSON(), 
        nullable=True, 
        default=None
    )
```

### Verwendung

```python
from vyra_base.storage.db_access import DbAccess
from vyra_base.storage.db_manipulator import DbManipulator
from your_module.models import Parameter, TypeEnum

# Datenbank öffnen
db = DbAccess(db_path="/workspace/storage/database/config.db")
await db.open()

# Manipulator erstellen
param_db = DbManipulator(db_access=db, model=Parameter)

# Parameter erstellen
new_param = {
    "name": "timeout",
    "value": {"val": 30},
    "default_value": {"val": 30},
    "type": TypeEnum.integer,
    "visible": True,
    "editable": True,
    "displayname": "Timeout (Sekunden)",
    "description": "Maximale Wartezeit für Verbindungen",
    "min_value": "1",
    "max_value": "300"
}

result = await param_db.create(data=new_param)
if result.status == DBSTATUS.SUCCESS:
    print(f"✅ Parameter erstellt: {result.value.name}")
```

## 3. Eigene Tabelle erstellen

### Beispiel: Benutzer-Tabelle

```python
from datetime import datetime
from sqlalchemy import String, DateTime, Integer
from sqlalchemy.orm import Mapped, mapped_column
from enum import Enum

from vyra_base.storage.tb_base import Base


class UserRole(str, Enum):
    """Benutzer-Rollen."""
    ADMIN = "ADMIN"
    USER = "USER"
    GUEST = "GUEST"


class User(Base):
    """Benutzer-Modell mit Authentifizierung."""
    __tablename__ = "user"
    
    # Primary Key (auto-increment)
    id: Mapped[int] = mapped_column(
        primary_key=True, 
        autoincrement=True
    )
    
    # Eindeutige Felder
    username: Mapped[str] = mapped_column(
        String(100), 
        unique=True, 
        nullable=False,
        index=True  # Index für schnelle Suche
    )
    email: Mapped[str] = mapped_column(
        String(255), 
        unique=True, 
        nullable=False
    )
    
    # Authentifizierung
    password_hash: Mapped[str] = mapped_column(
        String(255), 
        nullable=False
    )
    
    # Rolle (Enum)
    role: Mapped[UserRole] = mapped_column(
        SQLEnum(UserRole, values_callable=lambda enum: [e.value for e in enum]),
        nullable=False,
        default=UserRole.USER
    )
    
    # Status
    enabled: Mapped[bool] = mapped_column(
        Boolean(),
        nullable=False,
        default=True
    )
    
    # Timestamps
    created_at: Mapped[datetime] = mapped_column(
        DateTime(),
        nullable=False,
        default=datetime.utcnow
    )
    last_login: Mapped[Optional[datetime]] = mapped_column(
        DateTime(),
        nullable=True,
        default=None
    )
    
    # Metadaten (JSON)
    user_metadata: Mapped[Optional[dict]] = mapped_column(
        JSON(),
        nullable=True,
        default=None
    )
    
    # Login-Sicherheit
    login_attempts: Mapped[int] = mapped_column(
        Integer(),
        default=0
    )
    locked_until: Mapped[Optional[datetime]] = mapped_column(
        DateTime(),
        nullable=True,
        default=None
    )
```

### Datei-Struktur

```
your_module/
├── src/
│   └── your_module/
│       ├── models/
│       │   ├── __init__.py
│       │   ├── user.py          # User-Modell
│       │   ├── settings.py      # Settings-Modell
│       │   └── log_entry.py     # LogEntry-Modell
│       └── application/
│           └── component.py     # Verwendet die Modelle
```

**models/__init__.py:**
```python
from .user import User, UserRole
from .settings import Settings
from .log_entry import LogEntry

__all__ = ["User", "UserRole", "Settings", "LogEntry"]
```

## 4. Erweiterte Features

### Relationships (Foreign Keys)

```python
from sqlalchemy import ForeignKey
from sqlalchemy.orm import relationship

class Order(Base):
    __tablename__ = "order"
    
    id: Mapped[int] = mapped_column(primary_key=True)
    user_id: Mapped[int] = mapped_column(ForeignKey("user.id"))
    product_name: Mapped[str] = mapped_column(String(200))
    
    # Relationship zu User
    user: Mapped["User"] = relationship(back_populates="orders")

class User(Base):
    __tablename__ = "user"
    
    id: Mapped[int] = mapped_column(primary_key=True)
    username: Mapped[str] = mapped_column(String(100))
    
    # Relationship zu Orders
    orders: Mapped[list["Order"]] = relationship(back_populates="user")
```

### Composite Primary Key

```python
class ModulePermission(Base):
    __tablename__ = "module_permission"
    
    user_id: Mapped[int] = mapped_column(primary_key=True)
    module_name: Mapped[str] = mapped_column(String(100), primary_key=True)
    can_read: Mapped[bool] = mapped_column(default=True)
    can_write: Mapped[bool] = mapped_column(default=False)
```

### Table Args (Constraints, Indizes)

```python
from sqlalchemy import Index, UniqueConstraint, CheckConstraint

class Product(Base):
    __tablename__ = "product"
    
    id: Mapped[int] = mapped_column(primary_key=True)
    name: Mapped[str] = mapped_column(String(200))
    category: Mapped[str] = mapped_column(String(100))
    price: Mapped[float] = mapped_column()
    stock: Mapped[int] = mapped_column()
    
    __table_args__ = (
        # Composite Index
        Index('idx_product_category_name', 'category', 'name'),
        
        # Unique Constraint
        UniqueConstraint('name', 'category', name='uq_product_name_category'),
        
        # Check Constraint
        CheckConstraint('price >= 0', name='check_positive_price'),
        CheckConstraint('stock >= 0', name='check_positive_stock'),
    )
```

### Default-Funktionen

```python
import uuid
from datetime import datetime

class LogEntry(Base):
    __tablename__ = "log_entry"
    
    # UUID als Primary Key
    id: Mapped[str] = mapped_column(
        String(36),
        primary_key=True,
        default=lambda: str(uuid.uuid4())
    )
    
    # Automatischer Timestamp
    timestamp: Mapped[datetime] = mapped_column(
        DateTime(),
        default=datetime.utcnow
    )
    
    message: Mapped[str] = mapped_column(String(500))
    level: Mapped[str] = mapped_column(String(20))
```

## 5. Migration & Schema-Updates

### Neue Tabelle hinzufügen

```python
# 1. Modell in models/ erstellen
# 2. In models/__init__.py importieren
# 3. Datenbank neu öffnen (erstellt Tabellen automatisch)

db = DbAccess(db_path="/workspace/storage/database/module.db")
await db.open()  # Erstellt fehlende Tabellen
```

### Spalte hinzufügen (Breaking Change!)

```python
# WARNUNG: Änderungen an existierenden Tabellen erfordern Migration!
# Für Produktion: Alembic verwenden

# Beispiel: Neue Spalte mit Default-Wert
class User(Base):
    __tablename__ = "user"
    
    id: Mapped[int] = mapped_column(primary_key=True)
    username: Mapped[str] = mapped_column(String(100))
    
    # NEU: Phone-Nummer (optional)
    phone: Mapped[Optional[str]] = mapped_column(
        String(20),
        nullable=True,
        default=None
    )
```

**Empfehlung für Produktion:**
1. Alembic für Schema-Migrationen verwenden
2. Versionskontrolle für Schema-Änderungen
3. Backup vor Schema-Änderungen

### Alembic-Setup (optional)

```bash
# Installation
pip install alembic

# Initialisierung
alembic init alembic

# Migration erstellen
alembic revision --autogenerate -m "Add phone column to user"

# Migration anwenden
alembic upgrade head
```

## 6. Best Practices

### ✅ Empfohlen

```python
# Primary Key immer definieren
id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)

# Indizes für häufig gesuchte Felder
username: Mapped[str] = mapped_column(String(100), index=True)

# Nullable explizit angeben
email: Mapped[Optional[str]] = mapped_column(String(255), nullable=True)

# Default-Werte für optionale Felder
enabled: Mapped[bool] = mapped_column(default=True)

# String-Längen begrenzen
name: Mapped[str] = mapped_column(String(100))  # Nicht String()

# JSON für strukturierte Daten
metadata: Mapped[dict] = mapped_column(JSON(), default={})

# Timestamps tracken
created_at: Mapped[datetime] = mapped_column(DateTime(), default=datetime.utcnow)
```

### ❌ Vermeiden

```python
# NICHT: Keine Primary Key
class BadTable(Base):
    __tablename__ = "bad_table"
    name: Mapped[str] = mapped_column(String(100))  # FEHLER!

# NICHT: Unbegrenzte Strings
text: Mapped[str] = mapped_column(String())  # Langsam!

# NICHT: Nullable ohne Optional
email: Mapped[str] = mapped_column(nullable=True)  # Type-Hint falsch!

# BESSER:
email: Mapped[Optional[str]] = mapped_column(nullable=True)

# NICHT: Mutable Defaults
tags: Mapped[list] = mapped_column(JSON(), default=[])  # Gefährlich!

# BESSER:
tags: Mapped[list] = mapped_column(JSON(), default=list)
```

## 7. Troubleshooting

### Tabelle wird nicht erstellt

```python
# Prüfen, ob Modell in Base registriert ist
from vyra_base.storage.tb_base import Base
print(Base.metadata.tables.keys())
# Sollte Ihre Tabelle enthalten

# Manuelle Tabellenerstellung
await db.create_tables()  # Erstellt alle registrierten Tabellen
```

### "Table already exists" Fehler

```python
# Tabelle existiert bereits in DB
# Lösung 1: DB löschen (Development only!)
import os
os.remove("/workspace/storage/database/module.db")

# Lösung 2: IF NOT EXISTS verwenden (automatisch bei db.open())
db = DbAccess(db_path="/workspace/storage/database/module.db")
await db.open()  # Verwendet IF NOT EXISTS
```

### Enum-Werte ändern

```python
# WARNUNG: Existierende Enum-Werte ändern erfordert Migration!

# Alt:
class Status(str, Enum):
    ACTIVE = "active"
    INACTIVE = "inactive"

# Neu (mit neuem Wert):
class Status(str, Enum):
    ACTIVE = "active"
    INACTIVE = "inactive"
    SUSPENDED = "suspended"  # NEU

# Existierende Datenbank unterstützt SUSPENDED nicht automatisch!
# Lösung: Alembic Migration oder Spalte neu erstellen
```

## Nächste Schritte

- [Datenbank-Operationen durchführen](./database_operations.md)
- [API-Referenz](./api_reference.md)
- [Zurück zur Übersicht](./README.md)
