# Datenbank-Operationen

Dieser Guide zeigt alle verfügbaren CRUD-Operationen mit `DbManipulator`.

## Übersicht der Operationen

| Operation | Methode | Beschreibung |
|-----------|---------|--------------|
| **Create** | `create()` | Neuen Datensatz erstellen |
| **Read** | `get_by_id()`, `get_all()` | Datensätze lesen |
| **Update** | `update()` | Datensatz aktualisieren |
| **Delete** | `delete()` | Datensatz löschen |
| **Count** | `count()` | Anzahl Datensätze zählen |

## 1. Create - Datensätze erstellen

### Einzelner Datensatz

```python
from vyra_base.storage.db_access import DBSTATUS

# Daten vorbereiten
new_user = {
    "username": "max.mustermann",
    "email": "max@example.com",
    "password_hash": "hashed_password_here",
    "role": "USER",
    "level": "LEVEL_1",
    "enabled": True
}

# Erstellen
result = await user_db.create(data=new_user)

if result.status == DBSTATUS.SUCCESS:
    created_user = result.value  # SQLAlchemy Model-Instanz
    print(f"✅ Benutzer erstellt: {created_user.username} (ID: {created_user.id})")
else:
    print(f"❌ Fehler: {result.value}")
```

### Mehrere Datensätze (Bulk Insert)

```python
users = [
    {"username": "user1", "email": "user1@example.com", "role": "USER"},
    {"username": "user2", "email": "user2@example.com", "role": "ADMIN"},
    {"username": "user3", "email": "user3@example.com", "role": "USER"}
]

result = await user_db.create(data=users)

if result.status == DBSTATUS.SUCCESS:
    created_users = result.value  # Liste von Model-Instanzen
    print(f"✅ {len(created_users)} Benutzer erstellt")
```

## 2. Read - Datensätze lesen

### Alle Datensätze

```python
result = await user_db.get_all()

if result.status == DBSTATUS.SUCCESS:
    users = result.value  # Liste von SQLAlchemy-Objekten
    for user in users:
        print(f"- {user.username} ({user.email})")
else:
    print("Keine Benutzer gefunden")
```

### Mit Filtern

```python
# Filtern nach einem Feld
result = await user_db.get_all(filters={"role": "ADMIN"})

# Mehrere Filter (UND-Verknüpfung)
result = await user_db.get_all(filters={
    "role": "USER",
    "enabled": True
})

# Mit Sortierung
result = await user_db.get_all(
    filters={"enabled": True},
    order_by="username"  # Nach username sortieren
)
```

### Nach ID lesen

```python
result = await user_db.get_by_id(id=5)

if result.status == DBSTATUS.SUCCESS:
    user = result.value
    print(f"Benutzer: {user.username}")
elif result.status == DBSTATUS.NOT_FOUND:
    print("Benutzer nicht gefunden")
```

### Letzten Datensatz lesen

```python
# ID = -1 oder None gibt letzten Eintrag zurück
result = await user_db.get_by_id(id=-1)

if result.status == DBSTATUS.SUCCESS:
    last_user = result.value
    print(f"Letzter Benutzer: {last_user.username}")
```

## 3. Update - Datensätze aktualisieren

### Nach ID aktualisieren

```python
result = await user_db.update(
    id=5,
    data={
        "email": "new.email@example.com",
        "role": "ADMIN"
    }
)

if result.status == DBSTATUS.SUCCESS:
    updated_user = result.value
    print(f"✅ Aktualisiert: {updated_user.username}")
elif result.status == DBSTATUS.NOT_FOUND:
    print("❌ Benutzer nicht gefunden")
```

### Mit Filtern aktualisieren

```python
# Alle inaktiven Benutzer aktivieren
result = await user_db.update(
    filters={"enabled": False},
    data={"enabled": True}
)

if result.status == DBSTATUS.SUCCESS:
    count = result.value  # Anzahl aktualisierter Zeilen
    print(f"✅ {count} Benutzer aktiviert")
```

### Teilweise Aktualisierung

```python
# Nur angegebene Felder werden aktualisiert
result = await user_db.update(
    id=3,
    data={"last_login": "2026-01-29 14:30:00"}
)
# Andere Felder bleiben unverändert
```

## 4. Delete - Datensätze löschen

### Nach ID löschen

```python
result = await user_db.delete(id=5)

if result.status == DBSTATUS.SUCCESS:
    print(f"✅ Gelöscht: {result.value} Datensatz(e)")
elif result.status == DBSTATUS.NOT_FOUND:
    print("❌ Datensatz nicht gefunden")
```

### Mit Filtern löschen

```python
# Alle inaktiven Benutzer älter als 30 Tage löschen
result = await user_db.delete(filters={
    "enabled": False,
    "created_at": {"<": "2025-12-30"}  # Erweiterte Filter
})

if result.status == DBSTATUS.SUCCESS:
    count = result.value
    print(f"✅ {count} Benutzer gelöscht")
```

### Alle Datensätze löschen (Vorsicht!)

```python
# Komplette Tabelle leeren
result = await user_db.delete(filters={})

if result.status == DBSTATUS.SUCCESS:
    print(f"⚠️ {result.value} Datensätze gelöscht")
```

## 5. Count - Datensätze zählen

```python
# Alle Datensätze zählen
result = await user_db.count()
print(f"Gesamt: {result.value} Benutzer")

# Mit Filtern zählen
result = await user_db.count(filters={"role": "ADMIN"})
print(f"Admins: {result.value}")

result = await user_db.count(filters={"enabled": True})
print(f"Aktive Benutzer: {result.value}")
```

## 6. Erweiterte Filter

### Vergleichsoperatoren

```python
# Größer als
result = await user_db.get_all(filters={
    "id": {">": 10}
})

# Kleiner als oder gleich
result = await user_db.get_all(filters={
    "login_attempts": {"<=": 3}
})

# Ungleich
result = await user_db.get_all(filters={
    "role": {"!=": "GUEST"}
})

# IN-Liste
result = await user_db.get_all(filters={
    "role": ["ADMIN", "MODERATOR"]
})
```

### LIKE-Suche

```python
# Benutzer mit "admin" im Namen finden
result = await user_db.get_all(filters={
    "username": {"like": "%admin%"}
})

# Emails von example.com
result = await user_db.get_all(filters={
    "email": {"like": "%@example.com"}
})
```

### NULL-Checks

```python
# Benutzer ohne Email
result = await user_db.get_all(filters={
    "email": None
})

# Benutzer mit Email
result = await user_db.get_all(filters={
    "email": {"!=": None}
})
```

## 7. Transaktionen

### Manuelle Transaktionen

```python
from vyra_base.storage.db_access import DbAccess

async with db.session() as session:
    try:
        # Mehrere Operationen in einer Transaktion
        user1 = await user_db.create(data={"username": "user1"})
        user2 = await user_db.create(data={"username": "user2"})
        
        await session.commit()
        print("✅ Beide Benutzer erstellt")
    except Exception as e:
        await session.rollback()
        print(f"❌ Rollback: {e}")
```

### Automatische Transaktionen

Die meisten DbManipulator-Operationen verwenden automatisch Transaktionen:

```python
# Automatisch committed bei Erfolg
result = await user_db.create(data={"username": "test"})

# Automatisch Rollback bei Fehler
try:
    result = await user_db.create(data={"id": None})  # Fehler
except Exception:
    pass  # Rollback bereits durchgeführt
```

## 8. Fehlerbehandlung

### Status-Codes prüfen

```python
from vyra_base.storage.db_access import DBSTATUS

result = await user_db.get_by_id(id=999)

if result.status == DBSTATUS.SUCCESS:
    user = result.value
    print(f"Gefunden: {user.username}")
    
elif result.status == DBSTATUS.NOT_FOUND:
    print("Benutzer existiert nicht")
    
elif result.status == DBSTATUS.ERROR:
    print(f"Fehler: {result.value}")
    print(f"Details: {result.details}")
    
elif result.status == DBSTATUS.DUPLICATE:
    print("Datensatz bereits vorhanden (Primary Key Konflikt)")
```

### Try-Except

```python
from vyra_base.helper.logger import Logger, LogEntry

try:
    result = await user_db.create(data=new_user)
    if result.status == DBSTATUS.SUCCESS:
        return result.value
    else:
        Logger.log(LogEntry(f"Create failed: {result.value}").error())
        return None
except Exception as e:
    Logger.log(LogEntry(f"Exception: {e}").error())
    return None
```

## 9. Best Practices

### ✅ Empfohlen

```python
# Immer Status prüfen
result = await user_db.get_by_id(id=5)
if result.status == DBSTATUS.SUCCESS:
    user = result.value
    
# Felder explizit angeben
data = {"username": "test", "email": "test@example.com"}
await user_db.create(data=data)

# Filter verwenden statt get_all() + Python-Filterung
result = await user_db.get_all(filters={"enabled": True})
```

### ❌ Vermeiden

```python
# NICHT: Status nicht prüfen
user = (await user_db.get_by_id(id=5)).value  # Kann None sein!

# NICHT: Alle laden und filtern
all_users = await user_db.get_all()
active = [u for u in all_users.value if u.enabled]  # Ineffizient!

# NICHT: Einzelne Updates in Schleife
for user_id in user_ids:
    await user_db.update(id=user_id, data={"status": "active"})
# BESSER: Bulk-Update verwenden
```

## 10. Performance-Tipps

### Batch-Operationen verwenden

```python
# ✅ Gut: Bulk Insert
await user_db.create(data=users_list)

# ❌ Schlecht: Einzeln in Schleife
for user in users_list:
    await user_db.create(data=user)
```

### Nur benötigte Felder laden

```python
# Filter auf DB-Ebene anwenden
result = await user_db.get_all(
    filters={"enabled": True},
    order_by="created_at"
)
```

### Indizes verwenden

```python
# In Ihrem Model definieren:
from sqlalchemy import Index

class User(Base):
    __tablename__ = "user"
    username = mapped_column(String(100), index=True)  # Index für Suchen
    email = mapped_column(String(255), unique=True)    # Unique constraint
    
    __table_args__ = (
        Index('idx_user_role_enabled', 'role', 'enabled'),  # Composite index
    )
```

## Nächste Schritte

- [Redis-Verbindung einrichten](./redis_connection.md)
- [Tabellen und Modelle erstellen](./table_creation.md)
- [API-Referenz](./api_reference.md)
