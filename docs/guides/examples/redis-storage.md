# Example: Database and Storage Operations

Learn how to work with VYRA's database and storage layer.

## Setup

```python
from vyra_base.storage.db_access import DbAccess
from vyra_base.storage.db_manipulator import DbManipulator
from vyra_base.storage.storage import DBSTATUS
from sqlalchemy import Column, String, Integer, DateTime
from sqlalchemy.orm import declarative_base
from datetime import datetime
import asyncio

Base = declarative_base()
```

## Define a Model

```python
class User(Base):
    """User model for database."""
    
    __tablename__ = "users"
    
    id = Column(Integer, primary_key=True)
    username = Column(String(50), unique=True, nullable=False)
    email = Column(String(100), unique=True, nullable=False)
    age = Column(Integer, default=0)
    created_at = Column(DateTime, default=datetime.now)
    
    def __repr__(self):
        return f"<User(username='{self.username}', email='{self.email}', age={self.age})>"
```

## Basic Operations

### Setup Database

```python
async def setup_database():
    """Initialize database connection."""
    db_access = DbAccess(
        db_path="sqlite:///./vyra_app.db",
        db_echo=False  # Set True for SQL logging
    )
    
    await db_access.open()
    return db_access
```

### Create Records (Add)

```python
async def add_user(db_manipulator, username, email, age):
    """Add a new user to database."""
    
    user = User(
        username=username,
        email=email,
        age=age
    )
    
    result = await db_manipulator.add(user)
    
    if result.status == DBSTATUS.SUCCESS:
        print(f"✅ Added user: {username}")
        return result.value
    else:
        print(f"❌ Failed to add user: {result.error}")
        return None
```

### Read Records (Get)

```python
async def get_user_by_username(db_manipulator, username):
    """Get user by username."""
    
    result = await db_manipulator.get_one(filters={"username": username})
    
    if result.status == DBSTATUS.SUCCESS:
        user = result.value
        print(f"✅ Found user: {user}")
        return user
    else:
        print(f"⚠️  User not found")
        return None

async def get_all_users(db_manipulator):
    """Get all users."""
    
    result = await db_manipulator.get_all()
    
    if result.status == DBSTATUS.SUCCESS:
        users = result.value
        print(f"✅ Found {len(users)} users:")
        for user in users:
            print(f"  - {user.username} ({user.email})")
        return users
    else:
        print(f"❌ Failed to get users")
        return []
```

### Update Records

```python
async def update_user_age(db_manipulator, username, new_age):
    """Update user's age."""
    
    result = await db_manipulator.update(
        data={"age": new_age},
        filters={"username": username}
    )
    
    if result.status == DBSTATUS.SUCCESS:
        print(f"✅ Updated {username} age to {new_age}")
        return True
    else:
        print(f"❌ Failed to update user")
        return False
```

### Delete Records

```python
async def delete_user(db_manipulator, username):
    """Delete user by username."""
    
    result = await db_manipulator.delete(filters={"username": username})
    
    if result.status == DBSTATUS.SUCCESS:
        print(f"✅ Deleted user: {username}")
        return True
    else:
        print(f"❌ Failed to delete user")
        return False
```

### Bulk Operations

```python
async def bulk_add_users(db_manipulator, users_data):
    """Add multiple users at once."""
    
    users = [
        User(username=u["username"], email=u["email"], age=u["age"])
        for u in users_data
    ]
    
    result = await db_manipulator.bulk_add(users)
    
    if result.status == DBSTATUS.SUCCESS:
        print(f"✅ Added {len(users)} users in bulk")
        return True
    else:
        print(f"❌ Bulk add failed")
        return False

async def bulk_delete_users(db_manipulator, age_threshold):
    """Delete all users below age threshold."""
    
    result = await db_manipulator.bulk_delete(
        filters={"age": {"<": age_threshold}}
    )
    
    if result.status == DBSTATUS.SUCCESS:
        print(f"✅ Deleted users younger than {age_threshold}")
        return True
    else:
        print(f"❌ Bulk delete failed")
        return False
```

## Complete Example

```python
async def main():
    # Setup
    db_access = await setup_database()
    user_db = DbManipulator(db_access=db_access, model=User)
    
    print("📊 VYRA Storage Example\n")
    
    # Add single user
    print("1️⃣  Adding single user...")
    user1 = User(
        username="alice",
        email="alice@example.com",
        age=28
    )
    result = await user_db.add(user1)
    if result.status == DBSTATUS.SUCCESS:
        print("✅ Added alice\n")
    
    # Add multiple users
    print("2️⃣  Adding multiple users...")
    bulk_data = [
        {"username": "bob", "email": "bob@example.com", "age": 32},
        {"username": "charlie", "email": "charlie@example.com", "age": 25},
        {"username": "diana", "email": "diana@example.com", "age": 29},
    ]
    await bulk_add_users(user_db, bulk_data)
    print()
    
    # List all users
    print("3️⃣  Listing all users...")
    await get_all_users(user_db)
    print()
    
    # Get specific user
    print("4️⃣  Getting specific user...")
    user = await get_user_by_username(user_db, "bob")
    print()
    
    # Update user
    print("5️⃣  Updating user age...")
    await update_user_age(user_db, "bob", 33)
    print()
    
    # Delete user
    print("6️⃣  Deleting user...")
    await delete_user(user_db, "charlie")
    print()
    
    # Final list
    print("7️⃣  Final user list...")
    await get_all_users(user_db)
    print()
    
    # Cleanup
    await db_access.close()
    print("✅ Database closed")
```

## Output

```
📊 VYRA Storage Example

1️⃣  Adding single user...
✅ Added alice

2️⃣  Adding multiple users...
✅ Added 3 users in bulk

3️⃣  Listing all users...
✅ Found 4 users:
  - alice (alice@example.com)
  - bob (bob@example.com)
  - charlie (charlie@example.com)
  - diana (diana@example.com)

4️⃣  Getting specific user...
✅ Found user: <User(username='bob', email='bob@example.com', age=32)>

5️⃣  Updating user age...
✅ Updated bob age to 33

6️⃣  Deleting user...
✅ Deleted user: charlie

7️⃣  Final user list...
✅ Found 3 users:
  - alice (alice@example.com)
  - bob (bob@example.com)
  - diana (diana@example.com)

✅ Database closed
```

## Error Handling

```python
async def safe_database_operation():
    """Example with error handling."""
    
    db_access = await setup_database()
    user_db = DbManipulator(db_access=db_access, model=User)
    
    try:
        # Operation
        result = await user_db.get_all()
        
        if result.status == DBSTATUS.SUCCESS:
            return result.value
        elif result.status == DBSTATUS.NOT_FOUND:
            print("⚠️  No records found")
            return []
        elif result.status == DBSTATUS.ERROR:
            print(f"❌ Database error: {result.error}")
            return None
        else:
            print(f"❓ Unknown status: {result.status}")
            return None
            
    except Exception as e:
        print(f"❌ Exception: {e}")
        return None
    
    finally:
        await db_access.close()
```

## Transactions

```python
async def transaction_example():
    """Example using transactions."""
    
    db_access = await setup_database()
    user_db = DbManipulator(db_access=db_access, model=User)
    
    # Transaction wrapper
    async with db_access.session() as session:
        try:
            # Multiple operations
            user1 = User(username="user1", email="user1@example.com", age=20)
            user2 = User(username="user2", email="user2@example.com", age=25)
            
            session.add(user1)
            session.add(user2)
            
            # Commit
            session.commit()
            print("✅ Transaction committed")
            
        except Exception as e:
            session.rollback()
            print(f"❌ Transaction rolled back: {e}")
    
    await db_access.close()
```

## Return Value Codes

| Status | Meaning | Action |
|--------|---------|--------|
| SUCCESS | Operation succeeded | Use result.value |
| NOT_FOUND | No matching records | Handle gracefully |
| ERROR | Database error | Check result.error |
| INVALID | Invalid operation | Check parameters |

## Best Practices

1. **Always close connections** - Use context managers or finally blocks
2. **Use transactions** - For multi-step operations
3. **Handle errors** - Check DBSTATUS codes
4. **Use bulk operations** - For multiple records
5. **Filter carefully** - Avoid unintended deletions
6. **Log operations** - For debugging and audit trails

## Reference

- [Storage Layer Overview](../../components/storage/README.md)
- [Database Operations Guide](../../components/storage/database_operations.md)
- [Table Creation Guide](../../components/storage/table_creation.md)
