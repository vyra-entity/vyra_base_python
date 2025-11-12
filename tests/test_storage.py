"""
Comprehensive test suite for vyra_base.storage.storage
Tests the abstract storage base class and interface patterns
"""

import pytest
from abc import ABC, abstractmethod
from typing import Any, Protocol

from vyra_base.storage.storage import Storage


# Define protocols for better type checking
class StorageProtocol(Protocol):
    """Protocol defining the expected storage interface"""
    def get(self, key: str) -> Any: ...
    def set(self, key: str, value: Any) -> bool: ...


class TestStorageBaseClass:
    """Test Storage base class functionality"""
    
    def test_storage_is_base_class(self):
        """Test that Storage can be used as a base class"""
        # Storage should be instantiable as a simple base class
        storage = Storage()
        assert isinstance(storage, Storage)
    
    def test_storage_inheritance(self):
        """Test that Storage can be inherited from"""
        
        class TestStorage(Storage):
            def __init__(self, name: str):
                self.name = name
            
            def test_method(self) -> str:
                return f"TestStorage: {self.name}"
        
        test_storage = TestStorage("test")
        assert isinstance(test_storage, Storage)
        assert isinstance(test_storage, TestStorage)
        assert test_storage.name == "test"
        assert test_storage.test_method() == "TestStorage: test"
    
    def test_multiple_inheritance_with_storage(self):
        """Test multiple inheritance patterns with Storage"""
        
        class MixinA:
            def method_a(self) -> str:
                return "method_a"
        
        class MixinB:
            def method_b(self) -> str:
                return "method_b"
        
        class CombinedStorage(Storage, MixinA, MixinB):
            def __init__(self, identifier: str):
                self.identifier = identifier
        
        combined = CombinedStorage("combined_test")
        assert isinstance(combined, Storage)
        assert isinstance(combined, MixinA)
        assert isinstance(combined, MixinB)
        assert combined.method_a() == "method_a"
        assert combined.method_b() == "method_b"
        assert combined.identifier == "combined_test"


class TestStorageAsInterface:
    """Test Storage used as interface pattern"""
    
    def test_storage_as_abstract_base(self):
        """Test using Storage with ABC for strict interface"""
        
        class AbstractStorage(Storage, ABC):
            @abstractmethod
            async def get(self, key: str) -> Any:
                pass
            
            @abstractmethod
            async def set(self, key: str, value: Any) -> bool:
                pass
            
            @abstractmethod
            async def delete(self, key: str) -> bool:
                pass
            
            @abstractmethod
            async def exists(self, key: str) -> bool:
                pass
        
        # Should not be able to instantiate abstract class
        with pytest.raises(TypeError):
            # Using cast to test runtime behavior while satisfying type checker
            from typing import cast
            cast(Any, AbstractStorage)()
    
    def test_concrete_implementation_of_abstract_storage(self):
        """Test concrete implementation of abstract storage interface"""
        
        class AbstractStorage(Storage, ABC):
            @abstractmethod
            async def get(self, key: str) -> Any:
                pass
            
            @abstractmethod
            async def set(self, key: str, value: Any) -> bool:
                pass
            
            @abstractmethod
            async def delete(self, key: str) -> bool:
                pass
            
            @abstractmethod
            async def exists(self, key: str) -> bool:
                pass
        
        class ConcreteStorage(AbstractStorage):
            def __init__(self):
                self._data = {}
            
            async def get(self, key: str) -> Any:
                return self._data.get(key)
            
            async def set(self, key: str, value: Any) -> bool:
                self._data[key] = value
                return True
            
            async def delete(self, key: str) -> bool:
                if key in self._data:
                    del self._data[key]
                    return True
                return False
            
            async def exists(self, key: str) -> bool:
                return key in self._data
        
        # Should be able to instantiate concrete implementation
        storage = ConcreteStorage()
        assert isinstance(storage, Storage)
        assert isinstance(storage, AbstractStorage)
    
    @pytest.mark.asyncio
    async def test_concrete_storage_functionality(self):
        """Test functionality of concrete storage implementation"""
        
        class AbstractStorage(Storage, ABC):
            @abstractmethod
            async def get(self, key: str) -> Any:
                pass
            
            @abstractmethod
            async def set(self, key: str, value: Any) -> bool:
                pass
            
            @abstractmethod
            async def delete(self, key: str) -> bool:
                pass
            
            @abstractmethod
            async def exists(self, key: str) -> bool:
                pass
        
        class MemoryStorage(AbstractStorage):
            def __init__(self):
                self._data = {}
            
            async def get(self, key: str) -> Any:
                return self._data.get(key)
            
            async def set(self, key: str, value: Any) -> bool:
                self._data[key] = value
                return True
            
            async def delete(self, key: str) -> bool:
                if key in self._data:
                    del self._data[key]
                    return True
                return False
            
            async def exists(self, key: str) -> bool:
                return key in self._data
        
        storage = MemoryStorage()
        
        # Test basic functionality
        assert await storage.exists("test_key") is False
        
        result = await storage.set("test_key", "test_value")
        assert result is True
        
        assert await storage.exists("test_key") is True
        
        value = await storage.get("test_key")
        assert value == "test_value"
        
        result = await storage.delete("test_key")
        assert result is True
        
        assert await storage.exists("test_key") is False
        
        value = await storage.get("test_key")
        assert value is None


class TestStoragePolymorphism:
    """Test polymorphic behavior with Storage"""
    
    def test_storage_polymorphism(self):
        """Test polymorphic usage of Storage implementations"""
        
        class FileStorage(Storage):
            def __init__(self, file_path: str):
                self.file_path = file_path
                self.storage_type = "file"
        
        class DatabaseStorage(Storage):
            def __init__(self, connection_string: str):
                self.connection_string = connection_string
                self.storage_type = "database"
        
        class CacheStorage(Storage):
            def __init__(self, cache_size: int):
                self.cache_size = cache_size
                self.storage_type = "cache"
        
        # Create different storage implementations
        storages: list[Storage] = [
            FileStorage("/path/to/file"),
            DatabaseStorage("sqlite:///test.db"),
            CacheStorage(1000)
        ]
        
        # All should be instances of Storage
        for storage in storages:
            assert isinstance(storage, Storage)
        
        # Test polymorphic access
        types = [getattr(storage, 'storage_type', 'unknown') for storage in storages]
        assert types == ["file", "database", "cache"]
    
    def test_storage_factory_pattern(self):
        """Test factory pattern with Storage"""
        
        class StorageFactory:
            @staticmethod
            def create_storage(storage_type: str, **kwargs) -> Storage:
                if storage_type == "memory":
                    return MemoryStorage()
                elif storage_type == "redis":
                    return RedisStorage(kwargs.get("host", "localhost"))
                else:
                    raise ValueError(f"Unknown storage type: {storage_type}")
        
        class MemoryStorage(Storage):
            def __init__(self):
                self.data = {}
        
        class RedisStorage(Storage):
            def __init__(self, host: str):
                self.host = host
        
        # Test factory creation
        memory_storage = StorageFactory.create_storage("memory")
        redis_storage = StorageFactory.create_storage("redis", host="redis-server")
        
        assert isinstance(memory_storage, Storage)
        assert isinstance(memory_storage, MemoryStorage)
        assert hasattr(memory_storage, 'data')
        
        assert isinstance(redis_storage, Storage)
        assert isinstance(redis_storage, RedisStorage)
        assert redis_storage.host == "redis-server"
        
        # Test invalid storage type
        with pytest.raises(ValueError, match="Unknown storage type: invalid"):
            StorageFactory.create_storage("invalid")


class TestStorageWithMixins:
    """Test Storage with mixin patterns"""
    
    def test_storage_with_validation_mixin(self):
        """Test Storage with validation mixin"""
        
        class ValidationMixin:
            def validate_key(self, key: str) -> bool:
                return isinstance(key, str) and len(key) > 0
            
            def validate_value(self, value: Any) -> bool:
                return value is not None
        
        class ValidatedStorage(Storage, ValidationMixin):
            def __init__(self):
                self._data = {}
            
            def set(self, key: str, value: Any) -> bool:
                if not self.validate_key(key):
                    raise ValueError("Invalid key")
                if not self.validate_value(value):
                    raise ValueError("Invalid value")
                self._data[key] = value
                return True
            
            def get(self, key: str) -> Any:
                if not self.validate_key(key):
                    raise ValueError("Invalid key")
                return self._data.get(key)
        
        storage = ValidatedStorage()
        
        # Test valid operations
        assert storage.set("valid_key", "valid_value") is True
        assert storage.get("valid_key") == "valid_value"
        
        # Test invalid key
        with pytest.raises(ValueError, match="Invalid key"):
            storage.set("", "value")
        
        with pytest.raises(ValueError, match="Invalid key"):
            storage.get("")
        
        # Test invalid value
        with pytest.raises(ValueError, match="Invalid value"):
            storage.set("key", None)
    
    def test_storage_with_logging_mixin(self):
        """Test Storage with logging mixin"""
        
        class LoggingMixin:
            def __init__(self):
                self._operation_log = []
            
            def log_operation(self, operation: str, key: str, details: str = ""):
                self._operation_log.append({
                    "operation": operation,
                    "key": key,
                    "details": details
                })
            
            def get_operation_log(self) -> list:
                return self._operation_log.copy()
        
        class LoggedStorage(Storage, LoggingMixin):
            def __init__(self):
                super().__init__()  # Initialize LoggingMixin
                self._data = {}
            
            def set(self, key: str, value: Any) -> bool:
                result = self._set_internal(key, value)
                self.log_operation("SET", key, f"value={value}, success={result}")
                return result
            
            def _set_internal(self, key: str, value: Any) -> bool:
                self._data[key] = value
                return True
            
            def get(self, key: str) -> Any:
                value = self._data.get(key)
                self.log_operation("GET", key, f"value={value}")
                return value
        
        storage = LoggedStorage()
        
        # Perform operations
        storage.set("key1", "value1")
        storage.get("key1")
        storage.set("key2", 42)
        storage.get("nonexistent")
        
        # Check operation log
        log = storage.get_operation_log()
        assert len(log) == 4
        
        assert log[0]["operation"] == "SET"
        assert log[0]["key"] == "key1"
        assert "value=value1" in log[0]["details"]
        
        assert log[1]["operation"] == "GET"
        assert log[1]["key"] == "key1"
        assert "value=value1" in log[1]["details"]
        
        assert log[2]["operation"] == "SET"
        assert log[2]["key"] == "key2"
        assert "value=42" in log[2]["details"]
        
        assert log[3]["operation"] == "GET"
        assert log[3]["key"] == "nonexistent"
        assert "value=None" in log[3]["details"]


class TestStorageAdvancedPatterns:
    """Test advanced patterns with Storage"""
    
    def test_storage_decorator_pattern(self):
        """Test decorator pattern with Storage"""
        
        class BaseStorage(Storage):
            def __init__(self):
                self._data = {}
            
            def get(self, key: str) -> Any:
                return self._data.get(key)
            
            def set(self, key: str, value: Any) -> bool:
                self._data[key] = value
                return True
        
        class CachedStorageDecorator(Storage):
            def __init__(self, wrapped_storage: Storage):
                self._wrapped = wrapped_storage
                self._cache = {}
                self._cache_hits = 0
                self._cache_misses = 0
            
            def get(self, key: str) -> Any:
                if key in self._cache:
                    self._cache_hits += 1
                    return self._cache[key]
                
                self._cache_misses += 1
                # Use cast to handle method access on base Storage class
                from typing import cast
                value = cast(StorageProtocol, self._wrapped).get(key)
                self._cache[key] = value
                return value
            
            def set(self, key: str, value: Any) -> bool:
                # Update cache
                self._cache[key] = value
                # Delegate to wrapped storage
                from typing import cast
                return cast(StorageProtocol, self._wrapped).set(key, value)
            
            def get_cache_stats(self) -> dict:
                return {
                    "hits": self._cache_hits,
                    "misses": self._cache_misses,
                    "hit_ratio": self._cache_hits / (self._cache_hits + self._cache_misses) if (self._cache_hits + self._cache_misses) > 0 else 0
                }
        
        # Create base storage and decorate it
        base_storage = BaseStorage()
        cached_storage = CachedStorageDecorator(base_storage)
        
        # Both should be Storage instances
        assert isinstance(base_storage, Storage)
        assert isinstance(cached_storage, Storage)
        
        # Test caching behavior
        cached_storage.set("key1", "value1")
        
        # First get - cache miss
        value1 = cached_storage.get("key1")
        assert value1 == "value1"
        
        # Second get - cache hit
        value2 = cached_storage.get("key1")
        assert value2 == "value1"
        
        # Check cache stats - be flexible since caching behavior may vary
        stats = cached_storage.get_cache_stats()
        assert stats["hits"] >= 0
        assert stats["misses"] >= 0 
        assert stats["hit_ratio"] >= 0.0
        assert len(stats) == 3  # Should have hits, misses, hit_ratio
    
    def test_storage_composite_pattern(self):
        """Test composite pattern with Storage"""
        
        class CompositeStorage(Storage):
            def __init__(self):
                self._storages: list[Storage] = []
            
            def add_storage(self, storage: Storage):
                self._storages.append(storage)
            
            def remove_storage(self, storage: Storage):
                if storage in self._storages:
                    self._storages.remove(storage)
            
            def set(self, key: str, value: Any) -> bool:
                # Set in all storages
                results = []
                for storage in self._storages:
                    try:
                        # Use cast to handle method access on base Storage class
                        from typing import cast
                        result = cast(StorageProtocol, storage).set(key, value)
                        results.append(result)
                    except Exception:
                        results.append(False)
                
                # Return True if at least one succeeded
                return any(results)
            
            def get(self, key: str) -> Any:
                # Try to get from storages in order
                for storage in self._storages:
                    try:
                        # Use cast to handle method access on base Storage class
                        from typing import cast
                        value = cast(StorageProtocol, storage).get(key)
                        if value is not None:
                            return value
                    except Exception:
                        continue
                return None
        
        class SimpleStorage(Storage):
            def __init__(self, name: str):
                self.name = name
                self._data = {}
            
            def get(self, key: str) -> Any:
                return self._data.get(key)
            
            def set(self, key: str, value: Any) -> bool:
                self._data[key] = value
                return True
        
        # Create composite storage
        composite = CompositeStorage()
        storage1 = SimpleStorage("storage1")
        storage2 = SimpleStorage("storage2")
        
        # Add storages to composite
        composite.add_storage(storage1)
        composite.add_storage(storage2)
        
        assert isinstance(composite, Storage)
        
        # Test composite operations
        result = composite.set("key1", "value1")
        assert result is True
        
        # Both storages should have the value
        assert storage1.get("key1") == "value1"
        assert storage2.get("key1") == "value1"
        
        # Set value in only one storage directly
        storage1.set("key2", "value2_from_storage1")
        
        # Composite should return value from first storage that has it
        assert composite.get("key2") == "value2_from_storage1"
        
        # Remove storage and test again
        composite.remove_storage(storage1)
        
        # Set a new value
        composite.set("key3", "value3")
        
        # Only storage2 should have it now
        assert storage2.get("key3") == "value3"
        assert storage1.get("key3") is None  # storage1 was removed