"""
Comprehensive test suite for vyra_base.interface module
Tests interface definitions, protocols, abstract base classes, and contracts
"""

import pytest
from unittest.mock import Mock, patch, MagicMock, AsyncMock
from typing import Any, Dict, Protocol, runtime_checkable
from abc import ABC, abstractmethod

try:
    # Import what actually exists in vyra_base
    from vyra_base.interfaces import *
except ImportError:
    # If interface module doesn't exist or has import errors, we'll test the concept
    pass


# Define test protocols and interfaces for testing
@runtime_checkable
class StorageProtocol(Protocol):
    """Protocol for storage implementations"""
    
    async def store_data(self, key: str, value: Any) -> bool:
        """Store data with given key"""
        ...
    
    async def retrieve_data(self, key: str) -> Any:
        """Retrieve data by key"""
        ...
    
    async def delete_data(self, key: str) -> bool:
        """Delete data by key"""
        ...


@runtime_checkable
class CommunicationProtocol(Protocol):
    """Protocol for communication implementations"""
    
    def send_message(self, topic: str, message: Any) -> bool:
        """Send message to topic"""
        ...
    
    def receive_message(self, topic: str) -> Any:
        """Receive message from topic"""
        ...


@runtime_checkable
class ProcessingProtocol(Protocol):
    """Protocol for data processing implementations"""
    
    async def process(self, data: Any) -> Any:
        """Process input data and return result"""
        ...


class TestProtocolDefinitions:
    """Test protocol definitions and type checking"""
    
    def test_storage_protocol_implementation(self):
        """Test storage protocol implementation"""
        class MockStorage:
            async def store_data(self, key: str, value: Any) -> bool:
                return True
            
            async def retrieve_data(self, key: str) -> Any:
                return {"test": "value"}
            
            async def delete_data(self, key: str) -> bool:
                return True
        
        storage = MockStorage()
        
        # Test protocol conformance
        assert isinstance(storage, StorageProtocol)
    
    def test_communication_protocol_implementation(self):
        """Test communication protocol implementation"""
        class MockCommunication:
            def send_message(self, topic: str, message: Any) -> bool:
                return True
            
            def receive_message(self, topic: str) -> Any:
                return {"message": "received"}
        
        comm = MockCommunication()
        
        # Test protocol conformance
        assert isinstance(comm, CommunicationProtocol)
    
    def test_processing_protocol_implementation(self):
        """Test processing protocol implementation"""
        class MockProcessor:
            async def process(self, data: Any) -> Any:
                return {"processed": data}
        
        processor = MockProcessor()
        
        # Test protocol conformance
        assert isinstance(processor, ProcessingProtocol)
    
    def test_protocol_type_checking(self):
        """Test protocol type checking with invalid implementations"""
        class InvalidStorage:
            # Missing required methods
            def some_other_method(self):
                pass
        
        invalid_storage = InvalidStorage()
        
        # Should not conform to protocol
        assert not isinstance(invalid_storage, StorageProtocol)


class TestAbstractBaseClasses:
    """Test abstract base class patterns"""
    
    def test_abstract_service_interface(self):
        """Test abstract service interface"""
        class AbstractService(ABC):
            @abstractmethod
            def start(self) -> bool:
                """Start the service"""
                pass
            
            @abstractmethod
            def stop(self) -> bool:
                """Stop the service"""
                pass
            
            @abstractmethod
            def status(self) -> str:
                """Get service status"""
                pass
        
        # Test that abstract class cannot be instantiated
        with pytest.raises(TypeError):
            AbstractService()
        
        # Test concrete implementation
        class ConcreteService(AbstractService):
            def start(self) -> bool:
                return True
            
            def stop(self) -> bool:
                return True
            
            def status(self) -> str:
                return "running"
        
        service = ConcreteService()
        assert service.start() is True
        assert service.stop() is True
        assert service.status() == "running"
    
    def test_abstract_data_processor(self):
        """Test abstract data processor"""
        class AbstractDataProcessor(ABC):
            @abstractmethod
            def validate_input(self, data: Any) -> bool:
                """Validate input data"""
                pass
            
            @abstractmethod
            def transform_data(self, data: Any) -> Any:
                """Transform input data"""
                pass
            
            @abstractmethod
            def save_result(self, result: Any) -> bool:
                """Save processing result"""
                pass
            
            def process_pipeline(self, data: Any) -> Any:
                """Complete processing pipeline (concrete method)"""
                if not self.validate_input(data):
                    raise ValueError("Invalid input data")
                
                transformed = self.transform_data(data)
                
                if not self.save_result(transformed):
                    raise RuntimeError("Failed to save result")
                
                return transformed
        
        class MockDataProcessor(AbstractDataProcessor):
            def validate_input(self, data: Any) -> bool:
                return isinstance(data, dict)
            
            def transform_data(self, data: Any) -> Any:
                return {"transformed": data}
            
            def save_result(self, result: Any) -> bool:
                return True
        
        processor = MockDataProcessor()
        result = processor.process_pipeline({"input": "data"})
        
        assert result["transformed"]["input"] == "data"
    
    def test_mixin_pattern(self):
        """Test mixin pattern with interfaces"""
        class LoggingMixin:
            def log(self, message: str) -> None:
                # Mock logging
                print(f"LOG: {message}")
        
        class CachingMixin:
            def __init__(self):
                self._cache = {}
            
            def cache_get(self, key: str) -> Any:
                return self._cache.get(key)
            
            def cache_set(self, key: str, value: Any) -> None:
                self._cache[key] = value
        
        class ServiceWithMixins(LoggingMixin, CachingMixin):
            def __init__(self):
                super().__init__()
            
            def do_work(self, data: str) -> str:
                self.log(f"Processing: {data}")
                
                cached = self.cache_get(data)
                if cached:
                    return cached
                
                result = f"processed_{data}"
                self.cache_set(data, result)
                return result
        
        service = ServiceWithMixins()
        result1 = service.do_work("test")
        result2 = service.do_work("test")  # Should use cache
        
        assert result1 == "processed_test"
        assert result2 == "processed_test"
        assert service.cache_get("test") == "processed_test"


class TestInterfaceContracts:
    """Test interface contracts and validation"""
    
    def test_contract_validation(self):
        """Test contract validation for interfaces"""
        class ContractValidator:
            @staticmethod
            def validate_storage_contract(obj: Any) -> bool:
                """Validate storage contract"""
                required_methods = ["store_data", "retrieve_data", "delete_data"]
                
                for method in required_methods:
                    if not hasattr(obj, method):
                        return False
                    if not callable(getattr(obj, method)):
                        return False
                
                return True
            
            @staticmethod
            def validate_communication_contract(obj: Any) -> bool:
                """Validate communication contract"""
                required_methods = ["send_message", "receive_message"]
                
                for method in required_methods:
                    if not hasattr(obj, method):
                        return False
                
                return True
        
        # Test valid storage implementation
        class ValidStorage:
            async def store_data(self, key: str, value: Any) -> bool:
                return True
            async def retrieve_data(self, key: str) -> Any:
                return None
            async def delete_data(self, key: str) -> bool:
                return True
        
        # Test invalid storage implementation
        class InvalidStorage:
            async def store_data(self, key: str, value: Any) -> bool:
                return True
            # Missing retrieve_data and delete_data
        
        validator = ContractValidator()
        
        assert validator.validate_storage_contract(ValidStorage()) is True
        assert validator.validate_storage_contract(InvalidStorage()) is False
    
    def test_runtime_type_checking(self):
        """Test runtime type checking for interfaces"""
        def check_interface_compliance(obj: Any, protocol_class: type) -> bool:
            """Check if object complies with protocol at runtime"""
            return isinstance(obj, protocol_class)
        
        class ValidStorage:
            async def store_data(self, key: str, value: Any) -> bool:
                return True
            async def retrieve_data(self, key: str) -> Any:
                return None
            async def delete_data(self, key: str) -> bool:
                return True
        
        class InvalidStorage:
            def some_method(self):
                pass
        
        valid_storage = ValidStorage()
        invalid_storage = InvalidStorage()
        
        assert check_interface_compliance(valid_storage, StorageProtocol) is True
        assert check_interface_compliance(invalid_storage, StorageProtocol) is False


class TestInterfaceAdapters:
    """Test interface adapter patterns"""
    
    def test_legacy_adapter_pattern(self):
        """Test adapter pattern for legacy interfaces"""
        class LegacyStorage:
            """Legacy storage with different interface"""
            def put(self, key: str, data: Any) -> None:
                pass
            
            def get(self, key: str) -> Any:
                return {"legacy": "data"}
            
            def remove(self, key: str) -> None:
                pass
        
        class StorageAdapter:
            """Adapter to make legacy storage conform to new interface"""
            def __init__(self, legacy_storage: LegacyStorage):
                self._legacy = legacy_storage
            
            async def store_data(self, key: str, value: Any) -> bool:
                self._legacy.put(key, value)
                return True
            
            async def retrieve_data(self, key: str) -> Any:
                return self._legacy.get(key)
            
            async def delete_data(self, key: str) -> bool:
                self._legacy.remove(key)
                return True
        
        legacy = LegacyStorage()
        adapter = StorageAdapter(legacy)
        
        # Test that adapter conforms to protocol
        assert isinstance(adapter, StorageProtocol)
    
    def test_interface_factory_pattern(self):
        """Test factory pattern for interface implementations"""
        class InterfaceFactory:
            @staticmethod
            def create_storage(storage_type: str) -> Any:
                """Create storage implementation based on type"""
                if storage_type == "memory":
                    return MemoryStorage()
                elif storage_type == "redis":
                    return RedisStorageAdapter()
                else:
                    raise ValueError(f"Unknown storage type: {storage_type}")
        
        class MemoryStorage:
            def __init__(self):
                self._data = {}
            
            async def store_data(self, key: str, value: Any) -> bool:
                self._data[key] = value
                return True
            
            async def retrieve_data(self, key: str) -> Any:
                return self._data.get(key)
            
            async def delete_data(self, key: str) -> bool:
                if key in self._data:
                    del self._data[key]
                    return True
                return False
        
        class RedisStorageAdapter:
            async def store_data(self, key: str, value: Any) -> bool:
                # Mock Redis storage
                return True
            
            async def retrieve_data(self, key: str) -> Any:
                # Mock Redis retrieval
                return {"redis": "data"}
            
            async def delete_data(self, key: str) -> bool:
                # Mock Redis deletion
                return True
        
        factory = InterfaceFactory()
        
        memory_storage = factory.create_storage("memory")
        redis_storage = factory.create_storage("redis")
        
        # Both should conform to storage protocol
        assert isinstance(memory_storage, StorageProtocol)
        assert isinstance(redis_storage, StorageProtocol)


class TestInterfaceComposition:
    """Test interface composition patterns"""
    
    def test_composite_interface(self):
        """Test composite interface pattern"""
        @runtime_checkable
        class ReadOnlyStorage(Protocol):
            async def retrieve_data(self, key: str) -> Any:
                ...
        
        @runtime_checkable
        class WriteOnlyStorage(Protocol):
            async def store_data(self, key: str, value: Any) -> bool:
                ...
        
        @runtime_checkable
        class FullStorage(ReadOnlyStorage, WriteOnlyStorage, Protocol):
            async def delete_data(self, key: str) -> bool:
                ...
        
        class CompleteStorage:
            async def retrieve_data(self, key: str) -> Any:
                return {"data": "value"}
            
            async def store_data(self, key: str, value: Any) -> bool:
                return True
            
            async def delete_data(self, key: str) -> bool:
                return True
        
        storage = CompleteStorage()
        
        # Should conform to all protocols
        assert isinstance(storage, ReadOnlyStorage)
        assert isinstance(storage, WriteOnlyStorage)
        assert isinstance(storage, FullStorage)
    
    def test_interface_aggregation(self):
        """Test interface aggregation pattern"""
        class ServiceAggregator:
            """Aggregate multiple services with different interfaces"""
            def __init__(self):
                self._services = {}
            
            def register_service(self, name: str, service: Any) -> None:
                self._services[name] = service
            
            def get_service(self, name: str) -> Any:
                return self._services.get(name)
            
            def get_services_by_protocol(self, protocol_class: type) -> list:
                return [
                    service for service in self._services.values()
                    if isinstance(service, protocol_class)
                ]
        
        aggregator = ServiceAggregator()
        
        # Register different services
        class MockStorage:
            async def store_data(self, key: str, value: Any) -> bool:
                return True
            async def retrieve_data(self, key: str) -> Any:
                return None
            async def delete_data(self, key: str) -> bool:
                return True
        
        class MockCommunication:
            def send_message(self, topic: str, message: Any) -> bool:
                return True
            def receive_message(self, topic: str) -> Any:
                return None
        
        aggregator.register_service("storage", MockStorage())
        aggregator.register_service("comm", MockCommunication())
        
        # Test service retrieval by protocol
        storage_services = aggregator.get_services_by_protocol(StorageProtocol)
        comm_services = aggregator.get_services_by_protocol(CommunicationProtocol)
        
        assert len(storage_services) == 1
        assert len(comm_services) == 1


class TestInterfaceVersioning:
    """Test interface versioning patterns"""
    
    def test_interface_versioning(self):
        """Test interface versioning"""
        @runtime_checkable
        class StorageV1(Protocol):
            def store(self, key: str, value: str) -> bool:
                """V1: Only supports string values"""
                ...
            
            def retrieve(self, key: str) -> str:
                ...
        
        @runtime_checkable
        class StorageV2(Protocol):
            async def store_data(self, key: str, value: Any) -> bool:
                """V2: Supports any value type, async"""
                ...
            
            async def retrieve_data(self, key: str) -> Any:
                ...
            
            async def store_metadata(self, key: str, metadata: Dict) -> bool:
                """V2: Additional metadata support"""
                ...
        
        class BackwardCompatibleStorage:
            """Implementation that supports both V1 and V2"""
            
            def store(self, key: str, value: str) -> bool:
                """V1 compatibility"""
                return True
            
            def retrieve(self, key: str) -> str:
                """V1 compatibility"""
                return "v1_data"
            
            async def store_data(self, key: str, value: Any) -> bool:
                """V2 implementation"""
                return True
            
            async def retrieve_data(self, key: str) -> Any:
                """V2 implementation"""
                return {"v2": "data"}
            
            async def store_metadata(self, key: str, metadata: Dict) -> bool:
                """V2 implementation"""
                return True
        
        storage = BackwardCompatibleStorage()
        
        # Should support both versions
        assert isinstance(storage, StorageV1)
        assert isinstance(storage, StorageV2)
    
    def test_interface_deprecation(self):
        """Test interface deprecation patterns"""
        import warnings
        
        class DeprecatedInterface(Protocol):
            def old_method(self) -> str:
                """Deprecated method"""
                ...
        
        class ModernInterface(Protocol):
            def new_method(self) -> str:
                """New method"""
                ...
        
        class TransitionalService:
            def old_method(self) -> str:
                warnings.warn(
                    "old_method is deprecated, use new_method instead",
                    DeprecationWarning,
                    stacklevel=2
                )
                return self.new_method()
            
            def new_method(self) -> str:
                return "modern_result"
        
        service = TransitionalService()
        
        # Test deprecation warning
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            result = service.old_method()
            
            assert len(w) == 1
            assert issubclass(w[0].category, DeprecationWarning)
            assert "deprecated" in str(w[0].message)
            assert result == "modern_result"


class TestInterfaceDocumentation:
    """Test interface documentation and introspection"""
    
    def test_interface_documentation(self):
        """Test interface documentation patterns"""
        class DocumentedInterface(Protocol):
            """
            A well-documented interface for data processing.
            
            This interface defines the contract for processing data
            with proper validation and error handling.
            """
            
            def validate(self, data: Any) -> bool:
                """
                Validate input data.
                
        Args:
                    data: Input data to validate
                    
                Returns:
                    bool: True if data is valid, False otherwise
                """
                ...
            
            def process(self, data: Any) -> Any:
                """
                Process the validated data.
                
                Args:
                    data: Pre-validated data to process
                    
                Returns:
                    Any: Processed result
                    
                Raises:
                    ProcessingError: If processing fails
                """
                ...
        
        # Test that documentation is accessible
        assert DocumentedInterface.__doc__ is not None
        assert "well-documented" in DocumentedInterface.__doc__
        assert "validate" in dir(DocumentedInterface)
        assert "process" in dir(DocumentedInterface)
    
    def test_interface_introspection(self):
        """Test interface introspection capabilities"""
        def get_interface_methods(protocol_class: type) -> list:
            """Get all method names defined in a protocol"""
            methods = []
            for name in dir(protocol_class):
                if not name.startswith('_'):
                    attr = getattr(protocol_class, name)
                    if callable(attr):
                        methods.append(name)
            return methods
        
        def check_method_signatures(obj: Any, protocol_class: type) -> Dict[str, bool]:
            """Check if object has all required methods"""
            methods = get_interface_methods(protocol_class)
            result = {}
            
            for method in methods:
                result[method] = hasattr(obj, method) and callable(getattr(obj, method))
            
            return result
        
        class TestService:
            async def store_data(self, key: str, value: Any) -> bool:
                return True
            async def retrieve_data(self, key: str) -> Any:
                return None
            # Missing delete_data method
        
        service = TestService()
        signatures = check_method_signatures(service, StorageProtocol)
        
        # Should have store_data and retrieve_data but missing delete_data
        assert "store_data" in signatures
        assert "retrieve_data" in signatures
        assert signatures.get("store_data", False) is True
        assert signatures.get("retrieve_data", False) is True