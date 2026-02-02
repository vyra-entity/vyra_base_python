"""
Modbus Callable Implementation

Request/response interface for Modbus register/coil operations.
"""

import logging
from typing import Dict, Any, Optional
from vyra_base.com.core.types import VyraCallable
from vyra_base.com.core.exceptions import CallableError

logger = logging.getLogger(__name__)


class ModbusCallable(VyraCallable):
    """
    Modbus Callable Interface
    
    Supports reading/writing Modbus registers and coils via RPC-style calls.
    
    Request Format:
        {
            "operation": "read_holding"|"write_register"|"read_coils"|"write_coil",
            "address": int,
            "count": int (for reads),
            "value": int|bool (for writes),
            "unit_id": int (optional, overrides default)
        }
    
    Response Format:
        {
            "success": bool,
            "data": list (for reads) | None,
            "error": str (if success=False)
        }
    
    Example:
        # Read holding registers
        result = await callable.call({
            "operation": "read_holding",
            "address": 100,
            "count": 10
        })
        # Returns: {"success": True, "data": [1, 2, 3, ...]}
        
        # Write register
        result = await callable.call({
            "operation": "write_register",
            "address": 100,
            "value": 42
        })
        # Returns: {"success": True, "data": None}
    """
    
    def __init__(self, client: Any, unit_id: int, name: str):
        self._client = client
        self._unit_id = unit_id
        self._name = name
        self._initialized = False
    
    async def initialize(self) -> None:
        """Initialize callable (no-op for Modbus)"""
        self._initialized = True
    
    async def shutdown(self) -> None:
        """Shutdown callable (no-op for Modbus)"""
        self._initialized = False
    
    async def call(
        self,
        request: Dict[str, Any],
        timeout: float = 5.0
    ) -> Dict[str, Any]:
        """
        Execute Modbus operation
        
        Args:
            request: Operation parameters
            timeout: Not used (Modbus has its own timeout)
        
        Returns:
            Operation result
        """
        if not self._initialized:
            raise CallableError("Callable not initialized")
        
        operation = request.get("operation")
        if not operation:
            return {
                "success": False,
                "data": None,
                "error": "Missing 'operation' field"
            }
        
        try:
            # Extract common parameters
            address = request.get("address")
            unit_id = request.get("unit_id", self._unit_id)
            
            if address is None:
                return {
                    "success": False,
                    "data": None,
                    "error": "Missing 'address' field"
                }
            
            # Execute operation
            if operation == "read_holding":
                count = request.get("count", 1)
                result = self._client.read_holding_registers(
                    address=address,
                    count=count,
                    slave=unit_id
                )
                
                if result.isError():
                    return {
                        "success": False,
                        "data": None,
                        "error": str(result)
                    }
                
                return {
                    "success": True,
                    "data": result.registers,
                    "error": None
                }
            
            elif operation == "write_register":
                value = request.get("value")
                if value is None:
                    return {
                        "success": False,
                        "data": None,
                        "error": "Missing 'value' field"
                    }
                
                result = self._client.write_register(
                    address=address,
                    value=value,
                    slave=unit_id
                )
                
                if result.isError():
                    return {
                        "success": False,
                        "data": None,
                        "error": str(result)
                    }
                
                return {
                    "success": True,
                    "data": None,
                    "error": None
                }
            
            elif operation == "read_coils":
                count = request.get("count", 1)
                result = self._client.read_coils(
                    address=address,
                    count=count,
                    slave=unit_id
                )
                
                if result.isError():
                    return {
                        "success": False,
                        "data": None,
                        "error": str(result)
                    }
                
                return {
                    "success": True,
                    "data": result.bits,
                    "error": None
                }
            
            elif operation == "write_coil":
                value = request.get("value")
                if value is None:
                    return {
                        "success": False,
                        "data": None,
                        "error": "Missing 'value' field"
                    }
                
                result = self._client.write_coil(
                    address=address,
                    value=bool(value),
                    slave=unit_id
                )
                
                if result.isError():
                    return {
                        "success": False,
                        "data": None,
                        "error": str(result)
                    }
                
                return {
                    "success": True,
                    "data": None,
                    "error": None
                }
            
            elif operation == "read_input":
                count = request.get("count", 1)
                result = self._client.read_input_registers(
                    address=address,
                    count=count,
                    slave=unit_id
                )
                
                if result.isError():
                    return {
                        "success": False,
                        "data": None,
                        "error": str(result)
                    }
                
                return {
                    "success": True,
                    "data": result.registers,
                    "error": None
                }
            
            elif operation == "read_discrete":
                count = request.get("count", 1)
                result = self._client.read_discrete_inputs(
                    address=address,
                    count=count,
                    slave=unit_id
                )
                
                if result.isError():
                    return {
                        "success": False,
                        "data": None,
                        "error": str(result)
                    }
                
                return {
                    "success": True,
                    "data": result.bits,
                    "error": None
                }
            
            elif operation == "write_registers":
                values = request.get("values")
                if not values:
                    return {
                        "success": False,
                        "data": None,
                        "error": "Missing 'values' field"
                    }
                
                result = self._client.write_registers(
                    address=address,
                    values=values,
                    slave=unit_id
                )
                
                if result.isError():
                    return {
                        "success": False,
                        "data": None,
                        "error": str(result)
                    }
                
                return {
                    "success": True,
                    "data": None,
                    "error": None
                }
            
            elif operation == "write_coils":
                values = request.get("values")
                if not values:
                    return {
                        "success": False,
                        "data": None,
                        "error": "Missing 'values' field"
                    }
                
                result = self._client.write_coils(
                    address=address,
                    values=values,
                    slave=unit_id
                )
                
                if result.isError():
                    return {
                        "success": False,
                        "data": None,
                        "error": str(result)
                    }
                
                return {
                    "success": True,
                    "data": None,
                    "error": None
                }
            
            else:
                return {
                    "success": False,
                    "data": None,
                    "error": f"Unknown operation: {operation}"
                }
        
        except Exception as e:
            logger.error(f"Modbus operation failed: {e}")
            return {
                "success": False,
                "data": None,
                "error": str(e)
            }
