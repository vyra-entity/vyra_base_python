# vyra_base.com.industrial.opcua

OPC UA Industrial Protocol Module
=================================

OPC Unified Architecture for SCADA/MES northbound communication.

**Components:**
- OpcuaClient: Client for connecting to OPC UA servers
- OpcuaServer: Server for hosting OPC UA endpoints
- Handlers: Connection, subscription, and node handlers
- Provider: VYRA integration provider
- Callable: VYRA Callable wrapper for method calls
- Speaker: VYRA Speaker wrapper for subscriptions

**Features:**
- Client/Server architecture
- Node read/write operations
- Method calls
- Subscriptions with data change notifications
- Security (authentication, encryption)

**Usage:**

.. code-block:: python

    # Client
    from vyra_base.com.industrial.opcua import OpcuaClient
    
    client = OpcuaClient(endpoint="opc.tcp://192.168.1.10:4840")
    await client.connect()
    value = await client.read_node("ns=2;i=1001")
    await client.close()
    
    # Server
    from vyra_base.com.industrial.opcua import OpcuaServer
    
    async with OpcuaServer(endpoint="opc.tcp://0.0.0.0:4840") as server:
        idx = await server.register_namespace("http://example.com")
        node = await server.add_variable(idx, "Temperature", 23.5)
