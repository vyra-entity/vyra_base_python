Redis Transport Module
=====================

Provides Redis-based transport implementation with layered architecture.

Overview
--------

The Redis transport module implements the ``AbstractProtocolProvider`` interface
for Redis Pub/Sub and Request-Response patterns.

**Architecture Layers:**

- **communication/**: Core Redis functionality (RedisClient, connection handling)
- **vyra_models/**: VYRA abstractions (RedisCallable, RedisSpeaker)
- **provider.py**: Protocol provider implementation

**Features:**

- ✅ Request-Response pattern (RedisCallable)
- ✅ Pub/Sub pattern (RedisSpeaker)
- ✅ Server/Client pattern with ``is_callable`` flag
- ✅ Connection pooling
- ✅ TLS/SSL support
- ✅ Automatic reconnection
- ✅ JSON message serialization

Usage
-----

Basic Provider Setup
^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.transport.t_redis import RedisProvider, REDIS_AVAILABLE
   from vyra_base.com.core.types import ProtocolType
   
   if REDIS_AVAILABLE:
       # Create provider
       provider = RedisProvider(
           module_name="my_module",
           module_id="abc123",
           protocol=ProtocolType.REDIS
       )
       
       # Initialize with config
       await provider.initialize(config={
           "host": "redis.example.com",
           "port": 6379,
           "ssl": True,
           "ssl_ca_certs": "/path/to/ca-cert.pem",
           "ssl_certfile": "/path/to/client-cert.pem",
           "ssl_keyfile": "/path/to/client-key.pem"
       })

Create Request-Response Server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Server callback
   async def handle_request(request_data):
       # Process request
       result = {"sum": request_data["a"] + request_data["b"]}
       return result
   
   # Create server (is_callable=True)
   server = await provider.create_callable(
       name="calculation_service",
       callback=handle_request
   )

Create Request-Response Client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Create client (is_callable=False)
   client = await provider.create_callable(
       name="calculation_service",
       callback=None,  # No callback for client
       is_callable=False
   )
   
   # Make request
   request = {"a": 5, "b": 3}
   response = await client.call(request, timeout=5.0)
   print(f"Result: {response['sum']}")

Create Publisher
^^^^^^^^^^^^^^^^

.. code-block:: python

   # Create publisher (is_publisher=True)
   publisher = await provider.create_speaker(
       name="status_channel",
       is_publisher=True
   )
   
   # Publish message
   await publisher.shout({"status": "running", "progress": 75})

Create Subscriber
^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Subscriber callback
   def on_message(msg_data):
       print(f"Received: {msg_data}")
   
   # Create subscriber (is_publisher=False)
   subscriber = await provider.create_speaker(
       name="status_channel",
       is_publisher=False
   )
   
   # Start listening
   await subscriber.listen(on_message)

Configuration
-------------

Connection Configuration
^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   config = {
       "host": "redis.example.com",
       "port": 6379,
       "db": 0,
       "password": "secret",
       "ssl": True,
       "ssl_ca_certs": "/path/to/ca-cert.pem",
       "ssl_certfile": "/path/to/client-cert.pem",
       "ssl_keyfile": "/path/to/client-key.pem",
       "ssl_cert_reqs": "required",  # 'required', 'optional', 'none'
       "socket_timeout": 5.0,
       "socket_connect_timeout": 5.0,
       "retry_on_timeout": True,
       "max_connections": 50
   }
   
   await provider.initialize(config=config)

Connection Pooling
^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from redis.connection import ConnectionPool
   
   pool = ConnectionPool(
       host="redis.example.com",
       port=6379,
       max_connections=100,
       decode_responses=True
   )
   
   # Use pool in provider
   provider = RedisProvider(connection_pool=pool)

Server/Client Flags
^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 30 20 50

   * - Interface Type
     - Flag
     - Behavior
   * - Request-Response Server
     - ``is_callable=True``
     - Responds to requests (callback required)
   * - Request-Response Client
     - ``is_callable=False``
     - Makes requests (no callback)
   * - Publisher
     - ``is_publisher=True``
     - Publishes messages
   * - Subscriber
     - ``is_publisher=False``
     - Receives messages (callback required)

Advanced Usage
--------------

Custom Message Serialization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   import json
   import msgpack
   
   # Use MessagePack for binary serialization
   class MsgPackProvider(RedisProvider):
       def serialize(self, data):
           return msgpack.packb(data)
       
       def deserialize(self, data):
           return msgpack.unpackb(data, raw=False)

Pattern Subscriptions
^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Subscribe to pattern (e.g., "sensor.*")
   subscriber = await provider.create_speaker(
       name="sensor.*",  # Pattern with wildcard
       is_publisher=False
   )
   
   def on_sensor_data(msg_data, channel):
       print(f"From {channel}: {msg_data}")
   
   await subscriber.listen(on_sensor_data)

API Reference
-------------

.. automodule:: vyra_base.com.transport.t_redis.provider
   :members:
   :undoc-members:
   :show-inheritance:

.. automodule:: vyra_base.com.transport.t_redis.vyra_models
   :members:
   :undoc-members:
   :show-inheritance:

Dependencies
------------

- ``redis`` (Redis Python client)
- ``hiredis`` (Optional, for faster parsing)

See Also
--------

- :doc:`../core/factory` - InterfaceFactory for protocol-agnostic usage
- :doc:`ros2` - ROS2 transport provider
- :doc:`uds` - Unix Domain Socket transport provider
