ErrorFeeder
===========

Collection of the error messages.

**Docstring:**

   """
   Collection of the error messages.

   :param type: The ros2-msg type for the feeder.
   :type type: Any
   :param node: The VyraNode instance associated with this feeder (ROS2 Node).
   :type node: VyraNode
   :param module_config: The module configuration entry.
   :type module_config: ModuleEntry
   :param loggingOn: Flag to enable or disable logging next to feeding. Defaults to False.
   :type loggingOn: bool, Optional
   :raises FeederException: If the VyraSpeaker cannot be created with the given type.
   """

**Class:**
- ErrorFeeder
