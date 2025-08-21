from http import server
from xml.etree import ElementTree as ET

from vyra_base.helper.logger import Logger


class BaseInterface:
    """Base class for all interface types with common add/remove functionality."""
    client: list[str] = []
    server: list[str] = []
    permission_templates: dict = {
        "client": [],
        "server": []
    }

    @classmethod
    def add(cls, list_ref: list[str], name: str) -> bool:
        if name not in list_ref:
            list_ref.append(name)
            return True
        return False

    @classmethod
    def remove(cls, list_ref: list[str], name: str) -> bool:
        if name in list_ref:
            list_ref.remove(name)
            return True
        Logger.warn(f"Registration {name} not found in {cls.__name__}.")
        return False

    @classmethod
    def add_client(cls, name: str) -> bool:
        """Add a new client registration."""
        return BaseInterface.add(cls.client, name)

    @classmethod
    def add_server(cls, name: str) -> bool:
        """Add a new server registration."""
        return BaseInterface.add(cls.server, name)

    @classmethod
    def remove_client(cls, name: str) -> bool:
        """Remove a client registration."""
        return BaseInterface.remove(cls.client, name)

    @classmethod
    def remove_server(cls, name: str) -> bool:
        """Remove a server registration."""
        return BaseInterface.remove(cls.server, name)
    
    @classmethod
    def clear_all(cls) -> None:
        """Clear all client and server registrations."""
        cls.client.clear()
        cls.server.clear()
        Logger.info(f"Cleared all registrations for {cls.__name__}.")
    
    @classmethod
    def clear_client(cls) -> None:
        """Clear all client registrations."""
        cls.client.clear()
        Logger.info(f"Cleared all client registrations for {cls.__name__}.")

    @classmethod
    def clear_server(cls) -> None:
        """Clear all server registrations."""
        cls.server.clear()
        Logger.info(f"Cleared all server registrations for {cls.__name__}.")

    
    @classmethod
    def get_all_permissions(cls) -> list:
        """Get the client permissions based on the permission templates."""
        s_template = cls.permission_templates.get("server", [])
        c_template = cls.permission_templates.get("client", [])

        permissions = []

        for name in cls.client:
            for entry in s_template:
                permissions.append([n.replace("{name}", name) for n in entry])
        
        for name in cls.server:
            for entry in c_template:
                permissions.append([n.replace("{name}", name) for n in entry])
        
        return permissions


class Speaker(BaseInterface):
    client: list[str] = []
    server: list[str] = []
    permission_templates: dict = {
        "client": [
            ["request", "topics", "topic", "{name}"]
        ],
        "server": [
            ["reply", "topics", "topic", "{name}"]
        ]
    }

    @classmethod
    def add_client(cls, name: str) -> bool:
        """Add a new speaker client registration."""
        return Speaker.add(cls.client, name)

    @classmethod
    def add_server(cls, name: str) -> bool:
        """Add a new speaker server registration."""
        return Speaker.add(cls.server, name)

    @classmethod
    def remove_client(cls, name: str) -> bool:
        """Remove a speaker client registration."""
        return Speaker.remove(cls.client, name)

    @classmethod
    def remove_server(cls, name: str) -> bool:
        """Remove a speaker server registration."""
        return Speaker.remove(cls.server, name)


class Callable(BaseInterface):
    client: list[str] = []
    server: list[str] = []
    permission_templates: dict = {
        "client": [
            ["request", "services", "service", "{name}"]
        ],
        "server": [
            ["reply", "services", "service", "{name}"]
        ]
    }

    @classmethod
    def add_client(cls, name: str) -> bool:
        """Add a new callable client registration."""
        return Callable.add(cls.client, name)
    
    @classmethod
    def add_server(cls, name: str) -> bool:
        """Add a new callable server registration."""
        return Callable.add(cls.server, name)
    
    @classmethod
    def remove_client(cls, name: str) -> bool:
        """Remove a callable client registration."""
        return Callable.remove(cls.client, name)
    
    @classmethod
    def remove_server(cls, name: str) -> bool:
        """Remove a callable server registration."""
        return Callable.remove(cls.server, name)


class Job(BaseInterface):
    client: list[str] = []  # Jede Klasse braucht ihre eigene REG Liste
    server: list[str] = []
    permission_templates: dict = {
        "client": [
            ["request", "services", "service", "{name}/_action/send_goal"],
            ["request", "services", "service", "{name}/_action/get_result"],
            ["request", "services", "service", "{name}/_action/cancel"],
            ["subscribe", "topics", "topic", "{name}/_action/feedback"],
            ["subscribe", "topics", "topic", "{name}/_action/status"]

        ],
        "server": [
            ["reply", "services", "service", "{name}/_action/send_goal"],
            ["reply", "services", "services", "{name}/_action/get_result"],
            ["reply", "services", "services", "{name}/_action/cancel"],
            ["publish", "topics", "topic", "{name}/_action/feedback"],
            ["publish", "topics", "topic", "{name}/_action/status"],
        ]
    }
    @classmethod
    def add_client(cls, name: str) -> bool:
        """Add a new job client registration."""
        return Job.add(cls.client, name)

    @classmethod
    def add_server(cls, name: str) -> bool:
        """Add a new job server registration."""
        return Job.add(cls.server, name)

    @classmethod
    def remove_client(cls, name: str) -> bool:
        """Remove a job client registration."""
        return Job.remove(cls.client, name)

    @classmethod
    def remove_server(cls, name: str) -> bool:
        """Remove a job server registration."""
        return Job.remove(cls.server, name)


def prev_test():
    print("Add speaker_client: " + str(Speaker.add_client("my_speaker_client")))  # True
    print("Add speaker_server: " + str(Speaker.add_server("my_speaker_server")))  # True
    print("Add service_client: " + str(Callable.add_client("my_service_client")))  # True
    print("Add service_server: " + str(Callable.add_server("my_service_server")))  # True
    print("Add action_client: " + str(Job.add_client("my_action_client")))  # True
    print("Add action_server: " + str(Job.add_server("my_action_server")))  # True
    

    print("Speaker client subscriptions: " + str(Speaker.client))  # ["rq/my_speaker_client"]
    print("Speaker server publications: " + str(Speaker.server))  # ["rq/my_speaker_client"]
    print("Callable client subscriptions: " + str(Callable.client))  # ["rq/my_service_clientRequest"]
    print("Callable client publications: " + str(Callable.client))  # ["rr/my_service_clientReply"]
    print("Callable server subscriptions: " + str(Callable.server))  # ["rr/my_service_serverRequest"]
    print("Callable server publications: " + str(Callable.server))  # ["rq/my_service_serverReply"]
    print("Job client subscriptions: " + str(Job.client))  # Alle action client subscriptions
    print("Job client publications: " + str(Job.client))  # Alle action client publications
    print("Job server subscriptions: " + str(Job.server))  # Alle action server subscriptions
    print("Job server publications: " + str(Job.server))  # Alle action server publications

    # print("Remove speaker_client: " + str(Speaker.remove_client("my_speaker_client")))  # True
    # print("Remove speaker_server: " + str(Speaker.remove_server("my_speaker_server")))  # True
    # print("Remove service_client: " + str(Callable.remove_client("my_service_client")))  # True
    # print("Remove service_server: " + str(Callable.remove_server("my_service_server")))  # True
    # print("Remove action_client: " + str(Job.remove_client("my_action_client")))  # True
    # print("Remove action_server: " + str(Job.remove_server("my_action_server")))  # True

    print("\nAll speaker permissions:")
    print(Speaker.get_all_permissions())
    print("\nAll callable permissions:")
    print(Callable.get_all_permissions())
    print("\nAll job permissions:")
    print(Job.get_all_permissions())

    # permission_root = create_base_permissions_xml()

    # xml_str = ET.tostring(permission_root, 'utf-8')
    
    # # Verwende minidom, um die XML zu formatieren (mit Einrückungen und Zeilenumbrüchen)
    # import xml.dom.minidom
    # pretty_str = xml.dom.minidom.parseString(xml_str).toprettyxml(indent="    ")

    # with open("permissions.xml", "w") as f:
    #     f.write(pretty_str)

if __name__ == "__main__":
    prev_test()