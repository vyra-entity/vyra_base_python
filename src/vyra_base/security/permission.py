from http import server
from xml.etree import ElementTree as ET

from vyra_base.helper.logger import Logger


class PermissionGenerator:
    """Class to handle permission.xml-related operations."""

    def __init__(self, db_session, permission_file: str, domain_id: int) -> None:
        self.db_session = db_session
        self.root: ET.Element[str] = ET.fromstring(permission_file)
        self.domain_id = domain_id

    def remove_module(self, module_id: str, permission_file: str) -> None:
        """Remove a module from a permission xml file."""
        module_element = self.root.find(f"./module[@id='{module_id}']")
        if module_element is not None:
            self.root.remove(module_element)
        else:
            raise ValueError(f"Module with id {module_id} does not exist in permission file.")

    def _set_grant_name(self):
        pass

    def _set_validity(self, from_date: str, to_date: str) -> None:
        """Set the validity period for a permission."""
        pass

    def _set_defaults(self) -> None:
        """Set default values for permissions."""
        pass


class BaseInterface:
    """Base class for all interface types with common add/remove functionality."""
    client: list[str] = []
    server: list[str] = []


    @classmethod
    def add_client(cls, name: str) -> bool:
        """Add a new client registration."""
        if name not in cls.client:
            cls.client.append(name)
            return True
        return False

    @classmethod
    def add_server(cls, name: str) -> bool:
        """Add a new server registration."""
        if name not in cls.server:
            cls.server.append(name)
            return True
        return False

    @classmethod
    def remove_client(cls, name: str) -> bool:
        """Remove a client registration."""
        if name in cls.client:
            cls.client.remove(name)
            return True
        Logger.warn(f"Registration {name} not found in {cls.__name__}.")
        return False
    
    @classmethod
    def remove_server(cls, name: str) -> bool:
        """Remove a server registration."""
        if name in cls.server:
            cls.server.remove(name)
            return True
        Logger.warn(f"Registration {name} not found in {cls.__name__}.")
        return False


# def create_base_permissions_xml() -> ET.Element:
#     root = ET.Element("permissions")
    
#     # Beispiel für Topic Permissions
#     topic = ET.SubElement(root, "topic")
#     topic_name = ET.SubElement(topic, "name")
#     topic_name.text = "/example_topic"
    
#     permissions = ET.SubElement(topic, "permissions")
    
#     publish = ET.SubElement(permissions, "publish")
#     identity_publish = ET.SubElement(publish, "identity")
#     identity_publish_name = ET.SubElement(identity_publish, "name")
#     identity_publish_name.text = "user1"
#     permission_publish = ET.SubElement(identity_publish, "permission")
#     permission_publish.text = "allow"

#     subscribe = ET.SubElement(permissions, "subscribe")
#     identity_subscribe = ET.SubElement(subscribe, "identity")
#     identity_subscribe_name = ET.SubElement(identity_subscribe, "name")
#     identity_subscribe_name.text = "user2"
#     permission_subscribe = ET.SubElement(identity_subscribe, "permission")
#     permission_subscribe.text = "deny"
    
#     # Beispiel für Service Permissions
#     service = ET.SubElement(root, "service")
#     service_name = ET.SubElement(service, "name")
#     service_name.text = "/example_service"
    
#     permissions_service = ET.SubElement(service, "permissions")
    
#     call_service = ET.SubElement(permissions_service, "call")
#     identity_service = ET.SubElement(call_service, "identity")
#     identity_service_name = ET.SubElement(identity_service, "name")
#     identity_service_name.text = "user1"
#     permission_service = ET.SubElement(identity_service, "permission")
#     permission_service.text = "allow"

#     # Baum in XML umwandeln und speichern
#     return root # ET.ElementTree(root)


class Speaker(BaseInterface):
    client: list[str] = []
    server: list[str] = []

    @classmethod
    def get_client_subs(cls):
        return [f"rq/{name}" for name in cls.client]
    
    @classmethod
    def get_server_pubs(cls):
        return [f"rq/{name}" for name in cls.client]


class Callable(BaseInterface):
    client: list[str] = []
    server: list[str] = []

    @classmethod
    def get_client_subs(cls):
        return [f"rq/{name}Request" for name in cls.client]
    
    @classmethod
    def get_client_pubs(cls):
        return [f"rr/{name}Reply" for name in cls.client]
    
    @classmethod
    def get_server_subs(cls):
        return [f"rr/{name}Request" for name in cls.server]
    
    @classmethod
    def get_server_pubs(cls):
        return [f"rq/{name}Reply" for name in cls.server]


class Job(BaseInterface):
    client: list[str] = []  # Jede Klasse braucht ihre eigene REG Liste
    server: list[str] = []

    @classmethod
    def get_client_subs(cls):
        entries = []
        for name in cls.client:
            entries.extend([
                f"rr/{name}/_action/send_goalReply",
                f"rr/{name}/_action/cancel_goalReply",
                f"rr/{name}/_action/get_resultReply",
                f"rt/{name}/_action/feedback",
                f"rt/{name}/_action/status"
            ])
        return entries
    
    @classmethod
    def get_client_pubs(cls):
        entries = []
        for name in cls.client:
            entries.extend([
                f"rq/{name}/_action/send_goalRequest",
                f"rq/{name}/_action/cancel_goalRequest",
                f"rq/{name}/_action/get_resultRequest"
            ])
        return entries
    
    @classmethod
    def get_server_subs(cls):
        entries = []
        for name in cls.server:
            entries.extend([
                f"rq/{name}/_action/send_goalRequest",
                f"rq/{name}/_action/cancel_goalRequest",
                f"rq/{name}/_action/get_resultRequest"
            ])
        return entries
    
    @classmethod
    def get_server_pubs(cls):
        entries = []
        for name in cls.server:
            entries.extend([
                f"rr/{name}/_action/send_goalReply",
                f"rr/{name}/_action/cancel_goalReply",
                f"rr/{name}/_action/get_resultReply",
                f"rt/{name}/_action/feedback",
                f"rt/{name}/_action/status"
            ])
        return entries
    

def prev_test():
    print("Add speaker_client: " + str(Speaker.add_client("my_speaker_client")))  # True
    print("Add speaker_server: " + str(Speaker.add_server("my_speaker_server")))  # True
    print("Add service_client: " + str(Callable.add_client("my_service_client")))  # True
    print("Add service_server: " + str(Callable.add_server("my_service_server")))  # True
    print("Add action_client: " + str(Job.add_client("my_action_client")))  # True
    print("Add action_server: " + str(Job.add_server("my_action_server")))  # True
    

    print("Speaker client subscriptions: " + str(Speaker.get_client_subs()))  # ["rq/my_speaker_client"]
    print("Speaker server publications: " + str(Speaker.get_server_pubs()))  # ["rq/my_speaker_client"]
    print("Callable client subscriptions: " + str(Callable.get_client_subs()))  # ["rq/my_service_clientRequest"]
    print("Callable client publications: " + str(Callable.get_client_pubs()))  # ["rr/my_service_clientReply"]
    print("Callable server subscriptions: " + str(Callable.get_server_subs()))  # ["rr/my_service_serverRequest"]
    print("Callable server publications: " + str(Callable.get_server_pubs()))  # ["rq/my_service_serverReply"]
    print("Job client subscriptions: " + str(Job.get_client_subs()))  # Alle action client subscriptions
    print("Job client publications: " + str(Job.get_client_pubs()))  # Alle action client publications
    print("Job server subscriptions: " + str(Job.get_server_subs()))  # Alle action server subscriptions
    print("Job server publications: " + str(Job.get_server_pubs()))  # Alle action server publications

    print("Remove speaker_client: " + str(Speaker.remove_client("my_speaker_client")))  # True
    print("Remove speaker_server: " + str(Speaker.remove_server("my_speaker_server")))  # True
    print("Remove service_client: " + str(Callable.remove_client("my_service_client")))  # True
    print("Remove service_server: " + str(Callable.remove_server("my_service_server")))  # True
    print("Remove action_client: " + str(Job.remove_client("my_action_client")))  # True
    print("Remove action_server: " + str(Job.remove_server("my_action_server")))  # True

    permission_root = create_base_permissions_xml()

    xml_str = ET.tostring(permission_root, 'utf-8')
    
    # Verwende minidom, um die XML zu formatieren (mit Einrückungen und Zeilenumbrüchen)
    import xml.dom.minidom
    pretty_str = xml.dom.minidom.parseString(xml_str).toprettyxml(indent="    ")

    with open("permissions.xml", "w") as f:
        f.write(pretty_str)

if __name__ == "__main__":
    prev_test()