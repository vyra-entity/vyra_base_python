from http import server
from xml.etree import ElementTree as ET

from vyra_base.helper.logger import Logger


class PermissionGenerator:
    """Class to handle permission.xml-related operations."""

    def __init__(
            self, 
            db_session, 
            permission_file: str | ET.Element[str], 
            module_name: str,
            base_node_name: str="core",
            domain_id: int = 42) -> None:
        """
        Initialize the PermissionGenerator.
        :param db_session: Database session for accessing permissions.
        :type db_session: Any
        :param permission_file: Path to the permission file or an XML Element.
        :type permission_file: str | ET.Element[str]
        :param domain_id: Domain ID for the permissions.
        :type domain_id: int
        :raises ValueError: If the permission_file is not a valid string or XML Element.
        """
        self.db_session = db_session
        self._module_name = module_name
        self.base_node_name = base_node_name

        if isinstance(permission_file, str):
            self._root: ET.Element[str] = ET.fromstring(permission_file)
        else:
            self._root = permission_file

        Logger.info(f"Domain ID set to {domain_id}")
        self.domain_id: int = domain_id

    @property
    def permission_xml(self) -> ET.Element[str]:
        """Return the permission XML root element."""
        return self._root
    
    @permission_xml.setter
    def permission_xml(self, value: ET.Element[str]) -> None:
        """Set the permission XML root element."""
        raise ValueError(
            "Permission XML cannot be set directly. "
            "Use the constructor to initialize it.")

    def remove_module(self, module_name: str) -> None:
        """Remove all interfaces belonging to a specific module 
        from a permission xml file."""
        pass  # TODO: Implement this method

    def _set_grant_and_subject_name(self, name: str) -> None:
        """Set the grant name for a permission."""

        if not name:
            raise ValueError("Grant name cannot be empty.")
        
        grant: ET.Element[str] | None = self._root.find("./permissions/grant")
        subject_name_element: ET.Element[str] | None = self._root.find(
            "./permissions/grant/subject_name")

        if grant is None or subject_name_element is None:
            err_msg = "No grant or subject_name element found in the permission XML."
            Logger.error(err_msg)
            raise ValueError(err_msg)

        grant.set("name", f"/{self._module_name}/{self.base_node_name}")
        subject_name_element.set("name", f"CN=/{self._module_name}/{self.base_node_name}")

    def _set_validity_dates(
            self, 
            from_date: str="2025-01-01T00:00:00", 
            to_date: str="2125-01-01T00:00:00") -> None:
        """Set the validity period for a permission."""

        if not from_date or not to_date:
            raise ValueError("Both from_date and to_date must be provided.")

        if not self._check_datetime_format(from_date) or \
           not self._check_datetime_format(to_date):
            Logger.warn(f"Invalid date format: {from_date}, {to_date}. "
                         "Must be YYYY-MM-DDTHH:MM:SS.")
            raise ValueError(
                "Date format must be YYYY-MM-DDTHH:MM:SS, e.g. 2025-01-01T00:00:00.")

        validity_element: ET.Element[str] | None = self._root.find(
            "./permissions/grant/validity")
        
        if validity_element is None:
            err_msg = "No validity element found in the permission XML."
            Logger.error(err_msg)
            raise ValueError(err_msg)

        from_element = self._root.find(
            "./permissions/grant/validity/not_before_utc")
        
        if from_element is None:
            from_element = ET.SubElement(validity_element, "not_before_utc")

        from_element.text = from_date

        to_element = self._root.find(
            "./permissions/grant/validity/not_after_utc")
        
        if to_element is None:
            to_element = ET.SubElement(validity_element, "not_after_utc")

        to_element.text = to_date

    def _check_datetime_format(self, date_str: str) -> bool:
        """Check if the date string is in the correct format."""
        import re
        pattern = r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}$"
        return bool(re.match(pattern, date_str))

    def validate_permission(self) -> bool:
        """Validate the permission XML structure."""
        if not isinstance(self._root, ET.Element):
            Logger.error("Permission XML root is not a valid Element.")
            return False

        self._validate_permission_structure()
        self._validate_subject_name()


        Logger.info("Permission XML is valid.")
        return True

    def _validate_permission_structure(self) -> bool:
        # Check for required elements
        required_elements = [
            "permissions",
            "permissions/grant", 
            "permissions/grant/subject_name", 
            "permissions/grant/validity", 
            "permissions/grant/validity/not_before_utc", 
            "permissions/grant/validity/not_after_utc"
            "permissions/grant/allow_rule",
            "permissions/grant/allow_rule/domains",
            "permissions/grant/allow_rule/domains/publish",
            "permissions/grant/allow_rule/domains/subscribe",
            "permissions/grant/allow_rule/domains/publish/topics",
            "permissions/grant/allow_rule/domains/subscribe/topics",
            "permissions/grant/allow_rule/domains/request",
            "permissions/grant/allow_rule/domains/request/services",
            "permissions/grant/allow_rule/domains/request/services/service",
            "permissions/grant/allow_rule/domains/reply",
            "permissions/grant/allow_rule/domains/reply/services",
            "permissions/grant/allow_rule/domains/reply/services/service",
            "permissions/grant/default",
        ]
        for elem in required_elements:
            element_count = len(self._root.findall(f"./{elem}"))
            if not element_count:
                Logger.error(f"Missing required element: {elem}")
                return False
            if element_count > 1:
                Logger.warn(f"Multiple elements found for: {elem}. "
                            "Only the first one will be permitted.")
                return False

        default_rule: ET.Element[str] | None = self._root.find("./permissions/grant/default")
        if default_rule is None:
            Logger.error("Missing default_rule element in permission XML.")
            return False

        if default_rule.text != "DENY":
            Logger.warn("Default rule is not set to DENY. "
                        "It should be set to DENY for security reasons.")
            return False
        
        Logger.info("Permission XML structure is valid.")
        return True

    def _validate_subject_name(self) -> bool:
        """Validate the subject_name element."""
        subject_name_element: ET.Element[str] | None = self._root.find(
            "./permissions/grant/subject_name")
        
        if subject_name_element is None:
            Logger.error("Missing subject_name element in permission XML.")
            return False
        
        if not subject_name_element.text:
            Logger.error("subject_name element is empty.")
            return False
    
        if subject_name_element.text != self._module_name:
            Logger.warn(f"subject_name element does not match module name: "
                        f"{subject_name_element.text} != {self._module_name}")
            return False
        
        Logger.info("subject_name element is valid.")
        return True

    def load_permission_from_interface(self) -> None:
        """Load permissions from the interface and update the XML structure."""
        
        speaker =Speaker()
        callable_ = Callable()
        job = Job()

        publish: list = [
            "ros_discovery_info"
        ]
        subscribe: list = [
            "ros_discovery_info"
        ]


class BaseInterface:
    """Base class for all interface types with common add/remove functionality."""
    client: list[str] = []
    server: list[str] = []

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


class Speaker(BaseInterface):
    client: list[str] = []
    server: list[str] = []
    permission_templates: dict = {
        "client": ["request", "topics", "topic", "{name}"],
        "server": ["reply", "topics", "topic", "{name}"]
    }

    @classmethod
    def add_speaker_client(cls, name: str) -> bool:
        """Add a new speaker client registration."""
        return Speaker.add(cls.client, name)

    @classmethod
    def remove_speaker_client(cls, name: str) -> bool:
        """Remove a speaker client registration."""
        return Speaker.remove(cls.client, name)

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
    def add_callable_client(cls, name: str) -> bool:
        """Add a new callable client registration."""
        return Callable.add(cls.client, name)
    
    @classmethod
    def add_callable_server(cls, name: str) -> bool:
        """Add a new callable server registration."""
        return Callable.add(cls.server, name)
    
    @classmethod
    def remove_callable_client(cls, name: str) -> bool:
        """Remove a callable client registration."""
        return Callable.remove(cls.client, name)
    
    @classmethod
    def remove_callable_server(cls, name: str) -> bool:
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
    def add_job_client(cls, name: str) -> bool:
        """Add a new job client registration."""
        return Job.add(cls.client, name)

    @classmethod
    def add_job_server(cls, name: str) -> bool:
        """Add a new job server registration."""
        return Job.add(cls.server, name)

    @classmethod
    def remove_job_client(cls, name: str) -> bool:
        """Remove a job client registration."""
        return Job.remove(cls.client, name)

    @classmethod
    def remove_job_server(cls, name: str) -> bool:
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

    print("Remove speaker_client: " + str(Speaker.remove_client("my_speaker_client")))  # True
    print("Remove speaker_server: " + str(Speaker.remove_server("my_speaker_server")))  # True
    print("Remove service_client: " + str(Callable.remove_client("my_service_client")))  # True
    print("Remove service_server: " + str(Callable.remove_server("my_service_server")))  # True
    print("Remove action_client: " + str(Job.remove_client("my_action_client")))  # True
    print("Remove action_server: " + str(Job.remove_server("my_action_server")))  # True

    # permission_root = create_base_permissions_xml()

    # xml_str = ET.tostring(permission_root, 'utf-8')
    
    # # Verwende minidom, um die XML zu formatieren (mit Einrückungen und Zeilenumbrüchen)
    # import xml.dom.minidom
    # pretty_str = xml.dom.minidom.parseString(xml_str).toprettyxml(indent="    ")

    # with open("permissions.xml", "w") as f:
    #     f.write(pretty_str)

if __name__ == "__main__":
    prev_test()