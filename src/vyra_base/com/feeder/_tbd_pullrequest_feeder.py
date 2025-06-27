# from collections import deque

# import asyncua

# from application.backend.general.feed.feed_entry.pull_request_entry import PullRequestEntry
# from application.backend.general.feed import (
#     AbstractFeeder,
#     AbstractRemove
# )
# from application.backend.general.utilities.constant import Constant


# class PullRequestFeeder(AbstractFeeder, AbstractRemove):
#     """
#     Collection of pull request items.
#     """

#     def __init__(self):
#         self.feeds: list[asyncua.Node] = []
#         self.feed_elements: deque = deque(maxlen=Constant.MAX_LENGTH.value)

#     async def append(self, value: object) -> None:
#         """
#         Adds the passed value.

#         If the incoming value is not already in the list, the value is added
#         to the deque and passed on to the OPC UA server.

#         :param value: The value to add.
#         :type value: object
#         """
#         duplicate = [item for item in self.feed_elements if self.feed_elements.count(value) > 1]
#         if not duplicate:
#             for element in self.feeds:
#                 await element.set_value(self.feed_elements.append(value))

#     def add_opcua_handler(self, feed: asyncua.Node):
#         """
#         Add OPC UA handler to the list.

#         If the handler is not already in the list, the feed is added to it.

#         :param feed: Feed to be added to the list.
#         :type feed: asyncua.Node
#         """
#         duplicate_feed = [item for item in self.feeds if self.feeds.count(feed) > 1]
#         if not duplicate_feed:
#             self.feeds.append(feed)

#     async def remove(self, item: object) -> None:
#         """
#         Remove the passed item from the list.

#         Checks the passed object's UUID and removes the object
#         if it is in the list.

#         :param item: Object to delete.
#         :type item: object
#         """
#         item_uuid: str = item.uuid # type: ignore
#         for element in self.feed_elements:
#             if element.uuid == item_uuid:
#                 await self.feeds[0].set_value(self.feed_elements.remove(item))

#     def get_element(self) -> object:
#         """
#         Get the feed elements.

#         :return: The feed elements.
#         :rtype: object
#         """
#         return self.feed_elements

#     def get_size(self) -> int:
#         """
#         Get the length of the list.

#         :return: The number of elements in the list.
#         :rtype: int
#         """
#         return len(self.feed_elements)

#     def get_deque(self) -> deque:
#         """
#         Get the deque object.

#         :return: The deque object.
#         :rtype: deque
#         """
#         return self.feed_elements

#     async def __delitem__(self, index: int) -> None:
#         """
#         Delete item by index, overriding the magic method.

#         Iterates the list, compares the list element UUID to the element at
#         the index, removes it, and sends the new list as value to the OPC UA server.

#         :param index: Index of the element to be deleted.
#         :type index: int
#         """
#         item: PullRequestEntry = self.feed_elements[index]
#         for element in self.feed_elements:
#             if element.uuid == item.uuid:
#                 await self.feeds[0].set_value(self.feed_elements.remove([index]))

#     async def clear(self) -> None:
#         """
#         Clear all available modules in the deque.
#         """
#         for element in self.feeds:
#             await element.set_value(self.feed_elements.clear())
