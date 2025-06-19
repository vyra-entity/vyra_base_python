import asyncua

from collections import deque

from application.backend.general.feed.feed_entry.pull_request_entry import PullRequestEntry
from application.backend.general.feed import (
    AbstractFeeder,
    AbstractRemove
)
from application.backend.general.utilities.constant import Constant


class PullRequestFeeder(AbstractFeeder, AbstractRemove):
    """Collection of pull request items."""

    def __init__(self):
        self.feeds: list[asyncua.Node] = []
        self.feed_elements: deque = deque(maxlen=Constant.MAX_LENGTH.value)

    async def append(self, value: object) -> None:
        """Adds the passed value.

        In case the incoming value is not in the list already the value gets add
        to the deque and is passed on to the opcua server.
        """
        duplicate = [item for item in self.feed_elements if self.feed_elements.count(value) > 1]
        if not duplicate:
            for element in self.feeds:
                await element.set_value(self.feed_elements.append(value))

    def add_opcua_handler(self, feed: asyncua.Node):
        """Add OPCUA Handler to list.

        In case the handler is not already in the list the feed gets added to it.
        Arguments:
            feed (asuncua.Node): Feed to be added to list.
        """
        duplicate_feed = [item for item in self.feeds if self.feeds.count(feed) > 1]
        if not duplicate_feed:
            self.feeds.append(feed)

    async def remove(self, item: object) -> None:
        """Remove the passed item from list.

        Checks the passed objects uuid and removes the object
        if it is in the list.

        Arguments:
            item (object): object to delete.
        """
        item_uuid: str = item.uuid # type: ignore
        for element in self.feed_elements:
            if element.uuid == item_uuid:
              #  for feed in self.feeds:
                await self.feeds[0].set_value(self.feed_elements.remove(item))

    def get_element(self) -> object:
        return self.feed_elements

    def get_size(self) -> int:
        """ Returns the length of the list. """
        return len(self.feed_elements)

    def get_deque(self) -> deque:
        """ Returns the deque object. """
        return self.feed_elements

    async def __delitem__(self, index: int) -> None:
        """Delete item by index overriding the magic methods.

        Iterates the list comparing the list element uuid to the element at
        the index removes it and sends the new list as value to the opcua server.

        Arguments:
            index (ind): index of the element to be deleted.
        """
        item: PullRequestEntry = self.feed_elements[index]
        for element in self.feed_elements:
            if element.uuid == item.uuid:
                #for feed in self.feeds:
                await self.feeds[0].set_value(self.feed_elements.remove([index]))

    async def clear(self) -> None:
        """Clear all available module in deque. """
        for element in self.feeds:
            await element.set_value(self.feed_elements.clear())
