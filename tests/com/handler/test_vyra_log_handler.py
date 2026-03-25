"""
Unit tests for VyraLogHandler.

Tests the in-memory log ring-buffer handler used by VyraEntity.
"""
import logging
import pytest

from vyra_base.com.handler.logger import VyraLogHandler


class TestVyraLogHandler:
    """Tests for VyraLogHandler."""

    def test_captures_log_records(self):
        """Test that handler captures log records."""
        handler = VyraLogHandler(capacity=10)
        logger = logging.getLogger("test_logger")
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)
        
        logger.info("Test message 1")
        logger.warning("Test message 2")
        
        recent = handler.get_recent(limit=10)
        assert len(recent) == 2
        assert recent[0]["message"] == "Test message 1"
        assert recent[0]["level"] == "INFO"
        assert recent[1]["message"] == "Test message 2"
        assert recent[1]["level"] == "WARNING"

    def test_respects_capacity_limit(self):
        """Test that buffer respects capacity limit (FIFO)."""
        handler = VyraLogHandler(capacity=3)
        logger = logging.getLogger("test_capacity")
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)
        
        for i in range(5):
            logger.info(f"Message {i}")
        
        recent = handler.get_recent(limit=10)
        assert len(recent) == 3
        # Should keep last 3 messages (2, 3, 4)
        assert recent[0]["message"] == "Message 2"
        assert recent[1]["message"] == "Message 3"
        assert recent[2]["message"] == "Message 4"

    def test_truncates_long_messages(self):
        """Test that extremely long messages are truncated."""
        max_len = 1000
        handler = VyraLogHandler(capacity=10, max_message_length=max_len)
        logger = logging.getLogger("test_truncate")
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)
        
        # Create a message that exceeds the limit
        long_message = "X" * 5000
        logger.info(long_message)
        
        recent = handler.get_recent(limit=10)
        assert len(recent) == 1
        stored_message = recent[0]["message"]
        
        # Should be truncated to max_len + suffix
        assert len(stored_message) < len(long_message)
        assert stored_message.startswith("X" * 100)  # First part preserved
        assert "[TRUNCATED:" in stored_message
        assert "chars]" in stored_message

    def test_truncation_suffix_format(self):
        """Test that truncation suffix contains correct character count."""
        max_len = 100
        handler = VyraLogHandler(capacity=10, max_message_length=max_len)
        logger = logging.getLogger("test_suffix")
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)
        
        # Create a message with known length
        message_len = 250
        long_message = "A" * message_len
        logger.info(long_message)
        
        recent = handler.get_recent(limit=10)
        stored_message = recent[0]["message"]
        
        # Should have truncation suffix with correct count
        truncated_chars = message_len - max_len
        expected_suffix = f"... [TRUNCATED: {truncated_chars} chars]"
        assert stored_message.endswith(expected_suffix)
        assert stored_message[:max_len] == "A" * max_len

    def test_get_recent_limit(self):
        """Test that get_recent respects limit parameter."""
        handler = VyraLogHandler(capacity=10)
        logger = logging.getLogger("test_limit")
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)
        
        for i in range(10):
            logger.info(f"Message {i}")
        
        # Request only 3
        recent = handler.get_recent(limit=3)
        assert len(recent) == 3
        # Should return last 3
        assert recent[0]["message"] == "Message 7"
        assert recent[1]["message"] == "Message 8"
        assert recent[2]["message"] == "Message 9"

    def test_seq_field_present(self):
        """Test that each record has a monotonic seq field."""
        handler = VyraLogHandler(capacity=10)
        logger = logging.getLogger("test_seq")
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)
        
        logger.info("Message 1")
        logger.info("Message 2")
        
        recent = handler.get_recent(limit=10)
        assert "seq" in recent[0]
        assert "seq" in recent[1]
        assert isinstance(recent[0]["seq"], float)
        assert recent[1]["seq"] > recent[0]["seq"]

    def test_timestamp_field_format(self):
        """Test that timestamp is in ISO format."""
        handler = VyraLogHandler(capacity=10)
        logger = logging.getLogger("test_timestamp")
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)
        
        logger.info("Test")
        
        recent = handler.get_recent(limit=10)
        assert "timestamp" in recent[0]
        # Should be ISO format string
        timestamp = recent[0]["timestamp"]
        assert isinstance(timestamp, str)
        assert "T" in timestamp or " " in timestamp  # ISO format contains T or space

    def test_normal_length_messages_not_truncated(self):
        """Test that messages under the limit are not modified."""
        max_len = 10000
        handler = VyraLogHandler(capacity=10, max_message_length=max_len)
        logger = logging.getLogger("test_normal")
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)
        
        normal_message = "This is a normal message"
        logger.info(normal_message)
        
        recent = handler.get_recent(limit=10)
        assert recent[0]["message"] == normal_message
        assert "[TRUNCATED:" not in recent[0]["message"]
