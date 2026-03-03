"""
VYRA Feeder package — multi-protocol data publishers.

Available feeders:
- :class:`~vyra_base.com.feeder.state_feeder.StateFeeder`
- :class:`~vyra_base.com.feeder.news_feeder.NewsFeeder`
- :class:`~vyra_base.com.feeder.error_feeder.ErrorFeeder`
- :class:`~vyra_base.com.feeder.custom_feeder.CustomBaseFeeder`

Protocol resolution:
- :class:`~vyra_base.com.feeder.config_resolver.FeederConfigResolver`
- :class:`~vyra_base.com.feeder.config_resolver.FeederResolverResult`

Registry:
- :class:`~vyra_base.com.feeder.registry.FeederRegistry`
- :func:`~vyra_base.com.feeder.registry.register_feeder`

Interfaces:
- :class:`~vyra_base.com.feeder.interfaces.IFeeder`
"""

from vyra_base.com.feeder.config_resolver import FeederConfigResolver, FeederResolverResult
from vyra_base.com.feeder.interfaces import IFeeder
from vyra_base.com.feeder.custom_feeder import CustomBaseFeeder
from vyra_base.com.feeder.registry import FeederRegistry, register_feeder
from vyra_base.com.feeder.tracking import FeedConditionRegistry, FeedDebouncer, feed_tracker

__all__ = [
    "FeederConfigResolver",
    "FeederResolverResult",
    "IFeeder",
    "CustomBaseFeeder",
    "FeederRegistry",
    "register_feeder",
    "feed_tracker",
    "FeedConditionRegistry",
    "FeedDebouncer",
]
