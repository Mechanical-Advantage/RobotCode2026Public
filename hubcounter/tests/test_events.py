"""Tests for hub_counter.core.events.EventBus."""

import asyncio

import pytest

from hub_counter.core.events import EventBus, EventType


class TestEventBusSubscribeEmit:
    def test_subscribe_and_emit_async_sync_handler(self):
        bus = EventBus()
        received = []

        def handler(payload):
            received.append(payload)

        bus.subscribe(EventType.COUNT_CHANGED, handler)
        asyncio.run(bus.emit_async(EventType.COUNT_CHANGED, "data"))

        assert received == ["data"]

    def test_subscribe_and_emit_async_async_handler(self):
        bus = EventBus()
        received = []

        async def handler(payload):
            received.append(payload)

        bus.subscribe(EventType.COUNT_CHANGED, handler)
        asyncio.run(bus.emit_async(EventType.COUNT_CHANGED, 42))

        assert received == [42]

    def test_multiple_handlers_all_called(self):
        bus = EventBus()
        calls = []

        bus.subscribe(EventType.BALL_DETECTED, lambda p: calls.append(("a", p)))
        bus.subscribe(EventType.BALL_DETECTED, lambda p: calls.append(("b", p)))

        asyncio.run(bus.emit_async(EventType.BALL_DETECTED, {"channel": 1}))

        assert len(calls) == 2
        assert ("a", {"channel": 1}) in calls
        assert ("b", {"channel": 1}) in calls

    def test_unsubscribe_removes_handler(self):
        bus = EventBus()
        calls = []

        def handler(payload):
            calls.append(payload)

        bus.subscribe(EventType.NT_CONNECTED, handler)
        bus.unsubscribe(EventType.NT_CONNECTED, handler)
        asyncio.run(bus.emit_async(EventType.NT_CONNECTED, True))

        assert calls == []

    def test_unsubscribe_nonexistent_handler_is_noop(self):
        bus = EventBus()
        # Should not raise
        bus.unsubscribe(EventType.NT_CONNECTED, lambda p: None)

    def test_emit_with_no_subscribers_is_noop(self):
        bus = EventBus()
        # Should not raise
        asyncio.run(bus.emit_async(EventType.CONFIG_UPDATED, None))

    def test_emit_different_event_types_isolated(self):
        bus = EventBus()
        ball_calls = []
        nt_calls = []

        bus.subscribe(EventType.BALL_DETECTED, lambda p: ball_calls.append(p))
        bus.subscribe(EventType.NT_CONNECTED, lambda p: nt_calls.append(p))

        asyncio.run(bus.emit_async(EventType.BALL_DETECTED, "ball"))

        assert ball_calls == ["ball"]
        assert nt_calls == []

    def test_handler_exception_does_not_stop_other_handlers(self):
        bus = EventBus()
        second_called = []

        def bad_handler(payload):
            raise RuntimeError("deliberate error")

        def good_handler(payload):
            second_called.append(payload)

        bus.subscribe(EventType.COUNT_CHANGED, bad_handler)
        bus.subscribe(EventType.COUNT_CHANGED, good_handler)

        asyncio.run(bus.emit_async(EventType.COUNT_CHANGED, "test"))

        assert second_called == ["test"]

    def test_null_payload_allowed(self):
        bus = EventBus()
        received = []
        bus.subscribe(EventType.BALL_DETECTED, lambda p: received.append(p))
        asyncio.run(bus.emit_async(EventType.BALL_DETECTED))
        assert received == [None]


class TestEventBusEmitSync:
    def test_emit_sync_without_loop_logs_warning(self, caplog):
        import logging

        bus = EventBus()
        calls = []
        bus.subscribe(EventType.BALL_DETECTED, lambda p: calls.append(p))

        with caplog.at_level(logging.WARNING):
            bus.emit_sync(EventType.BALL_DETECTED, "x")

        assert calls == []  # no loop, nothing delivered

    def test_set_loop_and_emit_sync_delivers_event(self):
        bus = EventBus()
        received = []

        async def handler(payload):
            received.append(payload)

        bus.subscribe(EventType.BALL_DETECTED, handler)

        async def run():
            loop = asyncio.get_running_loop()
            bus.set_loop(loop)
            bus.emit_sync(EventType.BALL_DETECTED, "from_sync")
            # Give the coroutine a chance to run
            await asyncio.sleep(0.05)

        asyncio.run(run())
        assert received == ["from_sync"]
