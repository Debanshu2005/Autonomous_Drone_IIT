import asyncio
import os
import socket
import threading
import time
import pytest
from unittest.mock import MagicMock

import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../onboard_edge')))

from pi_edge_brain import EdgeBrain


def test_unauthenticated_connection_starts_telemetry_immediately(monkeypatch):
    # Ensure no auth token is set
    monkeypatch.delenv("EDGE_BRAIN_AUTH_TOKEN", raising=False)
    
    # Use port 0 to let OS pick a free port
    brain = EdgeBrain(host="127.0.0.1", port=0)
    
    # We don't want to actually connect to MAVLink
    brain.swarm = MagicMock()
    brain.swarm.connections = {}
    
    # We will run the socket listener in a separate thread
    listener_thread = threading.Thread(target=brain._socket_listener, args=(asyncio.new_event_loop(),))
    listener_thread.daemon = True
    listener_thread.start()
    
    try:
        # Wait for the server socket to be bound and listening
        time.sleep(0.5)
        bound_port = brain.server_sock.getsockname()[1]
        
        # Connect a dummy client
        client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_sock.connect(("127.0.0.1", bound_port))
        
        # Wait a moment for accept() to process
        time.sleep(0.5)
        
        # In unauthenticated mode, client_connected should become True IMMEDIATELY
        # without the client sending any data (like an AUTH: line or heartbeat).
        assert brain.client_connected is True
        assert brain.client_sock is not None
        
    finally:
        brain._stop_event.set()
        if brain.server_sock:
            brain.server_sock.close()
        listener_thread.join(timeout=1.0)

def test_edge_brain_config_parsing():
    # Test that EdgeBrain handles expected_peers correctly and passes to HotspotContainmentConfig
    brain1 = EdgeBrain(drone_id="custom-drone-1")
    assert brain1.hotspot_config.drone_id == "custom-drone-1"
    assert brain1.hotspot_config.expected_peer_ids == []
    
    brain2 = EdgeBrain(drone_id="custom-drone-2", expected_peers=["peer1", "peer2"])
    assert brain2.hotspot_config.expected_peer_ids == ["peer1", "peer2"]
