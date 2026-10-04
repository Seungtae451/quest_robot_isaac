"""Official OpenPI wire protocol with bounded request/connection waiting."""
import time


def connect_policy(host, port, timeout=120.):
    from openpi_client.websocket_client_policy import WebsocketClientPolicy
    from openpi_client import msgpack_numpy
    from websockets.sync.client import connect

    class TimedPolicy(WebsocketClientPolicy):
        def _wait_for_server(self):
            deadline=time.monotonic()+timeout
            while True:
                try:
                    connection=connect(self._uri,compression=None,max_size=None,
                                       open_timeout=min(10.,timeout))
                    return connection,msgpack_numpy.unpackb(connection.recv(timeout=timeout))
                except (ConnectionRefusedError,OSError):
                    if time.monotonic()>=deadline:
                        raise TimeoutError(f'Policy server did not start at {self._uri}')
                    time.sleep(.5)

        def infer(self, observation):
            self._ws.send(self._packer.pack(observation))
            response=self._ws.recv(timeout=timeout)
            if isinstance(response,str):
                raise RuntimeError(f'Policy server error:\n{response}')
            return msgpack_numpy.unpackb(response)

        def close(self):
            self._ws.close()

    return TimedPolicy(host,port)
