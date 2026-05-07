from polymetis import DualFrankaInterfaceServer
import os
import zerorpc


if __name__ == "__main__":
    port = int(os.environ.get("DUAL_INTERFACE_PORT", "4243"))
    server = DualFrankaInterfaceServer()
    s = zerorpc.Server(server)
    s.bind(f"tcp://0.0.0.0:{port}")
    s.run()
