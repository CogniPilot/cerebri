# Zenoh Pub

## 1. Start Zenoh Router
Set the router up to listen to the zeth device.

On the host run
```bash
./scripts/zenohd.sh
```

## 2. Build and Run Zephyr
In terminal within vscode run:
```bash
west build -b native_posix -p
west build -t run
```

In vscode you can also use F5 on zenoh\_pub/src/main.c to trigger gdb debugging.

## 3. Start Python Subscriber
Run the python subscriber to see the data.

On the host run
```bash
./scripts/sub.py
```
