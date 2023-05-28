
# Deploy Network
## Deploying Tessera nodes

In each NodeX/Tessera directory run:

``` 
cd Tessera/ && ../../../tessera/bin/tessera -configfile tessera.conf 
```


## Deploying besu nodes

### Run Node 1 (bootnode)

```
cd Node1
../../besu/bin/besu --data-path=data --genesis-file=../genesis.json --rpc-http-enabled --rpc-http-api=ETH,NET,IBFT,EEA,PRIV --host-allowlist="*" --rpc-http-cors-origins="all" --privacy-enabled --privacy-url=http://127.0.0.1:9102 --privacy-public-key-file=Tessera/nodeKey.pub --min-gas-price=0
```

### Run Node 2 
```
cd Node2
../../besu/bin/besu --data-path=data --genesis-file=../genesis.json --bootnodes=enode://16a9e30be684ae991f66e6e9d75ba62ca6045a76dd59fe4500e29d85ecfa0183cee8466ce660314f61395401291f0a40cff63d8a829deed6a6524a5cdd1f63fa@127.0.0.1:30303 --p2p-port=30304 --rpc-http-enabled --rpc-http-api=ETH,NET,IBFT,EEA,PRIV --host-allowlist="*" --rpc-http-cors-origins="all" --rpc-http-port=8546 --privacy-enabled --privacy-url=http://127.0.0.1:9202 --privacy-public-key-file=Tessera/nodeKey.pub --min-gas-price=0
```

### Run Node 3

```
cd Node3
../../besu/bin/besu --data-path=data --genesis-file=../genesis.json --bootnodes=enode://16a9e30be684ae991f66e6e9d75ba62ca6045a76dd59fe4500e29d85ecfa0183cee8466ce660314f61395401291f0a40cff63d8a829deed6a6524a5cdd1f63fa@127.0.0.1:30303 --p2p-port=30305 --rpc-http-enabled --rpc-http-api=ETH,NET,IBFT,EEA,PRIV --host-allowlist="*" --rpc-http-cors-origins="all" --rpc-http-port=8547 --privacy-enabled --privacy-url=http://127.0.0.1:9302 --privacy-public-key-file=Tessera/nodeKey.pub --min-gas-price=0
```

### Run Node 4

```
cd Node4
../../besu/bin/besu --data-path=data --genesis-file=../genesis.json --bootnodes=enode://16a9e30be684ae991f66e6e9d75ba62ca6045a76dd59fe4500e29d85ecfa0183cee8466ce660314f61395401291f0a40cff63d8a829deed6a6524a5cdd1f63fa@127.0.0.1:30303 --p2p-port=30306 --rpc-http-enabled --rpc-http-api=ETH,NET,IBFT,EEA,PRIV --host-allowlist="*" --rpc-http-cors-origins="all" --rpc-http-port=8548 --privacy-enabled --privacy-url=http://127.0.0.1:9402 --privacy-public-key-file=Tessera/nodeKey.pub --min-gas-price=0
```


# Deploy API

```
cd /app
node index.js

```

# Deploy Robots

```
cd coppeliaSimApi

Robot 0
python3 api.py 0 localhost:4000 0x6fa9aae0f1fa576171e15196a0c1fe042f9b4ed3 19999

Robot 1
python3 api.py 1 localhost:4000 0xb3d9d1a789957306cecf42242587c00a15fb324f 20000

Robot 2
python3 api.py 2 localhost:4000 0x3b84348606ada5075614573c87d2f2ebfeab9689 20001

Robot 3
python3 api.py 2 localhost:4000 0xd587d661abf33d0dbd6d16169ecd2443f4646cb9 20002

```


# Run frontend 
```
cd app/swarm-manager
npm run dev
```