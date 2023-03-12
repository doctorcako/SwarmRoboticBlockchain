# Deploying Tessera nodes

In each NodeX/Tessera directory run:

``` 
../../../tessera/bin/tessera -configfile tessera.conf 
```


# Deploying besu nodes

## Run Node 1 (bootnode)

```
cd Node1
../../besu/bin/besu --data-path=data --genesis-file=../genesis.json --rpc-http-enabled --rpc-http-api=ETH,NET,IBFT,EEA,PRIV --host-allowlist="*" --rpc-http-cors-origins="all" --privacy-enabled --privacy-url=http://127.0.0.1:9102 --privacy-public-key-file=Tessera/nodeKey.pub --min-gas-price=0
```

## Run Node 2 
```
cd Node2
../../besu/bin/besu --data-path=data --genesis-file=../genesis.json --bootnodes=enode://16a9e30be684ae991f66e6e9d75ba62ca6045a76dd59fe4500e29d85ecfa0183cee8466ce660314f61395401291f0a40cff63d8a829deed6a6524a5cdd1f63fa@127.0.0.1:30303 --p2p-port=30304 --rpc-http-enabled --rpc-http-api=ETH,NET,IBFT,EEA,PRIV --host-allowlist="*" --rpc-http-cors-origins="all" --rpc-http-port=8546 --privacy-enabled --privacy-url=http://127.0.0.1:9202 --privacy-public-key-file=Tessera/nodeKey.pub --min-gas-price=0
```

## Run Node 3

```
cd Node3
../../besu/bin/besu --data-path=data --genesis-file=../genesis.json --bootnodes=enode://16a9e30be684ae991f66e6e9d75ba62ca6045a76dd59fe4500e29d85ecfa0183cee8466ce660314f61395401291f0a40cff63d8a829deed6a6524a5cdd1f63fa@127.0.0.1:30303 --p2p-port=30305 --rpc-http-enabled --rpc-http-api=ETH,NET,IBFT,EEA,PRIV --host-allowlist="*" --rpc-http-cors-origins="all" --rpc-http-port=8547 --privacy-enabled --privacy-url=http://127.0.0.1:9302 --privacy-public-key-file=Tessera/nodeKey.pub --min-gas-price=0
```

## Run Node 4

```
cd Node4
besu --data-path=data --genesis-file=../genesis.json --bootnodes=enode://16a9e30be684ae991f66e6e9d75ba62ca6045a76dd59fe4500e29d85ecfa0183cee8466ce660314f61395401291f0a40cff63d8a829deed6a6524a5cdd1f63fa@127.0.0.1:30303 --p2p-port=30306 --rpc-http-enabled --rpc-http-api=ETH,NET,IBFT,EEA,PRIV --host-allowlist="*" --rpc-http-cors-origins="all" --rpc-http-port=8548 --privacy-enabled --privacy-url=http://127.0.0.1:9402 --privacy-public-key-file=Tessera/nodeKey.pub --min-gas-price=0
```
