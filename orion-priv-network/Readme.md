# Starting Orion nodes

```cd NodeX/Orion && ../../orion/bin/orion orion.config```


# Starting Besu nodes
```cd NodeX/  ```

## Node 1

../../besu_binaries/bin/besu --data-path=data --genesis-file=../genesis.json --rpc-http-enabled --rpc-http-api=ETH,NET,IBFT,EEA,PRIV --host-allowlist="*" --rpc-http-cors-origins="all" --privacy-enabled --privacy-url=http://127.0.0.1:8888 --privacy-public-key-file=Orion/nodekey.pub --min-gas-price=0

## Node 2
../../besu_binaries/bin/besu --data-path=data --genesis-file=../genesis.json --bootnodes=enode://b145cf99f9571ae10548301577dbbd165f28bdb7113b138ab35222e78819023207ddf0c019daae1a140cf690620507d2eeee60710dc247ca606f9e1d5669e1b9@127.0.0.1:30303 --p2p-port=30304 --rpc-http-enabled --rpc-http-api=ETH,NET,IBFT,EEA,PRIV --host-allowlist="*" --rpc-http-cors-origins="all" --rpc-http-port=8546 --privacy-enabled --privacy-url=http://127.0.0.1:8889 --privacy-public-key-file=Orion/nodekey.pub --min-gas-price=0

## Node 3
../../besu_binaries/bin/besu --data-path=data --genesis-file=../genesis.json --bootnodes=enode://b145cf99f9571ae10548301577dbbd165f28bdb7113b138ab35222e78819023207ddf0c019daae1a140cf690620507d2eeee60710dc247ca606f9e1d5669e1b9@127.0.0.1:30303 --p2p-port=30305 --rpc-http-enabled --rpc-http-api=ETH,NET,IBFT,EEA,PRIV --host-allowlist="*" --rpc-http-cors-origins="all" --rpc-http-port=8547 --privacy-enabled --privacy-url=http://127.0.0.1:8890 --privacy-public-key-file=Orion/nodekey.pub --min-gas-price=0


## Node 4
../../besu_binaries/bin/besu --data-path=data --genesis-file=../genesis.json --bootnodes=enode://b145cf99f9571ae10548301577dbbd165f28bdb7113b138ab35222e78819023207ddf0c019daae1a140cf690620507d2eeee60710dc247ca606f9e1d5669e1b9@127.0.0.1:30303 --p2p-port=30306 --rpc-http-enabled --rpc-http-api=ETH,NET,IBFT,EEA,PRIV --host-allowlist="*" --rpc-http-cors-origins="all" --rpc-http-port=8548 --privacy-enabled --privacy-url=http://127.0.0.1:8891 --privacy-public-key-file=Orion/nodekey.pub --min-gas-price=0


### Enode
enode://19a5f8731b0c35f5946cc01910f4143a342359339e7601d9433dc7dcbd9894ca1a90aa2ec51d99041ed5392cd4238b877d7a60547ed697c222ff43b3428249d1@127.0.0.1:30303