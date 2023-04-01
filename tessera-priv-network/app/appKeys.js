
// WARNING: the keys here are demo purposes ONLY. Please use a tool like Orchestrate or EthSigner for production, rather than hard coding private keys

module.exports = {
    tessera: {
      member1: {
        publicKey: "pUy4j8OQCPurntQWxHNqG0jSKVGpbl3DWTEhwBb4oFM="
      },
      member2: {
        publicKey: "KkY65wNc+pMekAJXexSS+QgRZ8VEQLlIJYlqROlmdzI="
      },
      member3: {
        publicKey: "RvqrpHLAk91q2TnB1V/Jam9n7Imnh00lM7SwjfDrMVA="
      },
      member4:{
        publicKey:"jJoJqINiDg6XTsW1WhdRU1a+e1E1oycN1CjA6AmPEzo="
      }
    },
    besu: {
      "0x3b84348606ada5075614573c87d2f2ebfeab9689": {
        name: "manager",
        url: "http://127.0.0.1:8545",
        privateUrl: "http://127.0.0.1:9101",
        nodekey: "0x16a9e30be684ae991f66e6e9d75ba62ca6045a76dd59fe4500e29d85ecfa0183cee8466ce660314f61395401291f0a40cff63d8a829deed6a6524a5cdd1f63fa",
        accountAddress: "0x3b84348606ada5075614573c87d2f2ebfeab9689",
        accountPrivateKey: "cb4d75469a79b5cbe8282ab15a35e6d215c9e735b0b910df170827e16cead6d3",
        publicKey: "pUy4j8OQCPurntQWxHNqG0jSKVGpbl3DWTEhwBb4oFM="
      },
      "0x6fa9aae0f1fa576171e15196a0c1fe042f9b4ed3": {
        name: "car1",
        url: "http://127.0.0.1:8546",
        privateUrl: "http://127.0.0.1:9201",
        nodekey: "0x485ce8574b1371516b1bdbfbd83e753d18428bc7abba6069b6ead9f46d8b75f7a855c1463ce48fdf8fba920cc21ab3f8ecc28b0261372d655eac61575241f9c4",
        accountAddress: "0x6fa9aae0f1fa576171e15196a0c1fe042f9b4ed3",
        accountPrivateKey: "960beac5864145b73ef7c8d786f0962cc11899610e0fbf80496ade4ca0413bdc",
        publicKey: "KkY65wNc+pMekAJXexSS+QgRZ8VEQLlIJYlqROlmdzI="
      },
      "0xb3d9d1a789957306cecf42242587c00a15fb324f": {
        name: "car2",
        url: "http://127.0.0.1:8547",
        wsUrl: "ws://127.0.0.1:20005",
        privateUrl: "http://127.0.0.1:9301",
        nodekey: "0x80f73ae73cc2d86cba7cf1ba72023bfda4f1dfcf58c44f7db3da59ab5b838723cb7fcee6463077d49f996512f4b1251c9d0264fc600ae5b7636c7a7c3bec0971",
        accountAddress: "0xb3d9d1a789957306cecf42242587c00a15fb324f",
        accountPrivateKey: "d68592f5e3bd05067d5ffce2bd08de9c5f5f0dcd1373d0bfc67525109401c52e",
        publicKey: "RvqrpHLAk91q2TnB1V/Jam9n7Imnh00lM7SwjfDrMVA="

      },
      "0xd587d661abf33d0dbd6d16169ecd2443f4646cb9": {
        name: "car3",
        url: "http://127.0.0.1:8547",
        wsUrl: "ws://127.0.0.1:20005",
        privateUrl: "http://127.0.0.1:9401",
        nodekey: "0x4ca4dc7bb91ac358a162eff6772ac9714a2c6cd6becbdc88a1fe1701af4ba5f2bbcaf0d8d5bfeccc98b7756a789fb29315c8d4ab8b97edaa97b7af70005c01ea",
        accountAddress: "0xd587d661abf33d0dbd6d16169ecd2443f4646cb9",
        accountPrivateKey: "3ce207be80bf0050fb04cd063ea01b7d65ae2108b43cc9718bc1b4aa24780b55",
        publicKey:"jJoJqINiDg6XTsW1WhdRU1a+e1E1oycN1CjA6AmPEzo="
      },
      "0x34aca9b6dc968d25ff0dac87bc50cf724ef0da83":{
        name: "car_test",
        url: "http://127.0.0.1:8547",
        wsUrl: "ws://127.0.0.1:20005",
        privateUrl: "http://127.0.0.1:9401",
        nodekey: "0x4ca4dc7bb91ac358a162eff6772ac9714a2c6cd6becbdc88a1fe1701af4ba5f2bbcaf0d8d5bfeccc98b7756a789fb29315c8d4ab8b97edaa97b7af70005c01ea",
        accountAddress: "0xd587d661abf33d0dbd6d16169ecd2443f4646cb9",
        accountPrivateKey: "3ce207be80bf0050fb04cd063ea01b7d65ae2108b43cc9718bc1b4aa24780b55",
        publicKey:"jJoJqINiDg6XTsW1WhdRU1a+e1E1oycN1CjA6AmPEzo="
      },
      ethsignerProxy: {
        url: "http://127.0.0.1:18545",
        accountAddress: "9b790656b9ec0db1936ed84b3bea605873558198"
      }
    },
    nodes:{
      1:"0x3b84348606ada5075614573c87d2f2ebfeab9689",
      2:"0x6fa9aae0f1fa576171e15196a0c1fe042f9b4ed3",
      3:"0xb3d9d1a789957306cecf42242587c00a15fb324f",
      4:"0xd587d661abf33d0dbd6d16169ecd2443f4646cb9",
    },
    accounts: {
      "0xfe3b557e8fb62b89f4916b721be55ceb828dbd73" : {
        "privateKey" : "0x8f2a55949038a9610f50fb23b5883af3b4ecb3c3bb792cbcefbd1542c692be63",
      },
      "0x627306090abaB3A6e1400e9345bC60c78a8BEf57" : {
        "privateKey" : "0xc87509a1c067bbde78beb793e6fa76530b6382a4c0241e5e4a9ec0a0f44dc0d3",
        },
      "0xf17f52151EbEF6C7334FAD080c5704D77216b732" : {
        "privateKey" : "0xae6ae8e5ccbfb04590405997ee2d52d2b330726137b875053c36d94e974d162f",
        },
      }
  };