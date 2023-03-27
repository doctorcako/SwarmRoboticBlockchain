const express = require("express");
const app = express();
const bodyparser = require("body-parser");
app.use(bodyparser.json())
const port = 3000;
const { tessera, besu } = require("./keys.js");
const { setUp, createRoute, getRouteIndex, routeHandler, changeRouteStatus } = require("../contracts/routeController.js")
let mainContractAddress = '';

app.get("/", (req, res) => {
  res.send({ message: "Hello World!" });
});

app.listen(port, async () => {
  mainContractAddress = await setUp()
  console.log('----------------------------------')
  console.log(`* CORE API ready `);
  console.log('* This API connects private network and the swarm - It is mandatory to run just before nodes ')
  console.log('----------------------------------')

});

app.post("/createNewRoute",async (req,res)=>{
  await createRoute(besu.member1.url, mainContractAddress, req.body.location, req.body.routeName, besu.member1.accountPrivateKey, tessera.member1.publicKey, tessera.member3.publicKey)
})

app.post("/enterRoute",async (req,res)=>{
  let routeId = await getRouteIndex('Alicante','AP-7',besu.member1.url, mainContractAddress, besu.member1.accountPrivateKey, tessera.member1.publicKey, tessera.member3.publicKey)
  await routeHandler('enterRoute','Alicante','AP-7',besu.member1.url, mainContractAddress, besu.member1.accountPrivateKey, tessera.member1.publicKey, tessera.member3.publicKey, besu.member1.name, routeId)
})

app.post("/leaveRoute",async (req,res)=>{
  let routeId = await getRouteIndex('Alicante','AP-7',besu.member1.url, mainContractAddress, besu.member1.accountPrivateKey, tessera.member1.publicKey, tessera.member3.publicKey)
  await routeHandler('leaveRoute','Alicante','AP-7',besu.member1.url, mainContractAddress, besu.member1.accountPrivateKey, tessera.member1.publicKey, tessera.member3.publicKey, besu.member1.name, routeId)
})

app.post("/getRouteCars",async (req,res)=>{
  let routeId = await getRouteIndex('Alicante','AP-7',besu.member1.url, mainContractAddress, besu.member1.accountPrivateKey, tessera.member1.publicKey, tessera.member3.publicKey)
  let cars = await routeHandler('getRouteCars','Alicante','AP-7',besu.member1.url, mainContractAddress, besu.member1.accountPrivateKey, tessera.member1.publicKey, tessera.member3.publicKey, besu.member1.name, routeId)
  res.send(cars['0'])
})

app.post('/updateStatus',async(req, res)=>{
  let routeId = await getRouteIndex('Alicante','AP-7',besu.member1.url, mainContractAddress, besu.member1.accountPrivateKey, tessera.member1.publicKey, tessera.member3.publicKey)
  let receipt = await changeRouteStatus(req.body.eventType,'Alicante','AP-7',besu.member1.url, mainContractAddress, besu.member1.accountPrivateKey, tessera.member1.publicKey, tessera.member3.publicKey, besu.member1.name, routeId)

  res.json(receipt)

})


app.post('/registerEvent',async(req, res)=>{
  
})

app.post("/store",(req,res)=>{
  console.log(req.body)
})
