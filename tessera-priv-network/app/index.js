const express = require("express");
const app = express();
const bodyparser = require("body-parser");
app.use(bodyparser.json())
app.use(bodyparser.urlencoded({ extended: true }));

//use cors to allow cross origin resource sharing
const cors = require("cors");
app.use(cors({ origin: "http://localhost:3000", credentials: true }));



const port = 4000;
const { tessera, besu } = require("./appKeys.js");
const { setUp, 
        createRoute, 
        getRouteIndex, 
        routeHandler, 
        changeRouteStatus, 
        getRoutes, getNamesAndLocations } = require("../contracts/routeController.js")

let mainContractAddress = '';

app.get("/", (req, res) => {
  res.send({ message: "Hello World!" });
});

app.listen(port, async () => {
  mainContractAddress = await setUp()
  console.log('----------------------------------')
  console.log(`* CORE API ready `);
  console.log('* This API connects private network and the swarm - It is mandatory to run just after nodes ')
  console.log('----------------------------------')

});

app.post("/getRoutes",async (req,res)=>{
  await getRoutes(req.body.nodeRPC, 
                                mainContractAddress, 
                                req.body.fromPrivateKey, 
                                req.body.fromPublicKey, 
                                req.body.toPublicKey)
                                .then(async (result) => {
                                  await getNamesAndLocations(req.body.nodeRPC, 
                                                              mainContractAddress, 
                                                              req.body.fromPrivateKey, 
                                                              req.body.fromPublicKey, 
                                                              req.body.toPublicKey).then((namesAndLocations) => {
                                                                //marge the two arrays result and names and Locations
                                                                let finalResult = []
                                                                result[0].forEach((route, index) => {
                                                                  finalResult.push({
                                                                    routeName: namesAndLocations['0'][index],
                                                                    address: result['0'][index]
                                                                  })
                                                                })
                                                                res.json(finalResult)
                                                              })
                                })
                                .catch((err) => {
                                  console.log('Not allowed to perform transactions: '+err)
                                  res.sendStatus(403)
                                });
})

app.post("/createNewRoute",async (req,res)=>{
  await createRoute(req.body.nodeRPC, 
                    mainContractAddress, 
                    req.body.location, 
                    req.body.routeName, 
                    req.body.fromPrivateKey, 
                    req.body.fromPublicKey, 
                    req.body.toPublicKey)
                    .catch((err) => {
                      console.log('Not allowed to perform transactions: '+err)
                      res.sendStatus(403)
                    });
})

app.post("/enterRoute",async (req,res)=>{
  await getRouteIndex(req.body.location,
                      req.body.routeName,
                      req.body.nodeRPC, 
                      mainContractAddress, 
                      req.body.fromPrivateKey, 
                      req.body.fromPublicKey, 
                      req.body.toPublicKey)
                      .then(async (routeId) => {
                        await routeHandler('enterRoute',req.body.location,
                          req.body.routeName,
                          req.body.nodeRPC, 
                          mainContractAddress, 
                          req.body.fromPrivateKey, 
                          req.body.fromPublicKey, 
                          req.body.toPublicKey, 
                          routeId).then((result) => {
                            res.json(result)
                          }
                        )

                      })
                      .catch((err) => {
                        console.log('Not allowed to perform transactions: '+err)
                        res.sendStatus(403)
                      });
   
})

app.post("/leaveRoute",async (req,res)=>{
  await getRouteIndex(req.body.location,
                      req.body.routeName,
                      req.body.nodeRPC, 
                      mainContractAddress, 
                      req.body.fromPrivateKey, 
                      req.body.fromPublicKey, 
                      req.body.toPublicKey)
                      .then(async (routeId) => {
                        await routeHandler('leaveRoute',req.body.location,
                                                        req.body.routeName,
                                                        req.body.nodeRPC, 
                                                        mainContractAddress, 
                                                        req.body.fromPrivateKey, 
                                                        req.body.fromPublicKey, 
                                                        req.body.toPublicKey, 
                                                        routeId)
                      })
                      .catch((err) => {
                        console.log('Not allowed to perform transactions: '+err)
                        res.sendStatus(403)
                      });
})

app.post("/getRouteInfo",async (req,res)=>{
  const ID = await getRouteIndex(req.body.location,
                      req.body.routeName,
                      req.body.nodeRPC, 
                      mainContractAddress, 
                      req.body.fromPrivateKey, 
                      req.body.fromPublicKey, 
                      req.body.toPublicKey)

  await routeHandler('getRouteCars',req.body.location,
                req.body.routeName,
                req.body.nodeRPC, 
                mainContractAddress, 
                req.body.fromPrivateKey, 
                req.body.fromPublicKey, 
                req.body.toPublicKey, 
                ID)
            .then(async (cars) => {
              await routeHandler('getRouteStatus',req.body.location,
                                                  req.body.routeName,
                                                  req.body.nodeRPC,
                                                  mainContractAddress,
                                                  req.body.fromPrivateKey,
                                                  req.body.fromPublicKey,
                                                  req.body.toPublicKey,
                                                  ID)
                                                  .then(async (status) => {
                                                    res.json({
                                                      cars: cars['0'],
                                                      status: status['0']
                                                    })
                                                  })
                })
  .catch((err) => {
    console.log('Not allowed to perform transactions: '+err)
    res.sendStatus(403)
  });      
})

app.post('/updateStatus',async(req, res)=>{
  const ID = await getRouteIndex(req.body.location, 
                      req.body.routeName,
                      req.body.nodeRPC, 
                      mainContractAddress, 
                      req.body.fromPrivateKey, 
                      req.body.fromPublicKey, 
                      req.body.toPublicKey)

   await routeHandler('getRouteCars',req.body.location,
                req.body.routeName,
                req.body.nodeRPC, 
                mainContractAddress, 
                req.body.fromPrivateKey, 
                req.body.fromPublicKey, 
                req.body.toPublicKey, 
                ID).then(async (cars) => {
                        await changeRouteStatus(req.body.eventType, 
                                                req.body.location, 
                                                req.body.routeName, 
                                                req.body.nodeRPC, 
                                                mainContractAddress, 
                                                req.body.fromPrivateKey, 
                                                req.body.fromPublicKey, 
                                                req.body.toPublicKey,
                                                //cars['0'],
                                                ID)
                                                .then((receipt) => {
                                                  res.json(receipt)
                                                }).catch((err) => {
                                                  console.log('Not allowed to perform transactions- '+err)
                                                  res.sendStatus(403)
                                                });
                                              })
})

app.post('/getKeys',async(req, res)=>{
  
  let nodeSender = besu[`${req.body.nodeAddress}`]

  let nodeReceivers = []
  Object.values(besu).forEach((node) => {
    if(node.accountAddress != nodeSender.accountAddress){
        nodeReceivers.push(node)
    }
 })

 let publicKeys = []
 nodeReceivers.forEach((node) => {
     if(node.publicKey)
         publicKeys.push(node.publicKey)
 })

  let data = {
    sender: nodeSender,
    publicKeys: publicKeys
  }
  
  if(nodeSender){
    res.json(data)
  }else{
    res.sendStatus(403)
  }

})

