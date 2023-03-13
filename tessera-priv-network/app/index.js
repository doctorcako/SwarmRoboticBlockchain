const express = require("express");
const app = express();
const bodyparser = require("body-parser");
app.use(bodyparser.json())
const port = 3000;
const Web3 = require('web3');
const Web3Quorum = require('web3js-quorum');
const { tessera, besu } = require("../contracts/keys.js");
const { createContract, getValueAtAddress } = require("../contracts/private_tx.js");


app.get("/", (req, res) => {
  res.send({ message: "Hello World!" });
});

app.listen(port, () => {
  console.log(`app listening at http://localhost:${port}`);
});

app.post("/registerEvent",(req,res)=>{
  console.log(req.body)
})

app.post("/deploy",(req,res)=>{
  console.log(req.body)
})

app.post("/store",(req,res)=>{
  console.log(req.body)
})
