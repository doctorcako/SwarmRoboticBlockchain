const express = require("express");
const app = express();
app.use(express.json())

const port = 3000;
const https = require('https')

app.get("/", (req, res) => {
  res.send({ message: "Hello World!" });
});

app.listen(port, () => {
  console.log(`app listening at http://localhost:${port}`);
});

app.post("/registerEvent",(req,res)=>{
  console.log(req.body)
})
